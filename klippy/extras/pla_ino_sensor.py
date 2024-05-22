# Control of INO hotend
#
# Copyright (C) 2023  Johannes Zischg <johannes.zischg@plasmics.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import serial
from . import bus
from serial import SerialException
from .protobuf_utils.protobuf_definitions.build import ino_msg_pb2
from .protobuf_utils.protobuf_definitions.build import pla_log_pb2
from .protobuf_utils.protobuf_definitions.build import pla_log_pb2


# from queue import Queue, Empty

# determines the timing for all interactions with INO including reading, writing and connection (attempts)
SERIAL_TIMER = 0.1


class PLA_INO_Sensor:
    """Custom class for the PLA_INO sensor"""

    def __init__(self, config):
        """The sensor is initialized, this includes especially
        - the registration for specific events (and how to handle those)
        - the configuration of INO specific G_code commands - this happens only with the first INO Sensor

        :param config: config file passed down from the heater
        :type config: ?
        """
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self.printer.add_object("pla_ino_sensor " + self.name, self)
        self.heater = None
        self.serial = None
        self.read_timer = None
        self.temp = 0.0
        self.target_temp = 0.0
        self.read_buffer = ""
        self.read_queue = []
        self.write_timer = None
        self.write_queue = []
        self._failed_connection_attempts = 0
        self._first_connect = True
        # this should be thrown away!
        self.debug_dictionaries = [None]
        self.read_from_board_outs = [None]
        #
        self.last_debug_timestamp = self.reactor.monotonic()
        self.last_debug_message = ""

        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler(
            "klippy:disconnect", self._handle_disconnect
        )
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)

        # add the gcode commands
        if "INO_FREQUENCY" in self.gcode.ready_gcode_handlers.keys():
            logging.info("J: INO Frequency already defined!")
        else:
            self.gcode.register_command(
                "INO_FREQUENCY",
                self.cmd_INO_FREQUENCY,
                desc=self.cmd_INO_FREQUENCY_help,
            )
            self.gcode.register_command(
                "INO_PID_TUNE", self.cmd_INO_PID_TUNE, desc=self.cmd_INO_PID_TUNE_help
            )
            self.gcode.register_command(
                "INO_READ_PID_VALUES",
                self.cmd_INO_READ_PID_VALUES,
                desc=self.cmd_INO_READ_PID_VALUES_help,
            )
            self.gcode.register_command(
                "INO_SET_PID_VALUES",
                self.cmd_INO_SET_PID_VALUES,
                desc=self.cmd_INO_SET_PID_VALUES_help,
            )
            self.gcode.register_command(
                "INO_RESET_ERROR_FLAGS",
                self.cmd_INO_RESET_ERROR_FLAGS,
                desc=self.cmd_INO_RESET_ERROR_FLAGS_help,
            )
            self.gcode.register_command(
                "INO_DEBUG_OUT",
                self.cmd_INO_DEBUG_OUT,
                desc=self.cmd_INO_DEBUG_OUT_help,
            )
            self.gcode.register_command(
                "INO_FIRMWARE_VERSION",
                self.cmd_INO_FIRMWARE_VERSION,
                desc=self.cmd_INO_FIRMWARE_VERSION_help,
            )
            logging.info(f"J: All Gcode commands added.")

    def make_heater_known(self, heater, config):
        """This function is called once the heater is set up - acts as a handshake between heater and sensor
        it passes the config file again and the 'process_config' function is called

        :param heater: heater object the sensor belongs to
        :type heater: heater object
        :param config: configuration file for the heater object
        :type config: ?
        """
        logging.info(f"J: heater registered in sensor: {heater}")
        self.heater = heater
        self.process_config(config)

    def process_config(self, config):
        """Reads out the config file and sets parameters necessary for the serial connection to the ino boards

        :param config: config file
        :type config: ?
        """
        self.baud = 115200
        self.serial_port = config.get("serial")
        self.pid_Kp = self.heater.pid_Kp
        self.pid_Ki = self.heater.pid_Ki
        self.pid_Kd = self.heater.pid_Kd

        self.sample_timer = self.reactor.register_timer(
            self._sample_PLA_INO, self.reactor.NOW
        )

    def _handle_connect(self):
        if self.serial is None:
            self._init_PLA_INO()

    def _handle_disconnect(self):
        logging.info("J: Klipper reports disconnect: Ino heater shutting down")
        self.disconnect("s 0")

    def _handle_shutdown(self):
        logging.info("J: Klipper reports shutdown: Ino heater shutting down")
        self._handle_disconnect()

    def disconnect(self, disconnect_message="d"):
        """Once disconnect is called, the sensor will start shutting down.
        This includes:
        - Setting the temperature of the INO heater to 0
        - closing of the serial connection to the INO board
        - Unregisters the timers from this sensor
        """
        self.write_queue.append(disconnect_message)
        try:
            self.serial.close()
            logging.info("Serial port closed due to disconnect.")
        except Exception as e:
            logging.error(f"J: Disconnection failed due to: {e}")
        self.serial = None
        try:
            self.reactor.unregister_timer(self.read_timer)
        except:
            logging.info(
                "J: Reactor read timer already unregistered before disconnection."
            )
        self.read_timer = None

        try:
            self.reactor.unregister_timer(self.write_timer)
        except:
            logging.info(
                "J: Reactor write timer already unregistered before disconnection."
            )
        self.write_timer = None

        logging.info("J: Ino heater shut down complete.")

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return SERIAL_TIMER

    def get_status(self, _):
        return {
            "temperature": round(self.temp, 2),
            "last_debug_timestamp": self.last_debug_timestamp,
            "last_debug_message": self.last_debug_message,
        }

    ### INO specifics
    def _sample_PLA_INO(self, eventtime):
        """This function is called infinitely by the reactor class every SERIAL_TIMER interval.
        Upon execution, it either tries to establish a connection to the INO OR - if connection for
        4 consecutive times was not possible, shut down the printer.

        :param eventtime: _description_
        :type eventtime: _type_
        :return: _description_
        :rtype: _type_
        """
        # logging.info(f"J: SAMPLE PLA INO CALLED WITH TIME {eventtime}")
        if self._failed_connection_attempts < 5:
            try:
                if self.serial is None:
                    self._handle_connect()
                else:
                    self.write_queue.append(f"s {self.target_temp}")
                    self.write_queue.append("d")
            except serial.SerialException:
                logging.error("Unable to communicate with Ino. Sample")
                self.temp = 0.0
        else:
            logging.info("No connection to INO possible - shutting down Klipper.")
            self.printer.invoke_shutdown(
                "Connection to INO lost and could not be reestablished!"
            )
            return self.reactor.NEVER

        current_time = self.reactor.monotonic()
        self._callback(current_time, self.temp)
        return eventtime + SERIAL_TIMER

    def _init_PLA_INO(self):
        """Initializes the INO by starting a serial connection to the ino board
        and sending the pid control parameters
        """
        try:
            self.serial = serial.Serial(self.serial_port, 115200, timeout=60)
            logging.info("Connection to Ino successfull.")
            self._failed_connection_attempts = 0
        except Exception as e:
            logging.error(
                f"Unable to connect to Ino. This was attempt number {self._failed_connection_attempts + 1}. Exception: {e}"
            )
            self._failed_connection_attempts += 1
            return

        self.write_queue = []
        self.read_queue = []

        logging.info("J: Ino queues cleared.")

        self.read_timer = self.reactor.register_timer(self._run_Read, self.reactor.NOW)
        self.write_timer = self.reactor.register_timer(
            self._run_Write, self.reactor.NOW
        )

        logging.info("Ino read/write timers started.")

        if self._first_connect:
            s = (
                "kp "
                + str(float(self.pid_Kp))
                + ";ki "
                + str(float(self.pid_Ki))
                + ";kd "
                + str(float(self.pid_Kd))
                + ";q"
            )
            self.write_queue.append(s)
            self._first_connect = False

    def _run_Read(self, eventtime):
        """Readout of the incoming messages over the serial port

        :param eventtime: current event time
        :type eventtime: ?
        :return: tell reactor not to call this function any more (if not available)
        :rtype: ?
        """
        # Do non-blocking reads from serial and try to find lines
        while True:
            try:
                raw_bytes = ""
                if self.serial.in_waiting > 0:
                    raw_bytes = self.serial.read()
            except Exception as e:
                logging.info(f"J: error in serial readout: {e}")
                self.disconnect()
                break

            if len(raw_bytes):
                text_buffer = self.read_buffer + str(raw_bytes.decode())
                while True:
                    i = text_buffer.find("\x00")
                    if i >= 0:
                        line = text_buffer[0 : i + 1]
                        self.read_queue.append(line.strip())
                        text_buffer = text_buffer[i + 1 :]
                    else:
                        break
                self.read_buffer = text_buffer

            else:
                break

        # logging.info(f"J: Read queue contents: {self.read_queue}")

        self.last_debug_timestamp = self.reactor.monotonic()
        self._process_read_queue()
        return eventtime + SERIAL_TIMER

    def _run_Write(self, eventtime):
        """Write the messages that are in the queue to the serial connection

        :param eventtime: current event time
        :type eventtime: ?
        :return: tell reactor not to call this function any more (if not available)
        :rtype: ?
        """
        while not len(self.write_queue) == 0:
            text_line = self.write_queue.pop(0)

            if text_line:
                try:
                    self.serial.write((text_line + ";\x00").encode())
                except Exception as e:
                    logging.info(f"J: error in serial communication (writing): {e}")
                    self.disconnect()
                    break

        # logging.info("J: Write queue is empty.")
        return eventtime + SERIAL_TIMER

    def _get_extruder_for_commands(self, index, gcmd):
        """lookup of the extruder the heater and sensor belong to

        :param index: extruder number
        :type index: int
        :param gcmd: gcode command (object) that is processed
        :type gcmd: ?
        """
        if index is not None:
            section = "extruder"
            if index:
                section = "extruder%d" % (index,)
            extruder = self.printer.lookup_object(section, None)
            if extruder is None:
                raise gcmd.error("Extruder not configured.")
        else:
            extruder = self.printer.lookup_object("toolhead").get_extruder()
        return extruder

    def _send_commands_to_extruder_ino(self, heater, message, gcmd):
        """write messages to queue if heater is PLA_INO heater

        :param heater: heater object
        :type heater: ?
        :param message: message that should be put in the queue
        :type message: ?
        :param gcmd: gcode command (object) that is processed
        :type gcmd: ?
        :raises gcmd.error: raises error if command can not be executed on the configured heater
        """
        if heater.__class__.__name__ == "PLA_INO_Heater":
            queue = heater.sensor.write_queue
            queue.append(message)
        else:
            raise gcmd.error("Command not defined for this heater.")

    cmd_INO_FREQUENCY_help = ""

    def cmd_INO_FREQUENCY(self, gcmd):
        """custom gcode command for changing the INO frequency

        :param gcmd: gcode command (object) that is processed
        :type gcmd: ?
        """
        t = gcmd.get_float("F", 0.0)
        index = gcmd.get_int("T", None, minval=0)
        s = "f " + str(int(t))

        extruder = self._get_extruder_for_commands(index, gcmd)
        heater = extruder.get_heater()

        self._send_commands_to_extruder_ino(heater, s, gcmd)

    cmd_INO_PID_TUNE_help = "z.B.: INO_PID_TUNE PID=250"

    def cmd_INO_PID_TUNE(self, gcmd):
        """custom gcode command for tuning the PID

        :param gcmd: gcode command (object) that is processed
        :type gcmd: ?
        """
        variable = gcmd.get_float("PID", 0.0)
        index = gcmd.get_int("T", None, minval=0)
        s = "pid " + str(float(variable))

        extruder = self._get_extruder_for_commands(index, gcmd)
        heater = extruder.get_heater()

        self._send_commands_to_extruder_ino(heater, s, gcmd)

    cmd_INO_SET_PID_VALUES_help = ""

    def cmd_INO_SET_PID_VALUES(self, gcmd):
        """custom gcode command for setting new PID values

        :param gcmd: gcode command (object) that is processed
        :type gcmd: ?
        """
        index = gcmd.get_int("T", None, minval=0)
        extruder = self._get_extruder_for_commands(index, gcmd)
        heater = extruder.get_heater()

        kp = gcmd.get_float("Kp", 0.0)
        s = "kp " + str(float(kp))
        self._send_commands_to_extruder_ino(heater, s, gcmd)

        ki = gcmd.get_float("Ki", 0.0)
        s = "ki " + str(float(ki))
        self._send_commands_to_extruder_ino(heater, s, gcmd)

        kd = gcmd.get_float("Kd", 0.0)
        s = "kd " + str(float(kd))
        self._send_commands_to_extruder_ino(heater, s, gcmd)

    cmd_INO_RESET_ERROR_FLAGS_help = "resets internel errors in INO"

    def cmd_INO_RESET_ERROR_FLAGS(self, gcmd):
        """custom gcode command for resetting the error flags

        :param gcmd: gcode command (object) that is processed
        :type gcmd: ?
        """
        index = gcmd.get_int("T", None, minval=0)
        s = "q"

        extruder = self._get_extruder_for_commands(index, gcmd)
        heater = extruder.get_heater()

        self._send_commands_to_extruder_ino(heater, s, gcmd)

    cmd_INO_DEBUG_OUT_help = "output one debug line from ino board"

    def cmd_INO_DEBUG_OUT(self, gcmd):
        """custom gcode command for outputting one debug line from the ino board

        :param gcmd: gcode command (object) that is processed
        :type gcmd: ?
        """
        index = gcmd.get_int("T", None, minval=0)

        extruder = self._get_extruder_for_commands(index, gcmd)
        heater = extruder.get_heater()
        message1 = heater.sensor.debug_dictionaries[-1]
        message2 = heater.sensor.read_from_board_outs[-1]

        self.gcode.respond_info(
            "INO debug output:\n" + str(message1) + "\n" + str(message2)
        )

    cmd_INO_READ_PID_VALUES_help = "read out internal pid values from ino board"

    def cmd_INO_READ_PID_VALUES(self, gcmd):
        """custom gcode command for reading the current PID values

        :param gcmd: gcode command (object) that is processed
        :type gcmd: ?
        """
        index = gcmd.get_int("T", None, minval=0)
        s = "a"

        extruder = self._get_extruder_for_commands(index, gcmd)
        heater = extruder.get_heater()

        self._send_commands_to_extruder_ino(heater, s, gcmd)

    cmd_INO_FIRMWARE_VERSION_help = "read out firmware version of INO"

    def cmd_INO_FIRMWARE_VERSION(self, gcmd):
        """custom gcode command for reading the firmware version of the INO

        :param gcmd: gcode command (object) that is processed
        :type gcmd: ?
        """
        index = gcmd.get_int("T", None, minval=0)
        s = "v"

        extruder = self._get_extruder_for_commands(index, gcmd)
        heater = extruder.get_heater()

        self._send_commands_to_extruder_ino(heater, s, gcmd)

    def _process_read_queue(self):
        # Process any decoded lines from the device
        while not len(self.read_queue) == 0:
            text_line = self.read_queue.pop(0)
            tmp = str(text_line.rstrip("\x00"))
            if tmp.startswith("ERROR"):
                self.gcode.respond_info(
                    "INO ERROR:)\n" + str(tmp)
                )  # output to mainsail console
                logging.info(
                    "\n--------------------ERROR: ------------------------\n"
                    + str(tmp)
                    + "\n--------------------ERROR: ------------------------\n"
                )
            elif tmp.startswith("tick:"):
                pairs = [pair.strip() for pair in tmp.split(",")]
                debug_dictionary = {}
                for pair in pairs:
                    key, value = pair.split(":")
                    debug_dictionary[key.strip()] = value.strip()

                self.last_debug_message = ",".join(
                    [str(i) for i in debug_dictionary.values()]
                )

                self.temp = int(debug_dictionary["T_a"]) / 100

                read_from_board = str(debug_dictionary["err"]).zfill(
                    6
                )  # fill left of string with zeros if not 6 long
                read_from_board_out = read_from_board

                if read_from_board[0] == "1":
                    read_from_board_out = read_from_board_out + " | open circuit"
                if read_from_board[1] == "1":
                    read_from_board_out = read_from_board_out + " | no heartbeat"
                if read_from_board[2] == "1":
                    read_from_board_out = read_from_board_out + " | heating slow"
                if read_from_board[3] == "1":
                    read_from_board_out = read_from_board_out + " | heating fast"
                if read_from_board[4] == "1":
                    read_from_board_out = read_from_board_out + " | no temp read"

                self.debug_dictionaries.append(debug_dictionary)
                self.read_from_board_outs.append(read_from_board_out)
                # temporary measure to prevent any sort of memory problems
                if len(self.debug_dictionaries) > 100:
                    self.debug_dictionaries.pop(0)
                if len(self.read_from_board_outs) > 100:
                    self.read_from_board_outs.pop(0)

            else:
                self.gcode.respond_info(f"Output from INO: {str(tmp)}")


def load_config(config):
    # Register sensor
    pheaters = config.get_printer().load_object(config, "heaters")
    logging.info(f"J: heater in ino sensor: {pheaters.heaters}")
    pheaters.add_sensor_factory("PLA_INO_SENSOR", PLA_INO_Sensor)
