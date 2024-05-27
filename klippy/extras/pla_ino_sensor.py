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
from .protobuf_utils import protobuf_utils



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

        self.flag = 126
        self.escape = 125
        self.sequence = 0

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

    def send_temp(self):
        serial_data = protobuf_utils.create_heating_request(self.target_temp, self.sequence,self.flag)
        self.sequence += 1
        self.write_queue.append(serial_data)

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
            message = self._create_PID_message(self.pid_Ki,self.pid_Kp,self.pid_Kd)
            self.write_queue.append(message)
            
    def _run_Read(self, eventtime):
        """Readout of the incoming messages over the serial port

        :param eventtime: current event time
        :type eventtime: ?
        :return: tell reactor not to call this function any more (if not available)
        :rtype: ?
        """
        # Do non-blocking reads from serial and try to find lines
        while True:
            # try:
            raw_bytes = bytearray(self.serial.read_until(self.flag.to_bytes()))
            # except Exception as e:
            #     logging.info(f"J: error in serial readout: {e}")
            #     self.disconnect()
            #     break
            # else:
            # Decoded any escaped bytes to get the original data frame.
            output = protobuf_utils.xor_and_remove_value(raw_bytes[:-1], bytes([self.escape]))

            # Calculate the checksum, and compare it with the value in the received packet.
            checksum = output[-1]
            calculated_checksum = protobuf_utils.calculate_checksum(output[:-1])
        
            if checksum != calculated_checksum:
                logging.warning(f"checksum failed: packet: {checksum}, calculated: {calculated_checksum}")
                continue
            try:
                # Deserialize the protobuf data in to an object.
                response = ino_msg_pb2.serial_response()
                logging.info(f"output {output}")
                response.ParseFromString(bytes(output[1:-1]))
            except:
                logging.warning("failed to decode")
            else:
                message_content = response.log_msg.message
                self.read_queue.append(message_content)
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
                    logging.info(f"writing {text_line}")
                    self.serial.write(text_line)
                except Exception as e:
                    logging.info(f"J: error in serial communication (writing): {e}")
                    self.disconnect()
                    break

        # logging.info("J: Write queue is empty.")
        return eventtime + SERIAL_TIMER

    cmd_INO_PID_TUNE_help = ""

    def cmd_INO_PID_TUNE(self, gcmd):
        """custom gcode command for tuning the PID

        :param gcmd: gcode command (object) that is processed
        :type gcmd: ?
        """
        request = ino_msg_pb2.serial_request()
        request.ino_cmd.command = ino_msg_pb2.start_autotune
        serial_data = protobuf_utils.create_request(request, self.sequence,self.flag)
        self.sequence += 1
        self.write_queue.append(serial_data)

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
        ki = gcmd.get_float("Ki", 0.0)
        kd = gcmd.get_float("Kd", 0.0)
        message = self._create_PID_message(ki,kp,kd)
        self.write_queue.append(message)
    
    def _create_PID_message(self, ki, kp, kd):
        request = ino_msg_pb2.serial_request()
        self.target_temp = 0
        request.settings.target = self.target_temp
        request.settings.ki = ki
        request.settings.kp = kp
        request.settings.kd = kd
        serial_data = protobuf_utils.create_request(request, self.sequence,self.flag)
        self.sequence += 1
        return serial_data

    cmd_INO_FREQUENCY_help = "Command INO_FREQUENCY is deprecated!"

    # dead
    def cmd_INO_FREQUENCY(self, gcmd):
        logging.warning("Command INO_FREQUENCY is deprecated!")

    cmd_INO_RESET_ERROR_FLAGS_help = "Command INO_RESET_ERROR_FLAGS is deprecated!"

    # dead
    def cmd_INO_RESET_ERROR_FLAGS(self, gcmd):
        logging.warning("Command INO_RESET_ERROR_FLAGS is deprecated!")

    cmd_INO_DEBUG_OUT_help = "Command INO_DEBUG_OUT is deprecated!"

    # dead
    def cmd_INO_DEBUG_OUT(self, gcmd):
        logging.warning("Command INO_DEBUG_OUT is deprecated!")

    cmd_INO_READ_PID_VALUES_help = "Command INO_READ_PID_VALUES is deprecated!"
    # dead
    def cmd_INO_READ_PID_VALUES(self, gcmd):
        logging.info("Command INO_READ_PID_VALUES is deprecated!")

    cmd_INO_FIRMWARE_VERSION_help = "Command INO_FIRMWARE_VERSION is deprecated!"
    # dead for now
    def cmd_INO_FIRMWARE_VERSION(self, gcmd):
        logging.info("Command INO_FIRMWARE_VERSION is deprecated!")

    def _process_read_queue(self):
        # Process any decoded lines from the device
        while not len(self.read_queue) == 0:
            text_line = self.read_queue.pop(0)
            if text_line.startswith("Temp"):
                text_dict = {i.split(":")[0].strip():i.split(":")[1].strip() for i in text_line.split(",")}
                logging.info(text_dict)
                if "Temp" in text_dict:
                    self.temp = float(text_dict["Temp"])
                else:
                    logging.warning("No temperature transmitted from INO.")

            elif text_line.startswith("debug"):
                text_dict = {i.split(":")[0].strip():i.split(":")[1].strip() for i in text_line.split(",")}
                logging.info(text_dict)
                #print message to Mainsail terminal
                self.gcode.respond_info(text_dict) # output to mainsail console

            else:
                logging.info(text_line)
    """
    def _process_read_queue(self):
        # Process any decoded lines from the device
        while not len(self.read_queue) == 0:
            text_line = self.read_queue.pop(0)
            if text_line.startswith("Tick"):
                text_dict = {i.split(":")[0].strip():i.split(":")[1].strip() for i in text_line.split(",")}
                logging.info(text_dict)
                if "Temp" in text_dict:
                    self.temp = float(text_dict["Temp"])
                else:
                    logging.warning("No temperature transmitted from INO.")
            else:
                logging.info(text_line)
    """

def load_config(config):
    # Register sensor
    pheaters = config.get_printer().load_object(config, "heaters")
    logging.info(f"J: heater in ino sensor: {pheaters.heaters}")
    pheaters.add_sensor_factory("PLA_INO_SENSOR", PLA_INO_Sensor)
