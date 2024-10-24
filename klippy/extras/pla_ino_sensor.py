# Control of INO hotend
#
# Copyright (C) 2023  Johannes Zischg <johannes.zischg@plasmics.com>
# changes Marcus Roth
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import serial
from . import bus
from serial import SerialException
from serial.threaded import ReaderThread, Packetizer
from queue import Queue
from .protobuf_utils.protobuf_definitions.build import ino_msg_pb2
from .protobuf_utils.protobuf_definitions.build import pla_log_pb2
from .protobuf_utils import protobuf_utils

import time

from serial.threaded import ReaderThread, Packetizer

# from queue import Queue, Empty

# determines the timing for all interactions with INO including reading, writing and connection (attempts)
SERIAL_TIMER = 0.1

# determines the timing for heartbeat messages to the INO board
HEARTBEAT_TIMER = 0.3


class PlaInoSensor:
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
        self.printer.add_object("PLA_INO_SENSOR " + self.name, self)
        self.heater = None
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
        self.printer.register_event_handler("klippy:disconnect", self._handle_disconnect)
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)

        self.flag = 126
        self.escape = 125

        self.ino_controller = None

        self.gcode.register_command(
            "INO_PID_TUNE",
            self.cmd_INO_PID_TUNE,
            desc=self.cmd_INO_PID_TUNE_help
        )
        self.gcode.register_command(
            "INO_READ_PID_VALUES",
            self.cmd_INO_READ_PID_VALUES,
            desc=self.cmd_INO_READ_PID_VALUES_help,
        )
        self.gcode.register_command(
            "INO_RESET_ERROR",
            self.cmd_INO_RESET_ERROR,
            desc=self.cmd_INO_RESET_ERROR_help,
        )
        self.gcode.register_command(
            "INO_FIRMWARE_VERSION",
            self.cmd_INO_FIRMWARE_VERSION,
            desc=self.cmd_INO_FIRMWARE_VERSION_help,
        )
        self.gcode.register_command(
            "INO_ERROR_OUTPUT",
            self.cmd_INO_ERROR_OUTPUT,
            desc=self.cmd_INO_ERROR_OUTPUT_help,
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

        self.sample_timer = self.reactor.register_timer(self._sample_PLA_INO, self.reactor.NOW)

    def _handle_connect(self):
        if self.ino_controller is None:
            self._init_PLA_INO()

    def _handle_disconnect(self):
        logging.info("J: Klipper reports disconnect: Ino heater shutting down")
        self.disconnect()

    def _handle_shutdown(self):
        logging.info("J: Klipper reports shutdown: Ino heater shutting down")
        self._handle_disconnect()

    def disconnect(self):
        """Once disconnect is called, the sensor will start shutting down.
        This includes:
        - Setting the temperature of the INO heater to 0
        - closing of the serial connection to the INO board
        - Unregisters the timers from this sensor
        """
        self.ino_controller.heater_off_now()
        # TODO MR 24.10.2024: add same function for pid off

        try:
            self.ino_controller = None
            self.serial.close()   #TODO MR 17.10.2024: ask joey if yes or no
            logging.info("Serial port closed due to disconnect.")
        except Exception as e:
            logging.error(f"J: Disconnection failed due to: {e}")
        self.serial = None

        try:
            self.reactor.unregister_timer(self.read_timer)
        except:
            logging.info("J: Reactor read timer already unregistered before disconnection.")
        self.read_timer = None

        try:
            self.reactor.unregister_timer(self.write_timer)
        except:
            logging.info("J: Reactor write timer already unregistered before disconnection.")
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
        """function used by pla_ino_heater, sends temperature to ino board
        """
        self.ino_controller.heat_to_target_temp(self.target_temp)

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
                if self.ino_controller is None:
                    self._handle_connect()

            except serial.SerialException:
                logging.error("Unable to communicate with Ino. Sample")
                self.temp = 0.0
        else:
            logging.info("No connection to INO possible - shutting down Klipper.")
            self.printer.invoke_shutdown("Connection to INO lost and could not be reestablished!")
            return self.reactor.NEVER

        current_time = self.reactor.monotonic()
        self._callback(current_time, self.temp)
        return eventtime + SERIAL_TIMER

    def _init_PLA_INO(self):
        """Initializes the INO by starting a serial connection to the ino board"""
        try:
            self.ino_controller = InoController(self, self.serial_port)
            logging.info("Connection to Ino successful.")
            self._failed_connection_attempts = 0
        except Exception as e:
            logging.error(f"Unable to connect to Ino. This was attempt number {self._failed_connection_attempts + 1}. Exception: {e}")
            self._failed_connection_attempts += 1
            return

        self.write_queue = []
        self.read_queue = []

        logging.info("J: Ino queues cleared.")

        self.read_timer = self.reactor.register_timer(self._run_Read, self.reactor.NOW)
        self.write_timer = self.reactor.register_timer(self._run_Write, self.reactor.NOW)

        logging.info("Ino read/write timers started.")

        if self._first_connect:
            self.ino_controller.heater_off_now()
            self.ino_controller.start_pid_tuning(0)
            self.ino_controller.request_ino_reset_error()
            self.ino_controller.send_heartbeat()

    def _run_Read(self, eventtime):
        """Readout of the incoming messages over the serial port

        :param eventtime: current event time
        :type eventtime: ?
        :return: tell reactor not to call this function any more (if not available)
        :rtype: ?
        """
        responses = self.ino_controller.process_serial_data()
        for response in responses:
            self.read_queue.append(response)

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
        # TODO MR:put this in a dedicated place where it belongs. This place is temporary!
        self.ino_controller.manage_heartbeat()

        while not len(self.write_queue) == 0:
            text_line = self.write_queue.pop(0)
            if text_line:
                try:
                    logging.info(f"{timestamp()} writing {text_line}")
                    self.ino_controller.reader_thread.write(text_line)
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

    def _get_error_code(self, message):  # TODO MR make this more elegant!
        """extracts the error code from the numerated message

        :param message: message that contains the error code
        :type message: str
        """
        error_code_nr_list = message.split(" ")[1::2] #extracts error code nr and past error code nr out of message

        error_code_mapping = {
            "0": "no_error",
            "1": "heating_too_fast",
            "2": "heating_too_slow",
            "3": "exceeded_max_temp",
            "4": "no_heartbeat_received",
            "5": "temperature_unstable",
            "6": "thermocouple_disconnected",
            "7": "user_shutdown",
            "8": "subceeded_min_temp",
            "9": "temperature_read_error"
        }

        error_code = 2*[None]

        for i in range(2):
            error_code[i] = error_code_mapping.get( error_code_nr_list[i], "unknown_error")

        return "\nnow: " + error_code[0] + "  \npast: " + error_code[1]
    

    cmd_INO_PID_TUNE_help = "z.B.: INO_PID_TUNE PID=250"
    def cmd_INO_PID_TUNE(self, gcmd):
        """custom gcode command: start PID tuning with INO board

        :param gcmd: Temperature value to start PID tuning at
        """
        variable = gcmd.get_float("PID", 0.0)
        self.ino_controller.start_pid_tuning(variable)

    cmd_INO_RESET_ERROR_help = "resets INO board error"
    def cmd_INO_RESET_ERROR(self, gcmd):
        """custom gcode command: reset INO board error
        """
        self.ino_controller.request_ino_reset_error()

    cmd_INO_READ_PID_VALUES_help = "returns current ino board PID values"
    def cmd_INO_READ_PID_VALUES(self, gcmd):
        """custom gcode command: requests current PID values from INO board
        """
        self.ino_controller.request_ino_pid_values()

    cmd_INO_FIRMWARE_VERSION_help = "returns firmware version of INO board"
    def cmd_INO_FIRMWARE_VERSION(self, gcmd):
        """custom gcode command: requests firmware version from INO board
        """
        self.ino_controller.request_ino_fw_version()

    cmd_INO_ERROR_OUTPUT_help = "returns current error code of INO board"
    def cmd_INO_ERROR_OUTPUT(self, gcmd):
        """custom gcode command: requests current and last error from INO board
        """
        self.ino_controller.request_ino_error()

    # TODO MR 24.10.24 add a wait function if queue is longer than 1 maybe that will print all messages?
    def _process_read_queue(self):
        # Process any decoded lines from the device
        while not len(self.read_queue) == 0:
            first_queue_element = self.read_queue.pop(0)
            try:
                if (first_queue_element.WhichOneof("responses") == "ino_standard_msg"):  # receive standard message from ino,
                    self.temp = (first_queue_element.ino_standard_msg.temp)  # get temp from standard message
                    logging.info(f"{timestamp()} tick:{first_queue_element.ino_standard_msg.tick}, temperature:{first_queue_element.ino_standard_msg.temp}, target_temp:{first_queue_element.ino_standard_msg.temp_target}, error_code:{first_queue_element.ino_standard_msg.error_code}, status:{first_queue_element.ino_standard_msg.status}, DC:{first_queue_element.ino_standard_msg.DC}")

                elif first_queue_element.WhichOneof("responses") == "ino_general_msg":
                    if first_queue_element.ino_settings.HasField("kp"):  # check if kp is contained, only triplets of kp ki kd wil be sent
                        self.gcode.respond_info(f"Kp:{first_queue_element.ino_general_msg.kp}, Ki:{first_queue_element.ino_general_msg.ki}, Kd:{first_queue_element.ino_general_msg.kd}")

                    if first_queue_element.ino_settings.HasField("pid_tune_finished"):
                        self.gcode.respond_info(f"pid tune finished")

                    if first_queue_element.ino_settings.HasField("general_message"):
                        self.gcode.respond_info(f"general_message:{first_queue_element.ino_general_msg.general_message}")

                elif first_queue_element.WhichOneof("responses") == "log_msg":
                    if first_queue_element.log_msg.message.startswith("s_"):
                        logging.info(f"{timestamp()} {first_queue_element.log_msg.message}")

                    elif first_queue_element.log_msg.message.startswith("error_code:"):
                        self.gcode.respond_info(self._get_error_code(first_queue_element.log_msg.message))

                    else:
                        self.gcode.respond_info(f"ino: {first_queue_element.log_msg.message}")

                else:
                    logging.info(f"message not recognized: {first_queue_element}")

            except:
                logging.info(f"\nmessage not recognized: {first_queue_element}\n")

    def add_request_to_sendqueue(self, request):
        """Adds the request to the send queue that will be transmitted to ino board.

        :param request: encoded protobuf message
        :type request: ?
        """
        packetizer = PlaSerialProtocol()
        encoded_request = packetizer.encode(request)
        self.write_queue.append(encoded_request)  # rename to send_queue


def load_config(config):
    # Register sensor
    pheaters = config.get_printer().load_object(config, "heaters")
    logging.info(f"J: heater in ino sensor: {pheaters.heaters}")
    pheaters.add_sensor_factory("PLA_INO_SENSOR", PlaInoSensor)


####TODO LEE -> Organize the code better
class PlaSerialProtocol:

    def __init__(self) -> None:
        self.escape = b"\x7d"
        self.TERMINATOR = b"\x7e"

    def decode_framing(self, data):
        index = 0
        while True:
            index_value = data.find(self.escape, index)
            if index_value == -1:
                break
            if index_value < len(data) - 1:
                xor_byte = data[index_value + 1] ^ 0x20
                data = data[:index_value] + data[index_value + 1 :]
                data = data[:index_value] + bytes([xor_byte]) + data[index_value + 1 :]
            else:
                data = data[:index_value] + data[index_value + 1 :]
            index = index_value + 1
        return data

    def encode_framing(self, data):
        processed_array = bytearray()
        index = 0

        # TODO - remove magic numbers
        while index < len(data):
            current_byte = data[index]
            if current_byte == 0x7D or current_byte == 0x7E:
                xor_byte = current_byte ^ 0x20
                processed_array.append(0x7D)
                processed_array.append(xor_byte)
            else:
                processed_array.append(current_byte)
            index += 1

        return bytes(processed_array + self.TERMINATOR)

    def calculate_checksum(self, data):
        checksum = sum(data) & 0xFF
        return checksum

    def decode_protobuf(self, data):
        try:
            ino_response = ino_msg_pb2.ino_serial_response()
            ino_response.ParseFromString(data)
        except Exception as e:
            print(f"Error: {e}")
            return None

        return ino_response

    def encode(self, ino_request):
        serial_request = ino_request.SerializeToString()

        # TODO: Remove sequence number
        sequence = 0
        serial_request = bytes([sequence]) + serial_request

        checksum = self.calculate_checksum(serial_request)
        serial_request = serial_request + bytes([checksum])

        encoded_bytes = self.encode_framing(serial_request)
        return encoded_bytes

    def decode(self, data):
        decoded_bytes = self.decode_framing(data)
        checksum = decoded_bytes[-1]
        calculated_checksum = self.calculate_checksum(decoded_bytes[:-1])

        if checksum != calculated_checksum:
            print(f"Checksums do not match: {checksum} != {calculated_checksum}")
            return

        return self.decode_protobuf(bytes(decoded_bytes[1:-1]))


class PlaSerialHandler(Packetizer):

    def __init__(self) -> None:
        super().__init__()
        self.escape = b"\x7d"
        self.TERMINATOR = bytes([126])
        self.protocol_decoder = PlaSerialProtocol()
        self.queue = None

    def connection_made(self, transport):
        super(PlaSerialHandler, self).connection_made(transport)
        print("port opened", transport)

    def handle_packet(self, data):
        ino_reponse = self.protocol_decoder.decode(data)
        if self.queue:
            self.queue.put(ino_reponse)

    def set_queue(self, queue):
        self.queue = queue


class InoController:

    def __init__(self, pla_obj, serial_port) -> None:
        self.pla_obj = pla_obj
        self.serial = None
        self.queue = Queue()
        self.packet_handler = PlaSerialHandler()
        self.packet_handler.set_queue(self.queue)
        self.reader_thread = None  # Initialize to None
        self.current_target_temp = 0

        self.last_heartbeat = time.time()

        try:
            self.serial = serial.Serial(serial_port, 115200, timeout=60)
        except Exception as e:
            # print(f"Failed to open serial port: {e}")
            logging.info(f"Failed to open serial port: {e}")
            return

        # print("Connected to serial port")
        logging.info("Connected to serial port")
        self.reader_thread = ReaderThread(self.serial, lambda: self.packet_handler)
        self.reader_thread.start()

    def __del__(self):
        self.close()

    def close(self):
        if self.reader_thread is None:
            return

        print("Closing ino controller")

        self.reader_thread.stop()
        self.reader_thread.join()
        self.serial.close()

    def send_request_now(self, request):
        """sends the request now, and not add it to the send queue to be sent later

        :param request: encoded protobuf message
        :type request: _type_
        """
        packetizer = PlaSerialProtocol()
        encoded_request = packetizer.encode(request)
        self.reader_thread.write(encoded_request)

    def heat_to_target_temp(self, target_temp):
        """Heats ino to the specified target temperature.

        This function creates a protobuf message, and adds the request to the send queue.

        @param target_temp: The desired target temperature to heat to.
        @type target_temp: float
        """
        ino_request = ino_msg_pb2.user_serial_request()
        ino_request.set_settings.target_temperature = target_temp
        self.pla_obj.add_request_to_sendqueue(ino_request)
        self.current_target_temp = target_temp

    def heater_off_now(self):
        """Adds a request to the send queue to turn off the heater."""
        logging.info("Turning off heater NOW")
        ino_request = ino_msg_pb2.user_serial_request()
        ino_request.set_settings.target_temperature = 0
        self.send_request_now(ino_request)
        self.current_target_temp = 0
        logging.info("Turning off heater NOW END")

    def send_heartbeat(self):
        """sends a heartbeat message to ino board to not trigger the heartbeat error
        TODO: currently, the message is a request for the hw_version. this needs to be replaced with a dedicated message
        """
        ino_request = ino_msg_pb2.user_serial_request()
        ino_request.pla_cmd.command = ino_msg_pb2.get_hw_version
        self.pla_obj.add_request_to_sendqueue(ino_request)

    def manage_heartbeat(self):
        """needs to be executed in a loop
        checks if it is time to send a heartbeat message and sends it if necessary
        """
        if time.time() - self.last_heartbeat > HEARTBEAT_TIMER:
            self.send_heartbeat()
            self.last_heartbeat = time.time()

    def start_pid_tuning(self, target_temp):
        """send command to ino board to start PID tuning
        target_temp: the target temperature to do the PID tuning at
        """
        ino_request = ino_msg_pb2.user_serial_request()
        ino_request.set_settings.pid_target_temperature = target_temp
        self.pla_obj.add_request_to_sendqueue(ino_request)

    def request_ino_pid_values(self):
        """To read PID values etc from ino board.
        execute this function, and the ino board will return a protobuf message containing "read_info" this needs to be decoded
        """
        ino_request = ino_msg_pb2.user_serial_request()
        ino_request.pla_cmd.command = ino_msg_pb2.read_info
        self.pla_obj.add_request_to_sendqueue(ino_request)

    def request_ino_error(self):
        """To get error values etc from ino board.
        execute this function, and the ino board will return a message containing the current error and last error
        """
        ino_request = ino_msg_pb2.user_serial_request()
        ino_request.pla_cmd.command = ino_msg_pb2.read_errors
        self.pla_obj.add_request_to_sendqueue(ino_request)

    def request_ino_fw_version(self):
        """To get the firmware version from ino board.
        execute this function, and the ino board will return a message containing its firmware version
        """
        ino_request = ino_msg_pb2.user_serial_request()
        ino_request.pla_cmd.command = ino_msg_pb2.get_fw_version
        self.pla_obj.add_request_to_sendqueue(ino_request)

    def request_ino_reset_error(self):
        """request, the ino board to reset its errors"""
        ino_request = ino_msg_pb2.user_serial_request()
        ino_request.pla_cmd.command = ino_msg_pb2.clear_errors
        self.pla_obj.add_request_to_sendqueue(ino_request)

    def process_serial_data(self):
        """Processes serial data from the queue and returns responses."""
        responses = []
        try:
            while not self.queue.empty():
                ino_response = self.queue.get()
                responses.append(ino_response)
        except Exception as e:
            logging.info(f"Error processing serial data: {e}")

        return responses


def timestamp():
    """Returns a timestamp in the format HH:MM:SS.ms dd.mm.yy"""
    time_and_date = time.strftime("%H:%M:%S." + str(time.time()).split(".")[1][:4] + " %d.%m.%y", time.localtime())
    return time_and_date
