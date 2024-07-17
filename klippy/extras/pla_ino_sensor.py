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
        # self.serial = None
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

        self.ino_controller = None

        # add the gcode commands
        if "INO_FREQUENCY" in self.gcode.ready_gcode_handlers.keys():   #MR TODO is the frequency if needed? frequency is never set
            logging.info("J: INO Frequency already defined!")
        else:
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
        self.pid_Kp = self.heater.pid_Kp
        self.pid_Ki = self.heater.pid_Ki
        self.pid_Kd = self.heater.pid_Kd

        self.sample_timer = self.reactor.register_timer(
            self._sample_PLA_INO, self.reactor.NOW
        )

    def _handle_connect(self):
        if self.ino_controller is None:
            self._init_PLA_INO()
        # if self.serial is None:
        #     self._init_PLA_INO()

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
            self.ino_controller = None
            # self.serial.close()
            logging.info("Serial port closed due to disconnect.")
        except Exception as e:
            logging.error(f"J: Disconnection failed due to: {e}")
        self.serial = None
        try:
            self.reactor.unregister_timer(self.read_timer)
        except:
            logging.info( "J: Reactor read timer already unregistered before disconnection." )
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
        self.ino_controller.heat_to_target_temp(self.target_temp)
        # serial_data = protobuf_utils.create_heating_request(self.target_temp, self.sequence,self.flag)
        # self.sequence += 1
        # self.write_queue.append(serial_data)

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
                # else:
                #     self.send_temp()
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
        and sending pid control parameters to the ino board
        """
        try:
            self.ino_controller = InoController(self.serial_port)
            # self.serial = serial.Serial(self.serial_port, 115200, timeout=1) #todo check timeout value 1=1s?
            logging.info("Connection to Ino successful.")
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
            #message = self._create_PID_message(self.pid_Kp,self.pid_Ki,self.pid_Kd)     #transmits PID vales stored in printer.cfg to ino
            #self.write_queue.append(message)

            message = self._create_error_reset_message()                                #resets error code in ino
            self.write_queue.append(message)



            
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

        # TODO -> Delete the other code in this function

        # Do non-blocking reads from serial and try to find lines
        while True:
            try:
                raw_bytes = bytearray(self.serial.read_until(self.flag.to_bytes()))
                logging.info("while true: log")
            except Exception as e:
                 logging.info(f"J: error in serial readout: {e}")
                 self.disconnect()
                 break
            else:
                #Decoded any escaped bytes to get the original data frame.
                output = protobuf_utils.xor_and_remove_value(raw_bytes[:-1], bytes([self.escape]))

            # Calculate the checksum, and compare it with the value in the received packet.
            checksum = output[-1]
            calculated_checksum = protobuf_utils.calculate_checksum(output[:-1])
        
            if checksum != calculated_checksum:
                logging.warning(f"checksum failed: packet: {checksum}, calculated: {calculated_checksum}")
                continue
            

            #try:
            #    # Deserialize the protobuf data in to an object.
            #    response = ino_msg_pb2.serial_response()
            #    logging.info(f"output {output}")
            #    response.ParseFromString(bytes(output[1:-1]))
            #except:
            #    logging.warning("failed to decode")
            #else:
            #    message_content = response.log_msg.message
            #    self.read_queue.append(message_content)
            #    break

            try:
                # Deserialize the protobuf data in to an object.
                response = ino_msg_pb2.ino_serial_response()
                #logging.info(f"output {output}")            
                response.ParseFromString(bytes(output[1:-1]))
            except:
                print("failed to decode")
            else:
                logging.info(f"response from ino: {response}")
                self.read_queue.append(response)
                break
            """
            if response.WhichOneof('responses') == 'ino_standard_msg':
                
                #print to mainsail console for testing
                self.gcode.respond_info(f"tick:{response.ino_standard_msg.tick}, temperature:{response.ino_standard_msg.temp}, target_temp:{response.ino_standard_msg.temp_target}, error_code:{response.ino_standard_msg.temp_target}, status:{response.ino_standard_msg.status}, DC:{response.ino_standard_msg.DC}")
                self.temp = response.ino_standard_msg.temp
                break
            """
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
        self.ino_controller.manage_heartbeat()  #TODO put this in a dedicated place where it belongs. This place is temporary!

        while not len(self.write_queue) == 0:
            text_line = self.write_queue.pop(0)
            if text_line:
                try:
                    logging.info(f"writing {text_line}")
                    self.ino_controller.reader_thread.write(text_line)
                    # self.serial.write(text_line)
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


    def _get_error_code(self, message): #TODO MR make this more elegant!
        """extracts the error code from the numerated message

        :param message: message that contains the error code
        :type message: str
        """
        error_code_nr = message.split(" ")[1]

        if error_code_nr == "0":
            error_code = "user_shutdown"
        elif error_code_nr == "1":
            error_code = "heating_too_fast"
        elif error_code_nr == "2":
            error_code = "heating_too_slow"
        elif error_code_nr == "3":
            error_code = "exceeded_max_temp"
        elif error_code_nr == "4":
            error_code = "no_heartbeat_received"
        elif error_code_nr == "5":
            error_code = "temperature_unstable"
        elif error_code_nr == "6":
            error_code = "thermocouple_disconnected"
        elif error_code_nr == "7":
            error_code = "no_error"


        past_error_code_nr = message.split(" ")[3]

        if past_error_code_nr == "0":
            past_error_code = "user_shutdown"
        elif past_error_code_nr == "1":
            past_error_code = "heating_too_fast"
        elif past_error_code_nr == "2":
            past_error_code = "heating_too_slow"
        elif past_error_code_nr == "3":
            past_error_code = "exceeded_max_temp"
        elif past_error_code_nr == "4":
            past_error_code = "no_heartbeat_received"
        elif past_error_code_nr == "5":
            past_error_code = "temperature_unstable"
        elif past_error_code_nr == "6":
            past_error_code = "thermocouple_disconnected"
        elif past_error_code_nr == "7":
            past_error_code = "no_error"

        return "\nnow: " + error_code + "  \npast: " + past_error_code


    cmd_INO_PID_TUNE_help = "z.B.: INO_PID_TUNE PID=250"
    def cmd_INO_PID_TUNE(self, gcmd):
        """custom gcode command for tuning the PID parameters of the ino board

        :param gcmd: gcode command (object) that is processed
        :type gcmd: ?
        """

        variable = gcmd.get_float('PID', 0.)

        self.ino_controller.start_pid_tuning(variable)

        # request = ino_msg_pb2.user_serial_request()
        # request.set_settings.pid_target_temperature = variable
        # serial_data = protobuf_utils.create_request(request, self.sequence,self.flag)
        # self.sequence += 1
        # self.write_queue.append(serial_data)



    cmd_INO_SET_PID_VALUES_help = "z.B.: INO_SET_PID_VALUES T=0 KP=1.0 KI=2.1 KD=3.2"
    def cmd_INO_SET_PID_VALUES(self, gcmd):
        """custom gcode command for setting new PID values

        :param gcmd: gcode command (object) that is processed
        :type gcmd: ?
        """
        index = gcmd.get_int('T', None, minval=0)
        extruder = self._get_extruder_for_commands(index, gcmd)
        heater = extruder.get_heater()

        kp = gcmd.get_float('KP', 0.0)
        ki = gcmd.get_float('KI', 0.0)
        kd = gcmd.get_float('KD', 0.0)
        message = self._create_PID_message(kp,ki,kd)

        # TODO Lee
        # self.write_queue.append(message)



    def _create_PID_message(self, kp, ki, kd):      #MR TODO: rename this to: send PID values to ino    #MR TODO: print the pid values to console to find out if the correct values are taken from printer.cfg
        """custom gcode command to send PID values that are saved in printer.cfg to INO board

        :param gcmd: gcode command (object) that is processed
        :type gcmd: ?
        """
        request = ino_msg_pb2.user_serial_request()
        request.set_su_values.kp = kp
        request.set_su_values.ki = ki
        request.set_su_values.kd = kd

        serial_data = protobuf_utils.create_request(request, self.sequence,self.flag)
        self.sequence += 1
        #self.write_queue.append(serial_data)
        return serial_data



    cmd_INO_RESET_ERROR_FLAGS_help = "resets INO board error flags"
    def cmd_INO_RESET_ERROR_FLAGS(self, gcmd):
        """custom gcode command the reset error modes in the INO board

        :param gcmd: gcode command (object) that is processed
        :type gcmd: ?
        """
        # request = ino_msg_pb2.user_serial_request()
        # request.pla_cmd.command = ino_msg_pb2.clear_errors

        # serial_data = protobuf_utils.create_request(request, self.sequence,self.flag)
        # self.sequence += 1
        # self.write_queue.append(serial_data)

        self.ino_controller.request_ino_reset_error()


    def _create_error_reset_message(self):    
        """custom gcode command to reset errors in INO board

        :param gcmd: gcode command (object) that is processed
        :type gcmd: ?
        """
        request = ino_msg_pb2.user_serial_request()
        request.pla_cmd.command = ino_msg_pb2.clear_errors

        serial_data = protobuf_utils.create_request(request, self.sequence,self.flag)
        self.sequence += 1
        #self.write_queue.append(serial_data)
        return serial_data

    # dead
    cmd_INO_DEBUG_OUT_help = "Command INO_DEBUG_OUT is deprecated!"
    def cmd_INO_DEBUG_OUT(self, gcmd):

        logging.warning("Command INO_DEBUG_OUT is deprecated!")


    cmd_INO_READ_PID_VALUES_help = "returns current ino board PID values"
    def cmd_INO_READ_PID_VALUES(self, gcmd):
        request = ino_msg_pb2.user_serial_request()
        request.pla_cmd.command = ino_msg_pb2.read_info #MR TODO: change read_info to better name, but needs to be implemented in Protobuf

        serial_data = protobuf_utils.create_request(request, self.sequence,self.flag)
        self.sequence += 1
        self.write_queue.append(serial_data)
        # TODO Lee?


    cmd_INO_FIRMWARE_VERSION_help = "returns firmware version of INO board"
    def cmd_INO_FIRMWARE_VERSION(self, gcmd):
        # request = ino_msg_pb2.user_serial_request()
        # request.pla_cmd.command = ino_msg_pb2.get_fw_version

        # serial_data = protobuf_utils.create_request(request, self.sequence,self.flag)
        # self.sequence += 1
        # self.write_queue.append(serial_data)
        self.ino_controller.request_ino_fw_version()



    cmd_INO_ERROR_OUTPUT_help = "returns current error code of INO board"
    def cmd_INO_ERROR_OUTPUT(self, gcmd):
        # request = ino_msg_pb2.user_serial_request()
        # request.pla_cmd.command = ino_msg_pb2.read_errors

        # serial_data = protobuf_utils.create_request(request, self.sequence,self.flag)
        # self.sequence += 1
        # self.write_queue.append(serial_data)
        self.ino_controller.request_ino_error()



    def _process_read_queue(self):
        # Process any decoded lines from the device
        while not len(self.read_queue) == 0:   
            first_queue_element = self.read_queue.pop(0)
            try:
                if first_queue_element.WhichOneof('responses') == 'ino_standard_msg':    # receive standard message from ino, 
                    self.temp = first_queue_element.ino_standard_msg.temp                # get temp from standard message

                    logging.info(f"tick:{first_queue_element.ino_standard_msg.tick}, temperature:{first_queue_element.ino_standard_msg.temp}, target_temp:{first_queue_element.ino_standard_msg.temp_target}, error_code:{first_queue_element.ino_standard_msg.error_code}, status:{first_queue_element.ino_standard_msg.status}, DC:{first_queue_element.ino_standard_msg.DC}")

                    #print to mainsail console for testing  
                    #self.gcode.respond_info(f"tick:{first_queue_element.ino_standard_msg.tick}, temperature:{first_queue_element.ino_standard_msg.temp}, target_temp:{first_queue_element.ino_standard_msg.temp_target}, error_code:{first_queue_element.ino_standard_msg.temp_target}, status:{first_queue_element.ino_standard_msg.status}, DC:{first_queue_element.ino_standard_msg.DC}")
                    #logging.info("text") # to log stuff in klippy log

                    """
                elif first_queue_element.WhichOneof('responses') == 'ino_settings':
                    #ino_target_temperature = first_queue_element.ino_settings.target_temperature
                    #ino_pid_target_temperature = first_queue_element.ino_settings.pid_target_temperature

                    self.gcode.respond_info(f"ino_target_temperature:{first_queue_element.ino_settings.target_temperature}, ino_pid_target_temperature:{first_queue_element.ino_settings.pid_target_temperature}")

                elif first_queue_element.WhichOneof('responses') == 'ino_su_settings':
                    #stuff
                    pass
                    """

                elif first_queue_element.WhichOneof('responses') == 'ino_general_msg':
                    if first_queue_element.ino_settings.HasField('kp'):  #check if kp is contained, only triplets of kp ki kd wil be sent
                        self.gcode.respond_info(f"Kp:{first_queue_element.ino_general_msg.kp}, Ki:{first_queue_element.ino_general_msg.ki}, Kd:{first_queue_element.ino_general_msg.kd}")

                    if first_queue_element.ino_settings.HasField('pid_tune_finished'):  
                        self.gcode.respond_info(f"pid tune finished")
                    
                    if first_queue_element.ino_settings.HasField('general_message'): 
                        self.gcode.respond_info(f"general_message:{first_queue_element.ino_general_msg.general_message}")


                elif first_queue_element.WhichOneof('responses') == 'log_msg':
                    #self.gcode.respond_info(f"ino_message:{first_queue_element.log_msg.message}, log_level:{first_queue_element.log_msg.log_lvl}")
                    self.gcode.respond_info(f"ino: {first_queue_element.log_msg.message}")

                    if (first_queue_element.log_msg.message.startswith("error_code:")):
                        self.gcode.respond_info( self._get_error_code(first_queue_element.log_msg.message) )

                else:
                    logging.info(f"message not recognized: {first_queue_element}")
                
            except:
                logging.info(f"\nmessage not recognized: {first_queue_element}\n")

           


def load_config(config):
    # Register sensor
    pheaters = config.get_printer().load_object(config, "heaters")
    logging.info(f"J: heater in ino sensor: {pheaters.heaters}")
    pheaters.add_sensor_factory("PLA_INO_SENSOR", PLA_INO_Sensor)



####TODO LEE -> Organize the code better


class PlaSerialProtocol():

    def __init__(self) -> None:
        self.escape = b'\x7d'
        self.TERMINATOR = b'\x7e'

    def decode_framing(self, data):
        index = 0
        while True:
            index_value = data.find(self.escape, index)
            if index_value == -1:
                break
            if index_value < len(data) - 1:
                xor_byte = data[index_value + 1] ^ 0x20
                data = data[:index_value] + data[index_value + 1:]
                data = data[:index_value] + bytes([xor_byte]) + data[index_value + 1:]
            else:
                data = data[:index_value] + data[index_value + 1:]
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
        sequence = 0 
        # TODO: Remove sequence number
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
        self.escape = b'\x7d'
        self.TERMINATOR = bytes([126])
        self.protocol_decoder = PlaSerialProtocol()
        self.queue = None

    def connection_made(self, transport):
        super(PlaSerialHandler, self).connection_made(transport)
        print('port opened', transport)


    def handle_packet(self, data):
        ino_reponse = self.protocol_decoder.decode(data)
        if self.queue:
            self.queue.put(ino_reponse)


    def set_queue(self, queue):
        self.queue = queue


class InoController():

    def __init__(self, serial_port) -> None:
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


    def send_request(self, request):
        packetizer = PlaSerialProtocol()
        encoded_request = packetizer.encode(request)
        self.reader_thread.write(encoded_request)
        # print(f"Sent request: {encoded_request.hex()}")

    def heat_to_target_temp(self, target_temp):
        ino_request = ino_msg_pb2.user_serial_request()
        ino_request.set_settings.target_temperature = target_temp
        self.send_request(ino_request)
        self.current_target_temp = target_temp

    def heater_off(self):
        ino_request = ino_msg_pb2.user_serial_request()
        ino_request.set_settings.target_temperature = 0
        self.send_request(ino_request)
        self.current_target_temp = 0

    def send_heartbeat(self):
        """
        sends a heartbeat message to ino board to not trigger the heartbeat error
        TODO: currently, the message is the temperature. Add a dedicated heartbeat message that can be sent over protobuff
        """
        ino_request = ino_msg_pb2.user_serial_request()
        ino_request.set_settings.target_temperature = self.current_target_temp
        self.send_request(ino_request) 

    def manage_heartbeat(self):
        """
        needs to be executed in a loop
        checks if it is time to send a heartbeat message and sends it if necessary
        """
        if time.time() - self.last_heartbeat > HEARTBEAT_TIMER:
            self.send_heartbeat()
            self.last_heartbeat = time.time() 

    def start_pid_tuning(self, target_temp):   
        ino_request = ino_msg_pb2.user_serial_request()
        ino_request.set_settings.pid_target_temperature = target_temp
        self.send_request(ino_request)

    def request_read_info(self):
        """
        To read PID values etc from ino board.
        execute this function, and the ino board will return a protobuff message containing "read_info" this needs to be decoded
        """
        ino_request = ino_msg_pb2.user_serial_request()
        ino_request.pla_cmd.command = ino_msg_pb2.read_info
        self.send_request(ino_request)

    def request_ino_error(self):
        """
        To get error values etc from ino board.
        execute this function, and the ino board will return a message containing the current error and last error
        """
        ino_request = ino_msg_pb2.user_serial_request()
        ino_request.pla_cmd.command = ino_msg_pb2.read_errors
        self.send_request(ino_request)

    def request_ino_fw_version(self):
        """
        To get the firmware version from ino board.
        execute this function, and the ino board will return a message containing its firmware version
        """
        ino_request = ino_msg_pb2.user_serial_request()
        ino_request.pla_cmd.command = ino_msg_pb2.get_fw_version
        self.send_request(ino_request)

    def request_ino_reset_error(self):
        """
        To get the firmware version from ino board.
        execute this function, and the ino board will return a message containing its firmware version
        """
        ino_request = ino_msg_pb2.user_serial_request()
        ino_request.pla_cmd.command = ino_msg_pb2.clear_errors
        self.send_request(ino_request)

    def set_inoboard_pid_values(self, kp, ki, kd):
        """
        set pid values in the ino board
        """
        ino_request = ino_msg_pb2.user_serial_request()
        ino_request.set_su_values.kp = kp
        ino_request.set_su_values.ki = ki
        ino_request.set_su_values.kd = kd
        self.send_request(ino_request)


    def process_serial_data(self):
        responses = []
        while not self.queue.empty():
            ino_response = self.queue.get()
            responses.append(ino_response)

        return responses


if __name__ == '__main__':
    ic = ino_controller('COM3')