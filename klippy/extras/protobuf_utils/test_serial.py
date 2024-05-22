import serial
import serial.threaded
import ino_msg_pb2
import pla_log_pb2
import msvcrt
import sys
import re
import csv
import time

#Remove any escape characters and restore the orignal data.
def xor_and_remove_value(byte_array, value_to_find):
    """
    XOR the byte following each occurrence of a specified value in a byte array with 0x20,
    and remove the found values from the byte array.

    Parameters:
        byte_array (bytes): The byte array in which to perform operations.
        value_to_find (bytes): The value to search for.

    Returns:
        bytes: The modified byte array.
    """
    index = 0
    while True:
        index_value = byte_array.find(value_to_find, index)
        if index_value == -1:
            break
        if index_value < len(byte_array) - 1:
            xor_byte = byte_array[index_value + 1] ^ 0x20
            byte_array = byte_array[:index_value] + byte_array[index_value + 1:]
            byte_array = byte_array[:index_value] + bytes([xor_byte]) + byte_array[index_value + 1:]
        else:
            byte_array = byte_array[:index_value] + byte_array[index_value + 1:]
        index = index_value + 1
    return byte_array

#Escape reserved characters in the byte_array.
def process_byte_array(byte_array):
    """
    Search a byte array for the hex values 0x7D or 0x7E. Replace the value with itself XOR with 0x20
    and insert 0x7D at the index before it. Ensure that the inserted 0x7D is not also replaced.

    Parameters:
        byte_array (bytes): The byte array to process.

    Returns:
        bytes: The modified byte array.
    """
    processed_array = bytearray()
    index = 0

    while index < len(byte_array):
        current_byte = byte_array[index]
        if current_byte == 0x7D or current_byte == 0x7E:
            xor_byte = current_byte ^ 0x20
            processed_array.append(0x7D)
            processed_array.append(xor_byte)
        else:
            processed_array.append(current_byte)
        index += 1

    return bytes(processed_array)


def calculate_checksum(byte_array):
    checksum = sum(byte_array) & 0xFF
    return checksum


def parse_tick_and_temperature(input_string):
    """
    Parse a string to extract the integer values of tick and temperature.

    Parameters:
        input_string (str): The input string in the format 'Tick: xxxx, Temperature: x.xxxxxx'.

    Returns:
        tuple: A tuple containing the integer values of tick and temperature.
    """
    # Define the regular expression pattern
    pattern = r'Tick:\s*(\d+),\s*Temperature:\s*([\d\.]+)'
    
    # Search for the pattern in the input string
    match = re.search(pattern, input_string)
    
    if match:
        tick = int(match.group(1))
        temperature = float(match.group(2))
        return tick, temperature
    else:
        raise ValueError("Input string is not in the expected format.")


def write_tick_and_temperature_to_csv(tick, temperature, filename='output.csv'):
    """
    Write tick and temperature values to a CSV file.

    Parameters:
        tick (int): The tick value to write.
        temperature (float): The temperature value to write.
        filename (str): The name of the CSV file. Default is 'output.csv'.
    """
    # Open the CSV file in append mode
    with open(filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        # Write the tick and temperature values to the CSV file
        writer.writerow([tick, temperature])


def create_request(request, sequence):
    # Serialize the object using protobuf
    serial_request = request.SerializeToString()

    # Add sequence byte to the start
    serial_request = bytes([sequence]) + serial_request
    sequence = (sequence + 1) & 0xFF

    # Calcualte checksum and add it to the end
    checksum = calculate_checksum(serial_request)
    serial_request = serial_request + bytes([checksum])

    # Escape reserved characters (0x7E/0x7D) and add the flag 0x7D byte to the end of the sequence
    processed = process_byte_array(serial_request) + bytes([flag])
    return processed


def create_heating_request(target, sequence):
    # Create an object for the message data and populate the neccessary fields.
    request = ino_msg_pb2.serial_request()
    request.settings.target = target

    # Call the function to serialize and frame the data.
    serial_data = create_request(request, sequence)
    print(serial_data.hex())
    return serial_data


# Simple application to control ino to heat to different target temperatures and record the measured temperature.
if __name__ == "__main__":

    flag = 126
    escape = 125
    sequence = 0

    targets = [150, 200, 250, 200, 150]
    # targets = [250, 150, 250, 150]
    index = 0

    heating_target = 250
    cooling_target = 100

    heating = 0
    repeats = 4

    holding_temp = 1
    hold_temp_end = 0


    # Connect to the serial port
    ser = serial.Serial('COM4', 115200, timeout=60)

    while True:
        # Receive bytes until the flag 0x7E is found
        ser_bytes = bytearray(ser.read_until(flag.to_bytes()))

        # Decoded any escaped bytes to get the original data frame.
        output = xor_and_remove_value(ser_bytes[:-1], bytes([escape]))

        # Calculate the checksum, and compare it with the value in the received packet.
        checksum = output[-1]
        calculated_checksum = calculate_checksum(output[:-1])
    
        if checksum != calculated_checksum :
            print(f"checksum failed: packet: {checksum}, calculated: {calculated_checksum}")
            print(ser_bytes.hex())
            continue

        try:
            # Deserialize the protobuf data in to an object.
            response = ino_msg_pb2.serial_response()
            response.ParseFromString(bytes(output[1:-1]))
        except:
            print("failed to decode")


        if response.WhichOneof('responses') == 'settings':
            if response.result == ino_msg_pb2.ack:
                print("command acknowledged")
            else:
                print("fail")


        if response.WhichOneof('responses') == 'log_msg':
            tick, temp = parse_tick_and_temperature(response.log_msg.message)

            # Save logs to a file.
            write_tick_and_temperature_to_csv(tick, temp)
            print(f"{tick} : {temp}")
            
            if holding_temp:
                if time.time() > hold_temp_end:
                    holding_temp = 0
                    print(f"heating to {heating_target}!")
                    ser.write(create_heating_request(heating_target, sequence))
                    sequence = (sequence + 1) & 0xFF
                
                else:
                    continue

            # Check if we hit the target and set the new target.
            if heating:
                if heating_target - 0.5 <= temp <= heating_target + 0.5:
                    index = index + 1
                    if index >= len(targets):
                        heating = 0
                        print("cooling!")
                        ser.write(create_heating_request(0, sequence))
                        sequence = (sequence + 1) & 0xFF
                    else:
                        hold_temp_end = time.time() + 2
                        heating_target = targets[index]
                        holding_temp = 1

            else:
                if temp <= cooling_target:

                    index = 0
                    heating_target = targets[index]

                    repeats = repeats - 1
                    # check if we're done! 
                    if repeats == 0:
                        ser.close()
                        sys.exit("Success!")

                    heating = 1
                    print(f"heating to {heating_target}!")
                    ser.write(create_heating_request(heating_target, sequence))
                    sequence = (sequence + 1) & 0xFF





