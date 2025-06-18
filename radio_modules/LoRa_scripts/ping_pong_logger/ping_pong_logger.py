#!/usr/bin/env python3

import serial
import csv
import argparse
import os
from datetime import datetime

def log_serial_data(port, baud_rate, output_file):
    """
    Reads data from a specified serial port and logs it to a unique CSV file.

    Args:
        port (str): The serial port to read from (e.g., /dev/ttyACM0).
        baud_rate (int): The baud rate of the serial communication.
        output_file (str): The name of the CSV file to save data to.
    """
    serial_port = None
    csv_file = None
    
    print(f"Attempting to open serial port {port} at {baud_rate} baud.")
    print(f"Output will be saved to: {output_file}")

    try:
        # Open the serial port
        serial_port = serial.Serial(port, baud_rate, timeout=1)
        print("Serial port opened successfully.")

        # Open the CSV file for writing (in 'w' mode for a new file)
        csv_file = open(output_file, 'w', newline='')
        csv_writer = csv.writer(csv_file)
        
        # Write the header row to the new CSV file
        csv_writer.writerow(['timestamp', 'data'])
        print(f"Logging data... Press Ctrl+C to stop.")

        while True:
            # Read a line from the serial port
            line = serial_port.readline()

            if line:
                # Decode bytes to string and strip newline characters
                try:
                    data_str = line.decode('utf-8').strip()
                except UnicodeDecodeError:
                    print(f"Warning: Could not decode line: {line}")
                    continue

                # Get the current timestamp in ISO 8601 format
                timestamp = datetime.now().isoformat()
                
                # Write the timestamp and data to the CSV file
                csv_writer.writerow([timestamp, data_str])
                # We print to console to show it's working
                print(f"Logged: {timestamp}, {data_str}")

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {port}. {e}")
    except KeyboardInterrupt:
        print("\nScript interrupted by user. Closing port and file.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if serial_port and serial_port.is_open:
            serial_port.close()
            print("Serial port closed.")
        if csv_file:
            csv_file.close()
            print("CSV file closed.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Log data from a serial port to a CSV file with a unique timestamped filename.")
    parser.add_argument('port', help="The serial port to read from (e.g., /dev/ttyACM0 or COM3).")
    parser.add_argument('--baud', type=int, default=9600, help="The baud rate for the serial communication (default: 9600).")
    parser.add_argument('--prefix', default='serial_log', help="The prefix for the output CSV filename (default: serial_log).")

    args = parser.parse_args()

    # Generate a timestamp for the filename
    timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    
    # Create the full filename
    filename = f"{args.prefix}_{timestamp_str}.csv"
    
    # Start the logging process
    log_serial_data(args.port, args.baud, filename)