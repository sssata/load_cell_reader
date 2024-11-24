import serial
import csv
import time
from datetime import datetime
import argparse

def setup_serial_port(port, baud_rate=115200):
    """Initialize serial port connection"""
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        raise

def create_output_file():
    """Create a CSV file with timestamp in filename"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"sensor_data_{timestamp}.csv"
    return open(filename, 'w', newline='')

def main(port, baud_rate=115200):
    # Column headers based on the C++ code output
    headers = [
        'timestamp_us',
        # 'reading_number',
        'raw_reading',
        # 'reading',
        # 'interrupt_duration_us'
    ]

    ser = setup_serial_port(port, baud_rate)
    csv_file = create_output_file()
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(headers)

    print(f"Logging data from {port} to CSV...")
    print("Press Ctrl+C to stop logging")

    try:
        while True:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8').strip()
                
                # Skip empty lines and "No readings yet" messages
                if not line or "No readings yet" in line:
                    continue

                # Split the TSV data and convert to appropriate types
                try:
                    values = line.split('\t')
                    if len(values) == len(headers):
                        # Convert values to appropriate types
                        processed_values = [
                            int(values[0]),    # timestamp_us
                            # int(values[1]),    # reading_number
                            int(values[1]),    # raw_reading
                            # float(values[3]),  # reading
                            # int(values[4])     # interrupt_duration_us
                        ]
                        csv_writer.writerow(processed_values)
                        csv_file.flush()  # Ensure data is written immediately
                except (ValueError, IndexError) as e:
                    print(f"Error processing line: {line}")
                    print(f"Error details: {e}")
                    continue

    except KeyboardInterrupt:
        print("\nLogging stopped by user")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        ser.close()
        csv_file.close()
        print("Serial port and CSV file closed")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Log sensor data from serial port to CSV file')
    parser.add_argument('-p', '--port', type=str, help='Serial port (e.g., COM3 on Windows or /dev/ttyUSB0 on Linux)')
    parser.add_argument('-b','--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    
    args = parser.parse_args()
    main(args.port, args.baud)