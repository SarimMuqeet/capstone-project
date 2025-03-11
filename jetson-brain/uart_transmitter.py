# uart_transmitter.py
import serial
import time
import struct

class UART_Transmitter:
    #default tested, /dev/ttyTHS1 on UART0 (pins 6, 8)
    def __init__(self, port="/dev/ttyTHS1", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
        self.connect()

    def connect(self):
        try:
            #create serial_port for connection
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                #default always
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            time.sleep(1)  # Wait for initialization
            print(f"Connected to UART port {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            print(f"Error connecting to UART: {e}")
            self.serial_port = None

    # NEW helper for pick and place object
    def send_command(self, command_type: int, x: float, y: float, z: float):
        """
        command_type: 0 = PICK, 1 = PLACE
        x,y,z: coordinates of object (float32)
        """
        if command_type not in {0, 1}:
            raise ValueError("Invalid command type (0=PICK, 1=PLACE)")
        
        # packed binary message used for faster transmission
        # Format: StartDelimiter(!) | Command(1B) | X(4B) | Y(4B) | Z(4B) | EndDelimiter(#)
        message = struct.pack('!cB3f3s', 
                            b'!',          # Start delimiter
                            command_type,  # 1 byte
                            x, y, z,       # 3 floats (4 bytes each)
                            b'##')         # End delimiter (2 bytes)
        
        self.send_message(message)

    def send_message(self, message: bytes):
        if self.serial_port and self.serial_port.is_open:
            try:
                # Write raw bytes directly - no encoding needed
                self.serial_port.write(message)
                print(f"Sent {len(message)} bytes: {message.hex()}")
            except serial.SerialException as e:
                print(f"Transmit error: {e}")
                self.handle_comm_failure(message)
        else:
            self.handle_comm_failure(message)

    def handle_comm_failure(self, message: bytes):
        print("Reconnecting...")
        self.connect()
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(message)
                print(f"Retry success: {message.hex()}")
            except serial.SerialException as e:
                print(f"Retry failed: {e}")
        else:
            print("Permanent connection failure")

    #Tested to work prev (no bytes, just encoded string):

    # def send_message(self, message):
    #     if self.serial_port is not None and self.serial_port.is_open:
    #         try:
    #             self.serial_port.write(message.encode())
    #             print(f"Sent message: {message}")
    #         except serial.SerialException as e:
    #             print(f"Error sending message: {e}")
    #     else:
    #         print("UART not connected.  Attempting to reconnect...")
    #         # Try to reconnect
    #         self.connect()
    #         if self.serial_port is not None and self.serial_port.is_open:
    #             try:
    #                 self.serial_port.write(message.encode())
    #                 print(f"Sent message: {message}")
    #             except serial.SerialException as e:
    #                 print(f"Error sending message: {e}")
    #         else:
    #             print("UART not connected.  Message not sent.")

    def close(self):
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()
            print("UART connection closed.")


if __name__ == '__main__':
    # Initialize UART connection
    uart = UART_Transmitter(port="/dev/ttyTHS1", baudrate=115200)

    # Test values
    command_type = 0
    # command_type = 1  # 0 = PICK, 1 = PLACE


    #example PICK: -20, 20
    #platform: -10, 8
    #table destination: -18, 25


    x, y, z = -15, 20, -5.25
    #-20, 20, -5.25  # Example XYZ coordinates
    # x, y, z = -18, 25, 0

    # Send test command
    uart.send_command(command_type, x, y, z)

    # Allow some time for transmission
    time.sleep(1)

    # Close UART connection
    uart.close()


# if __name__ == '__main__':
#     # Example usage (for testing)
#     uart_comm = UART_Transmitter()
#     try:
#         uart_comm.send_message("Test message from uart_comms.py\r\n")
#         time.sleep(2)
#     finally:
#         uart_comm.close()
