import serial
import time
import binascii
import struct


class RFIDReaderTCP:
    def __init__(self, ip, port, baudrate, debug=True):
        self.ip = ip
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.debug = debug

    def _debug_print(self, message):
        """ Only print debug messages if debug mode is enabled. """
        if self.debug:
            print(message)

    def _calculate_crc16(self, data):
        """
        Calculate CRC16 as used by the reader (matching the C implementation):
        - preset: 0xFFFF
        - polynomial: 0x8408 (reflected CRC-16-CCITT for LSB first)
        The algorithm XORs each input byte into the CRC and shifts right 8 times,
        XORing the polynomial when the LSB is set (same as the C sample).
        """
        crc = 0xFFFF
        POLY = 0x8408
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ POLY
                else:
                    crc >>= 1
        return crc & 0xFFFF

    def connect(self):
        """Establish TCP connection to the reader."""
        try:
            # Use pyserial's socket handler to treat TCP as a serial port
            url = f"socket://{self.ip}:{self.port}"
            self._debug_print(f"Attempting to connect to {url}...")
            # The baudrate here is a formality for pyserial's socket://,
            # as TCP/IP doesn't have a baudrate, but we set it for consistency.
            self.ser = serial.serial_for_url(url, baudrate=self.baudrate, timeout=3)
            self._debug_print(f"Successfully connected to {url}")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            self.ser = None
            return False

    def close(self):
        if self.ser:
            self.ser.close()
            self._debug_print("Connection closed.")

    def create_frame(self, command, data=None, address=0x00):
        """
        Constructs command frame for CHAFON CF815 documented protocol.
        Frame: [Len][Adr][Cmd][Data...][LSB-CRC16][MSB-CRC16]
        """
        if data is None:
            data = []

        # 1. Calculate frame length
        # Len: length of Adr + Cmd + Data[] + 2 (for CRC16)
        # The 'Len' byte itself is not included in this count.
        len_field_value = len(data) + 4     # Adr(1) + Cmd(1) + CRC16(2)

        # 2. Prepare data for CRC calculation
        # Data for CRC calculation: [Len][Adr][Cmd][Data...]
        crc_data_bytes = bytearray([len_field_value, address, command] + data)
        
        # 3. Calculate and split CRC16 into LSB and MSB
        crc16 = self._calculate_crc16(crc_data_bytes)
        lsb_crc16 = crc16 & 0xFF
        msb_crc16 = (crc16 >> 8) & 0xFF

        # 4. Construct full byte array
        full_packet = bytearray([len_field_value, address, command] + data + [lsb_crc16, msb_crc16])
        return full_packet

    def send_command(self, command, data=None, address=0x00):
        if not self.ser:
            print("Serial port not connected.")
            return

        frame = self.create_frame(command, data, address)
        try:
            self.ser.write(frame)
            self._debug_print(f"Sent: {binascii.hexlify(frame).decode().upper()}")
        except Exception as e:
            print(f"Error sending data: {e}")
            self.close()

    def receive_response(self):
        """
        Receives data from the serial port and extracts a single complete frame.
        This method blocks until a full frame is received or timeout occurs.
        Returns the full frame (bytearray) if valid, otherwise None.
        """
        if not self.ser:
            return None

        try:
            # Read the first byte (Len)
            len_byte_data = self.ser.read(1)
            if not len_byte_data:
                return None  # Timeout or no data

            len_field_value = len_byte_data[0]
            
            # Total expected bytes for the frame (including the initial Len byte)
            total_frame_length = 1 + len_field_value

            # Read the remaining bytes of the frame
            # The 'Len' field value includes Adr, reCmd, Status, Data[], CRC16 (2 bytes)
            # So, we need to read 'len_field_value' more bytes.
            remaining_frame_data = self.ser.read(len_field_value)
            if len(remaining_frame_data) != len_field_value:
                print(f"Error: Incomplete frame received. Expected {len_field_value} bytes, got {len(remaining_frame_data)}.")
                return None  # Incomplete frame

            full_frame = len_byte_data + remaining_frame_data
            
            # Verify CRC16 using the reader's CRC algorithm (preset 0xFFFF, poly 0x8408).
            # Per device behaviour, calculating CRC over the full frame (including CRC bytes)
            # should yield 0x0000 when CRC bytes are correct.
            crc_check = self._calculate_crc16(full_frame)
            if crc_check == 0x0000:
                self._debug_print(f"Received (Valid CRC): {binascii.hexlify(full_frame).decode().upper()}")
                return full_frame
            else:
                print(f"CRC16 mismatch for received frame: {binascii.hexlify(full_frame).decode().upper()} (crc={crc_check:04X})")
                return None
        except Exception as e:
            print(f"Error receiving data: {e}")
            return None

    def handle_response_frame(self, frame):
        """
        Processes a single valid response frame.
        Frame: [Len][Adr][reCmd][Status][Data...][LSB-CRC16][MSB-CRC16]
        """
        if frame is None:
            return

        # Len is frame[0]
        adr = frame[1]
        re_cmd = frame[2]
        status = frame[3]
        # Data starts at index 4, ends before the last 2 bytes (CRC16)
        payload = frame[4:-2]

        # Accept both general success (0x00) and inventory success/status codes (e.g., 0x01)
        if status in (0x00, 0x01):
            if re_cmd == 0x21:  # Get Info Response
                print(f"Reader Info (Address: {adr:02X}, Command: {re_cmd:02X}, Status: {status:02X}, Payload: {binascii.hexlify(payload).decode().upper()})")
                # Further parse info from payload if needed
            elif re_cmd == 0x01:  # Inventory Response (EPC C1G2)
                if payload:
                    epc_str = binascii.hexlify(payload).decode().upper()
                    print(f"Inventory Response (Status {status:02X}) - Payload: {epc_str}")
                else:
                    print(f"Inventory Response (Status {status:02X}) - No tag data in this frame.")
            elif re_cmd in (0x50, 0x51):  # 6B inventory responses
                epc_str = binascii.hexlify(payload).decode().upper() if payload else ""
                print(f"6B Inventory Response (Cmd {re_cmd:02X}, Status {status:02X}) - Payload: {epc_str}")
            else:
                print(f"Command Response (Success) - Adr: {adr:02X}, Cmd: {re_cmd:02X}, Status: {status:02X}, Payload: {binascii.hexlify(payload).decode().upper()})")
        else:
            print(f"Command Response (Error Status {status:02X}) - Adr: {adr:02X}, Cmd: {re_cmd:02X}, Status: {status:02X}, Payload: {binascii.hexlify(payload).decode().upper()}")

    def get_info(self, address=0x00):
        """Command 0x21: Get Reader Information"""
        self._debug_print("Requesting Reader Info...")
        self.send_command(0x21, address=address)
        response_frame = self.receive_response()
        self.handle_response_frame(response_frame)

    def inventory(self, address=0x00):
        """Command 0x01: Tag Inventory (EPC C1G2)"""
        # For inventory, the data field might be empty or contain specific inventory parameters.
        # Use 0x01 for Tag Inventory as per protocol.
        self.send_command(0x01, data=[], address=address)
        response_frame = self.receive_response()
        self.handle_response_frame(response_frame)

if __name__ == "__main__":
    # Configuration
    READER_IP = "192.168.1.200"
    READER_PORT = 2022 # Default for Chafon/CF815. Try 27011 or 6000 if 2022 fails.
    BAUDRATE = 57600
    READER_ADDRESS = 0x00  # Default reader address

    reader = RFIDReaderTCP(READER_IP, READER_PORT, BAUDRATE, debug=True)
    
    # CRC self-test: construct a simple header ([Len][Adr][Cmd]), append LSB+MSB CRC,
    # and verify that CRC over the full frame equals 0x0000 (device verification behavior).
    test_hdr = bytearray([0x05, READER_ADDRESS, 0x21])  # Len, Adr, Cmd (no data)
    crc = reader._calculate_crc16(test_hdr)
    test_frame = test_hdr + bytearray([crc & 0xFF, (crc >> 8) & 0xFF])
    print(f"CRC self-test: {'OK' if reader._calculate_crc16(test_frame) == 0x0000 else 'FAIL'} - {binascii.hexlify(test_frame).decode().upper()}")
    
    if reader.connect():
        # 1. Get Reader Info to verify protocol
        reader.get_info(address=READER_ADDRESS)
        time.sleep(0.5)

        # 2. Loop for scanning tags
        print("\nStarting Inventory Loop (Press Ctrl+C to stop)...")
        try:
            while True:
                reader.inventory(address=READER_ADDRESS)
                # Small sleep to prevent flooding the network/CPU
                time.sleep(0.1) 
        except KeyboardInterrupt:
            print("\n User key pressed, stopping...")
        finally:
            reader.close()
