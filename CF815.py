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

        # 1. Create command frame
        frame = self.create_frame(command, data, address)
        
        # 2. Send frame over serial TCP/IP
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
            # 1. Read the first byte (Len)
            len_byte_data = self.ser.read(1)
            if not len_byte_data:
                return None  # Timeout or no data
            
            # 2. Calculate total bytes for the frame (including initial Len byte)
            len_field_value = len_byte_data[0]
            total_frame_length = 1 + len_field_value

            # 2. Read the remaining bytes of the frame (len_field_value bytes)
            # The 'Len' field value includes Adr, reCmd, Status, Data[], CRC16 (2 bytes)
            remaining_frame_data = self.ser.read(len_field_value)
            if len(remaining_frame_data) != len_field_value:
                print(f"Error: Incomplete frame received. Expected {len_field_value} bytes, got {len(remaining_frame_data)}.")
                return None  # Incomplete frame

            full_frame = len_byte_data + remaining_frame_data
            
            # Verify CRC16 using the reader's CRC algorithm (preset 0xFFFF, poly 0x8408).
            # Per device behaviour, calculating CRC over the full frame (including CRC bytes)
            # should give 0x0000 when CRC bytes are correct.
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
        data = frame[4:-2]  # Data starts at index 4, ends before the last 2 bytes (LSB and MSB CRC16)
        self._debug_print(f"Response Frame Parsed - Adr: {adr:02X}, reCmd: {re_cmd:02X}, Status: {status:02X}, Data: {binascii.hexlify(data).decode().upper()}")

        # Handle specific response commands
        match re_cmd:
            case 0x21:
                """
                Command 0x21: Get Reader Information Response
                """
                print("[RESPONSE] Reader Information Response Received.")
                
                if status == 0x00:
                    # Operation successful
                    print(f"[Response Details] (Success) - Adr: {adr:02X}, Cmd: {re_cmd:02X}, Status: {status:02X}, Data: {binascii.hexlify(data).decode().upper()})")
                    
                    # Parse Reader Information (Command 0x21)
                    print("\n" + "-"*60)
                    print("READER INFORMATION")
                    print("-"*60)
                    
                    version_major = data[0]
                    version_minor = data[1]
                    reader_type = data[2]
                    protocols = data[3]
                    dmaxfre = data[4]
                    dminfre = data[5]
                    power = data[6]
                    scntm = data[7]
                    ant_config = data[8]
                    # reserved = data[9], data[10]
                    check_ant = data[-1]
                    
                    print(f"  Version: {version_major}.{version_minor}")
                    print(f"  Reader Type: 0x{reader_type:02X}")
                    
                    # Parse protocol support
                    protocols_str = []
                    if protocols & 0b00000001:
                        protocols_str.append("ISO18000-6B")
                    if protocols & 0b00000010:
                        protocols_str.append("ISO18000-6C (EPC Gen2)")
                    print(f"  Supported Protocols: {', '.join(protocols_str)} (0b{protocols:08b})")
                    
                    # Parse frequency band
                    freq_band_bits = ((dmaxfre >> 6) << 2) | (dminfre >> 6)
                    freq_bands = {
                        0b0001: "Chinese band2 (920-925 MHz)",
                        0b0010: "US band (902-928 MHz)",
                        0b0011: "Korean band (917-921 MHz)",
                        0b0100: "EU band (865-868 MHz)",
                        0b0110: "Ukraine band (868-869 MHz)",
                        0b0111: "Peru band (916-928 MHz)",
                        0b1000: "Chinese band1 (840-845 MHz)",
                        0b1001: "EU3 band (865-868 MHz)",
                        0b1010: "Taiwan band (922-928 MHz)",
                        0b1100: "US band3 (902-928 MHz)"
                    }
                    freq_band = freq_bands.get(freq_band_bits, "Unknown")
                    max_freq_point = dmaxfre & 0b00111111
                    min_freq_point = dminfre & 0b00111111
                    print(f"  Frequency Band: {freq_band}")
                    print(f"    Max Frequency Point: {max_freq_point} (0x{dmaxfre:02X})")
                    print(f"    Min Frequency Point: {min_freq_point} (0x{dminfre:02X})")
                    
                    # Parse RF power
                    print(f"  RF Power: {power} dBm (0-33 dBm)")
                    
                    # Parse scan time
                    scan_time_seconds = scntm * 0.1
                    print(f"  Inventory Scan Time: {scntm} (0x{scntm:02X}) = {scan_time_seconds}s")

                    # Parse antenna configuration
                    enabled_antennas = []
                    for i in range(8):
                        if ant_config & (1 << i):
                            enabled_antennas.append(i + 1)
                    print(f"  Antenna Config: 0b{ant_config:08b}")
                    print(f"    Enabled Antennas: {', '.join(map(str, enabled_antennas)) if enabled_antennas else 'None'}")
                    
                    # Parse antenna check
                    print(f"  Antenna Check: {'Enabled' if check_ant == 1 else 'Disabled'}")
                    print("-"*60 + "\n")

            case 0x77:
                """
                Command 0x77: Get Reader Working Mode Response
                """
                print("[RESPONSE] Reader Working Mode Response Received.")
                if status == 0x00:
                    # Operation successful
                    print(f"[Response Details] (Success) - Adr: {adr:02X}, Cmd: {re_cmd:02X}, Status: {status:02X}, Data: {binascii.hexlify(data).decode().upper()})")
                    
                    # Parse Reader Information (Command 0x21)
                    print("\n" + "="*60)
                    print("READER WORKING MODE DETAILS")
                    print("="*60)
                    
                    work_mode = data[0]
                    tag_protocol = data[1]
                    read_pause_time = data[2]
                    filter_time = data[3]
                    q_value = data[4]
                    session = data[5]
                
            case 0x25:
                """
                Command 0x25: Modify reader inventory time
                """
                print("[RESPONSE] Modify Inventory Time Received.")
                if status == 0x00:
                    # Operation successful
                    print(f"[Response Details] (Success) - Adr: {adr:02X}, Cmd: {re_cmd:02X}, Status: {status:02X}, Data: {binascii.hexlify(data).decode().upper()})")

            case 0x2F:
                """
                Command 0x2F: Modify Antenna Power
                """
                print("[RESPONSE] Modify Antenna Power Received.")
                if status == 0x00:
                    # Operation successful
                    print(f"[Response Details] (Success) - Adr: {adr:02X}, Cmd: {re_cmd:02X}, Status: {status:02X}, Data: {binascii.hexlify(data).decode().upper()})")

            case 0x01:
                """
                Command 0x01: Tag Inventory return command
                """
                print("[RESPONSE] Inventory Response Received.")
            case 0x18:
                """
                Command 0x18: Tag Inventory return command with memory buffer
                """
                print("[RESPONSE] Inventory with Memory Response Received.")
            case 0x72:
                """
                Command 0x72: Tag Inventory with Memory Buffer Response
                """
                if (status == 0x01):
                    print("[RESPONSE] Inventory with Memory Buffer Response Received.")
                elif (status == 0x03):
                    print("[RESPONSE] Inventory with Memory Buffer More Data Available Response Received.")
            case 0x94:
                """
                Command 0x94: Read Antenna Power
                """
                print("[RESPONSE] Read Antenna Power Response Received.")
            
            case _:
                print(f"[RESPONSE] Unknown reCmd: {re_cmd:02X} received.")
                return None
        
        match status:
            case 0x00:
                # Operation successful
                print(f"[Response Details] (Success) - Adr: {adr:02X}, Cmd: {re_cmd:02X}, Status: {status:02X}, Data: {binascii.hexlify(data).decode().upper()})")
            case 0x01:
                # Inventory successful
                print(f"[Response Details] (Inventory Success) - Adr: {adr:02X}, Cmd: {re_cmd:02X}, Status: {status:02X}, Data: {binascii.hexlify(data).decode().upper()})")
                # Display data breakdown
                print(f"  Tag Data: {binascii.hexlify(data).decode().upper()}")
            case 0x02:
                # Inventory timeout
                print(f"[Response Details] (Inventory Timeout) - Adr: {adr:02X}, Cmd: {re_cmd:02X}, Status: {status:02X}, Data: {binascii.hexlify(data).decode().upper()})")
            case 0x03:
                # Further data available to be delivered
                print(f"[Response Details] (Inventory More Data Available) - Adr: {adr:02X}, Cmd: {re_cmd:02X}, Status: {status:02X}, Data: {binascii.hexlify(data).decode().upper()})")
            case 0x04:
                # Reader memory full
                print(f"[Response Details] (Inventory Memory Full) - Adr: {adr:02X}, Cmd: {re_cmd:02X}, Status: {status:02X}, Data: {binascii.hexlify(data).decode().upper()})")
            case 0xFF:
                # Command Parameter error
                print(f"[Response Details] (Wrong Command Parameters Error) - Adr: {adr:02X}, Cmd: {re_cmd:02X}, Status: {status:02X}, Data: {binascii.hexlify(data).decode().upper()})")
            case _:
                # Unknown error status
                print(f"[Response Details] UNKNOWN RESPONSE (Error Status {status:02X}) - Adr: {adr:02X}, Cmd: {re_cmd:02X}, Status: {status:02X}, Data: {binascii.hexlify(data).decode().upper()})")
                return None
        
        return status
    
    def get_info(self, address=0x00):
        """Command 0x21: Get Reader Information"""
        print("\nRequesting Reader Info...")
        self.send_command(0x21, address=address)
        response_frame = self.receive_response()
        self.handle_response_frame(response_frame)

    def inventory(self, address=0x00, q_value=0b00000110, session=0x00, mask_mem=0x00, mask_adr=0x0000, mask_len=0x00
                  , adr_tid=0x00, len_tid=0x00, target=None, ant=None, scan_time=None):
        """
        Command 0x01: Tag Inventory (EPC C1G2)
        
        Data[] Parameters:
            q_value: Query value (Bit3-Bit0: 0-15, Default: 0b0110 ~ reads around ) | Flags (Bit7: Stats, Bit6: Strategy, Bit5: FastID, Bit4: Phase)
                -> 0b00000110 = 0x06 (Default) -> Q=6 [Slots = 2^Q = 2^6], No Stats, Standard Strategy, No FastID, No Phase
            session: Session (0x00-0x03, 0xFF=Smart)
            mask_mem: Mask Memory Bank (default 0x00=EPC)
            mask_adr: Mask Start Address (default 0x20)
            mask_len: Mask Length in bits (default 0x00)
            adr_tid: TID Start Address (default 0x00)
            len_tid: TID Length in bits (default 0x00)
            target: (optional) Target (0x00=A, 0x01=B)
            ant: (optional) Antenna Selection (0x80=antenna1, 0x81=antenna2, 0x82=antenna3, 0x83=antenna4)
            scan_time: (optional) Scan time in scan_time*100ms
        """
        
        print("\nPerforming Inventory Scan (Press Ctrl+C to stop)...")

        # 1. Generate data array for inventory command
        data = [
            q_value,
            session,
            mask_mem,
            (mask_adr >> 8) & 0xFF, mask_adr & 0xFF,
            mask_len,
            adr_tid,
            len_tid
        ]
        
        if target is not None:
            data.append(target)
        if ant is not None:
            data.append(ant)
        if scan_time is not None:
            data.append(scan_time)

        # 2. Send inventory command
        self.send_command(0x18, data=data, address=address)
        
        # 3. Loop to receive all tag frames until a final status frame is received
        while True:
            response_frame = self.receive_response()
            # if not response_frame:
            #     # response_frame = None received, exit loop
            #     break

            # try:
            #     response_frame = response_frame.decode()
            # except (UnicodeDecodeError, AttributeError):
            #     pass
            
            if response_frame:
                print(f"\n[INVENTORY RESPONSE FRAME] Received: {binascii.hexlify(response_frame).decode().upper()}")

            """
                Possible Status Codes for Inventory Response:
                0x01 - Inventory successful
                0x02 - Inventory timeout
                0x03 - Further data available to be delivered
                0x04 - Reader memory
                0x26 - Inventory successful, now return statistic data
                0xF8 - Antenna error detected, the current antenna may be disconnected
            """
            
            status = self.handle_response_frame(response_frame)
            if status == 0x00:
                # Operation completed
                print("Inventory operation completed.")
                return
                
            # if status == 0x26:
            #     # Statistics frame received, operation completed
            #     print("Inventory statistics frame received, operation completed.")
            #     break
    
    def obtain_memory_buffer(self, address=0x00):
        """
        Command 0x72: Tag Inventory with Memory Buffer
        """
        print("\nRequesting Inventory with Memory Buffer...")
        self.send_command(0x72, address=address)
        response_frame = self.receive_response()
        self.handle_response_frame(response_frame)
    
    def read_antenna_power(self, address=0x00):
        """
        Command 0x94: Read Antenna Power
        """
        print("\nRequesting Antenna Power...")
        self.send_command(0x94, address=address)
        response_frame = self.receive_response()
        self.handle_response_frame(response_frame)
    
    def modify_antenna_power(self, address=0x00, power_level=0x14):
        """
        Command 0x2f: Modify Antenna Power
        Data[] RF Power Params: 4 bytes, From left to right, there are antennas 1 to 4, each represented as follows the
        RF power parameters.
        bit0 ~ bit6: RF power setting, the valid value of this parameter is 0 ~ 30. For setting of 30, the
        output power is approximately 1W.
        UHF RFID Reader Series User Manual v2.20
        64
        bit7: configuration preservation status during power off.
        0 - configuration preserved during reader power off;
        1 - configuration is not preserved
        """
        print(f"\nSetting Antenna Power to {power_level} dBm...")
        self.send_command(0x2f, data=[power_level], address=address)
        response_frame = self.receive_response()
        self.handle_response_frame(response_frame)
    
    def get_reader_working_mode(self, address=0x00):
        """
        Command 0x77: Get Reader Working Mode
        """
        print("\nRequesting Reader Working Mode...")
        self.send_command(0x77, address=address)
        response_frame = self.receive_response()
        self.handle_response_frame(response_frame)
    
    def set_scan_time_persistent(self, scan_time=0x64, address=0x00):
        """
        Command 0x25: Modify reader inventory time (persists until changed)
        """
        print(f"\nSetting persistent scan time to {scan_time}s...")
        self.send_command(0x25, data=[int(scan_time * 10)], address=address)
        response = self.receive_response()
        self.handle_response_frame(response)
    
    # def set_working_frequency(self, address=0x00, max_fre=0x01, min_fre=0x00):
    #     """
    #     Australian UHF RFID Standard:
    #       - EPC® Radio-Frequency Identity Generation-2 UHF RFID Standard
    #     Australian RFID Range: 918-926MHz
    #     Chinese Band 2: 920-925MHz (max_fre=0x00, min_fre=0x01)
    #
    #     Command 0x22: Set Working Frequency
    #     Data[] Parameters:
    #         max_freq: Maximum frequency channel (926MHz = 0b00010011 = 0x13)
    #         min_freq: Minimum frequency channel (920MHz = 0b01000000 = 0x40)

if __name__ == "__main__":
    # Configuration
    READER_IP = "192.168.1.200"
    READER_PORT = 2022 # Default for Chafon/CF815. Try 27011 or 6000 if 2022 fails.
    BAUDRATE = 57600
    READER_ADDRESS = 0x00  # Default reader address

    # Initialize RFID Reader connection over TCP
    reader = RFIDReaderTCP(READER_IP, READER_PORT, BAUDRATE, debug=True)
    
    # # CRC self-test: construct a simple header ([Len][Adr][Cmd]), append LSB+MSB CRC,
    # # to verify that CRC over the full frame equals 0x0000.
    # test_hdr = bytearray([0x05, READER_ADDRESS, 0x21])  # Len, Adr, Cmd (no data)
    # crc = reader._calculate_crc16(test_hdr)
    # test_frame = test_hdr + bytearray([crc & 0xFF, (crc >> 8) & 0xFF])
    # print(f"CRC self-test: {'OK' if reader._calculate_crc16(test_frame) == 0x0000 else 'FAIL'} - {binascii.hexlify(test_frame).decode().upper()}")
    
    # Prompt user to specify which operation to perform
    print("="*60)
    print("CHAFON CF815 RFID Reader TCP Interface")
    print("="*60)
    usr_input = input("Select operation:\n1 - Get Reader Info\n2 - Perform Inventory Scan\n3 - Set Inventory Scan Time\n4 - Modify Antenna Power\nEnter choice (as a number) or 'exit' to quit: ")
    print("="*60)
    if reader.connect():
        while usr_input != "exit":
            match usr_input:
                case "1":
                    # Get Reader Info
                    reader.get_info(address=READER_ADDRESS)
                case "2":
                    scn_time = float(input("Enter desired inventory scan time in seconds (valid range: 0.3-25.5): "))
                    if scn_time < 0.3 or scn_time > 25.5:
                        print("Error: Scan time must be between 0.3 and 25.5 seconds.")
                        continue
                    # Perform Inventory Scan
                    try:
                        reader.inventory(
                                address=READER_ADDRESS,
                                q_value=0b00000110,   # 0x06 = 0b00000110 (No Stats, Standard Strategy, No FastID, No Phase Info, Q=6)
                                session=0xFF,  # Smart session
                                mask_mem=0x01,
                                mask_adr=0x0020,
                                mask_len=0x00,
                                adr_tid=0x00,
                                len_tid=0x00,
                                target=0x00,  # Target A
                                ant=0x80,  # Antenna 1
                                scan_time= int(scn_time * 10)   # scan time in 100ms units
                            )
                        # Still need to fix this command -> getting Status: FF (Unknown Response)
                        # Print out raw output for debugging
                    except KeyboardInterrupt:
                        print("\n User ctrl+c pressed, stopping...")
                    finally:
                        reader.close()
                case "3":
                    scn_time = float(input("Enter desired inventory scan time in seconds (valid range: 0.3-25.5): "))
                    if scn_time < 0.3 or scn_time > 25.5:
                        print("Error: Scan time must be between 0.3 and 25.5 seconds.")
                        continue
                    # Set reader inventory scan time persistently
                    reader.set_scan_time_persistent(scan_time=scn_time, address=READER_ADDRESS)
                case "4":
                    antenna_power = int(input("Enter desired antenna power level (0-30 dBm): "))
                    if antenna_power < 0 or antenna_power > 30:
                        print("Error: Antenna power level must be between 0 and 30 dBm.")
                        continue
                    # Modify Antenna Power
                    reader.modify_antenna_power(address=READER_ADDRESS, power_level=antenna_power)
                case "5":
                    # Obtain Memory Buffer Inventory
                    reader.obtain_memory_buffer(address=READER_ADDRESS)
                
                case "exit":
                    print("Exiting...")
                    reader.close()

            print()
            print("="*60)
            usr_input = input("Select operation:\n1 - Get Reader Info\n2 - Perform Inventory Scan\n3 - Set Inventory Scan Time\n4 - Modify Antenna Power\nEnter choice (as a number) or 'exit' to quit: ")
            print("="*60)
            
            
        time.sleep(0.5)