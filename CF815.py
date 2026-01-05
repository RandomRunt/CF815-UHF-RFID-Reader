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
            len_byte_data = self.ser.read()
            print("len_byte_data:", len_byte_data)
            return len_byte_data
            # if not len_byte_data:
            #     return None  # Timeout or no data
            
            # # 2. Calculate total bytes for the frame (including initial Len byte)
            # len_field_value = len_byte_data[0]
            # total_frame_length = 1 + len_field_value

            # # 2. Read the remaining bytes of the frame (len_field_value bytes)
            # # The 'Len' field value includes Adr, reCmd, Status, Data[], CRC16 (2 bytes)
            # remaining_frame_data = self.ser.read(len_field_value)
            # if len(remaining_frame_data) != len_field_value:
            #     print(f"Error: Incomplete frame received. Expected {len_field_value} bytes, got {len(remaining_frame_data)}.")
            #     return None  # Incomplete frame

            # full_frame = len_byte_data + remaining_frame_data
            
            # # Verify CRC16 using the reader's CRC algorithm (preset 0xFFFF, poly 0x8408).
            # # Per device behaviour, calculating CRC over the full frame (including CRC bytes)
            # # should give 0x0000 when CRC bytes are correct.
            # crc_check = self._calculate_crc16(full_frame)
            # if crc_check == 0x0000:
            #     self._debug_print(f"Received (Valid CRC): {binascii.hexlify(full_frame).decode().upper()}")
            #     return full_frame
            # else:
            #     print(f"CRC16 mismatch for received frame: {binascii.hexlify(full_frame).decode().upper()} (crc={crc_check:04X})")
            #     return None
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
                if status == 0x00:
                    print(f"[Response Details] (Success) - Buffer Count: {int.from_bytes(data[0:2], byteorder='big')} tags, Tag Num: {int.from_bytes(data[2:4], byteorder='big')} tags")
            case 0x72:
                """
                Command 0x72: Tag Inventory with Memory Buffer Response
                """
                if status == 0x01:
                    print("[RESPONSE] Inventory with Memory Buffer Response Received.")
                elif status == 0x03:
                    print("[RESPONSE] Inventory with Memory Buffer More Data Available Response Received.")
            case 0x73:
                """
                Command 0x73: Clear Memory Buffer Response
                """
                if status == 0x00:
                    print("[RESPONSE] Clear Memory Buffer Response Received.")
            case 0x74:
                """
                Command 0x74: Obtain Memory Buffer Tag Amount Response
                """
                if status == 0x00:
                    print("[RESPONSE] Obtain Memory Buffer Tag Amount Response Received.")
                    print(f"  Tag Amount: {int.from_bytes(data, byteorder='big')} tags")
            case 0x94:
                """
                Command 0x94: Read Antenna Power
                """
                print("[RESPONSE] Read Antenna Power Response Received.")
            
            case _:
                print(f"[RESPONSE] Unknown reCmd: {re_cmd:02X} received.")
                return None
        
        # match status:
        #     case 0x00:
        #         # Operation successful
        #         print(f"[Response Details] (Success) - Adr: {adr:02X}, Cmd: {re_cmd:02X}, Status: {status:02X}, Data: {binascii.hexlify(data).decode().upper()})")
        #     case 0x01:
        #         # Inventory successful
        #         print(f"[Response Details] (Inventory Success) - Adr: {adr:02X}, Cmd: {re_cmd:02X}, Status: {status:02X}, Data: {binascii.hexlify(data).decode().upper()})")
        #         # Display data breakdown
        #         print(f"  Tag Data: {binascii.hexlify(data).decode().upper()}")
        #     case 0x02:
        #         # Inventory timeout
        #         print(f"[Response Details] (Inventory Timeout) - Adr: {adr:02X}, Cmd: {re_cmd:02X}, Status: {status:02X}, Data: {binascii.hexlify(data).decode().upper()})")
        #     case 0x03:
        #         # Further data available to be delivered
        #         print(f"[Response Details] (Inventory More Data Available) - Adr: {adr:02X}, Cmd: {re_cmd:02X}, Status: {status:02X}, Data: {binascii.hexlify(data).decode().upper()})")
        #     case 0x04:
        #         # Reader memory full
        #         print(f"[Response Details] (Inventory Memory Full) - Adr: {adr:02X}, Cmd: {re_cmd:02X}, Status: {status:02X}, Data: {binascii.hexlify(data).decode().upper()})")
        #     case 0xFF:
        #         # Command Parameter error
        #         print(f"[Response Details] (Wrong Command Parameters Error) - Adr: {adr:02X}, Cmd: {re_cmd:02X}, Status: {status:02X}, Data: {binascii.hexlify(data).decode().upper()})")
        #     case _:
        #         # Unknown error status
        #         print(f"[Response Details] UNKNOWN RESPONSE (Error Status {status:02X}) - Adr: {adr:02X}, Cmd: {re_cmd:02X}, Status: {status:02X}, Data: {binascii.hexlify(data).decode().upper()})")
        #         return None
        
        return status
    
    def get_info(self, address=0x00):
        """Command 0x21: Get Reader Information"""
        print("\nRequesting Reader Info...")
        self.send_command(0x21, address=address)
        response_frame = self.receive_response()
        self.handle_response_frame(response_frame)

    def inventory(self, address=0x00, q_value=0b00000100, session=0x00, mask_mem=0x01, mask_adr=0x0000, mask_len=0x00
                  , adr_tid=0x00, len_tid=0x00, target=None, ant=None, scan_time=None, scan_time_sec=5.0):
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
        
        # print(f"\n--- ROBUST SCAN (Profile 1: Miller-4) ---")
        # # 1. FORCE PROFILE 1 (Miller-4 250KHz)
        # # Command 0x7F: Setup Reader Profile
        # # Data: 0x81 (Bit7=1 for 'Modify', Value=1 for 'Profile 1')
        # print("Configuring Reader to Profile 1 (Miller-4)...")
        # self.send_command(0x7F, data=[0x81], address=address)
        # resp = self.receive_response()
        # if resp[2] == 0x7F and resp[3] == 0x00:
        #     print(" -> Profile set successfully.")
        # else:
        #     print(" -> Failed to set profile (or already set).")
        
        # print("\nPerforming Inventory Scan (Press Ctrl+C to stop)...")
        
        # 1. Note start time
        start_time = time.time()
        unique_tags = set()
        
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
        self.send_command(0x01, data=data, address=address)
        
        response_frame = self.receive_response()
        response_length = response_frame[0]
        
        if response_frame:
            status = self.handle_response_frame(response_frame)
            if status == 0x01 and response_length > 7:
                tag_data = response_frame[5:-2]  # Extract tag data
                if tag_data is None or len(tag_data) == 0:
                    pass
                else:
                    tag_epc = binascii.hexlify(tag_data).decode().upper()
                    if tag_epc not in unique_tags:
                        unique_tags.add(tag_epc)
                        print(f"\n[NEW TAG DETECTED] EPC: {tag_epc}")

                print(f"Tags Found So Far: {len(unique_tags)}", end="\r")
        
        # while time.time() - start_time < scan_time_sec:  # Limit total inventory time to 30 seconds
        #     # 2. Send inventory command
        #     self.send_command(0x01, data=data, address=address)
            
        #     response_frame = self.receive_response()
        #     response_length = response_frame[0]
            
        #     if response_frame:
        #         status = self.handle_response_frame(response_frame)
        #         if status == 0x01 and response_length > 7:
        #             tag_data = response_frame[5:-2]  # Extract tag data
        #             if tag_data is None or len(tag_data) == 0:
        #                 continue
        #             else:
        #                 tag_epc = binascii.hexlify(tag_data).decode().upper()
        #                 if tag_epc not in unique_tags:
        #                     unique_tags.add(tag_epc)
        #                     print(f"\n[NEW TAG DETECTED] EPC: {tag_epc}")

        #             print(f"Tags Found So Far: {len(unique_tags)}", end="\r")
            
        #     # Small sleep to prevent overwhelming the serial buffer
        #     time.sleep(0.05)
            
        print(f"\nScan Finished! Total unique tags found: {len(unique_tags)}")
        return list(unique_tags)
        
        # # 2. Send inventory command
        # self.send_command(0x01, data=data, address=address)
            
        # # 3. Loop to receive all tag frames until a final status frame is received
        # while True:
        #     response_frame = self.receive_response()
        #     # if not response_frame:
        #     #     # response_frame = None received, exit loop
        #     #     break

        #     # try:
        #     #     response_frame = response_frame.decode()
        #     # except (UnicodeDecodeError, AttributeError):
        #     #     pass
            
        #     if response_frame:
        #         print(f"\n[INVENTORY RESPONSE FRAME] Received: {binascii.hexlify(response_frame).decode().upper()}")

        #     """
        #         Possible Status Codes for Inventory Response:
        #         0x01 - Inventory successful
        #         0x02 - Inventory timeout
        #         0x03 - Further data available to be delivered
        #         0x04 - Reader memory
        #         0x26 - Inventory successful, now return statistic data
        #         0xF8 - Antenna error detected, the current antenna may be disconnected
        #     """
            
        #     status = self.handle_response_frame(response_frame)
        #     if status == 0x00:
        #         # Operation completed
        #         print("Inventory operation completed.")
        #         return
                
        #     # if status == 0x26:
        #     #     # Statistics frame received, operation completed
        #     #     print("Inventory statistics frame received, operation completed.")
        #     #     break
        
    def inventory_continuous(self, address=0x00, duration_sec=5.0, q_value=0x06):
        """
        Continuously poll for tags (like Windows app does).
        This sends many quick scans instead of one long scan.
        """
        print(f"\n=== CONTINUOUS SCAN MODE ({duration_sec}s) ===")
        
        start_time = time.time()
        unique_tags = {}  # Store EPC -> last seen time
        scan_count = 0
        
        while time.time() - start_time < duration_sec:
            scan_count += 1
            
            # Quick 200ms scan
            data = [
                q_value,           # Q value
                0x00,              # Auto session
                0x01, 0x00, 0x00,  # No mask
                0x00,              # MaskLen
                0x00, 0x00,        # No TID
                0x01,              # Target A
                0x80,              # Antenna 1
                0x05               # 500ms scan (5 × 100ms)
            ]
            
            self.send_command(0x01, data=data, address=address)
            response = self.receive_response()
            
            if response and len(response) > 6:
                num_tags = response[5]
                
                if num_tags > 0:
                    idx = 6
                    for _ in range(num_tags):
                        if idx >= len(response) - 2:
                            break
                        
                        epc_len = response[idx]
                        idx += 1
                        
                        if idx + epc_len + 1 > len(response) - 2:
                            break
                        
                        epc_bytes = response[idx:idx + epc_len]
                        idx += epc_len
                        
                        rssi = response[idx]
                        idx += 1
                        
                        epc_hex = binascii.hexlify(epc_bytes).decode().upper()
                        
                        if epc_hex not in unique_tags:
                            print(f"\n[NEW TAG] {epc_hex} (RSSI: {rssi})")
                        
                        unique_tags[epc_hex] = time.time()
            
            # Show progress
            elapsed = time.time() - start_time
            print(f"Scans: {scan_count} | Tags: {len(unique_tags)} | Time: {elapsed:.1f}s", 
                end="\r", flush=True)
            
            time.sleep(0.05)  # Small delay between scans
        
        print(f"\n\n=== SCAN COMPLETE ===")
        print(f"Total scans: {scan_count}")
        print(f"Unique tags: {len(unique_tags)}")
        
        for epc in unique_tags:
            print(f"  > {epc}")
        
        return list(unique_tags.keys())
    
    def inventory_with_buffer(self, address=0x00, scan_time_sec=5.0):
        """
        Command 0x18: Inventory with Memory Buffer
        This forces the reader to scan for the specific time and store results internally.
        """
        # 1. Clear the buffer first so we don't get old data
        self.clear_memory_buffer(address)

        print(f"\nStarting Buffered Inventory for {scan_time_sec} seconds (Command 0x18)...")
        print("The reader will remain silent until the time is up.")

        # Calculate time in 100ms units (e.g., 5.0s * 10 = 50 = 0x32)
        scan_time_hex = int(scan_time_sec * 10)
        if scan_time_hex > 255: scan_time_hex = 255

        # 2. Construct Data Packet for Command 0x18
        # Format: Q(1), Session(1), MaskMem(1), MaskAdr(2), MaskLen(1), 
        #         AdrTID(1), LenTID(1), Target(1), Ant(1), ScanTime(1)
        data = [
            0x06,       # QValue    0b00000110 = 0x06 (No Stats, Standard Strategy, No FastID, No Phase, Q=6)
            0x00,       # Session (0x00 = S0 Read any tag immediately, 0xFF = Smart/Auto)
            0x01,       # MaskMem (EPC)
            0x00, 0x00, # MaskAdr
            0x00,       # MaskLen
            0x00,       # AdrTID
            0x00,       # LenTID
            0x01,       # Target A
            0x80,       # Antenna 1
            scan_time_hex # ScanTime (Mandatory for this logic)
        ]

        # 3. Send Command 0x18
        self.send_command(0x18, data=data, address=address)
        
        # time.sleep(scan_time_sec)
        
        import sys
        steps = int(scan_time_sec * 10) # 10 updates per second
        
        print(f"Scanning: [", end="", flush=True)
        for i in range(steps):
            time.sleep(0.1)
            # Update bar every 10% or so to avoid spamming, or just print dots/chars
            if i % (steps // 20 + 1) == 0: 
                print("#", end="", flush=True)
        print("] Done!")

        response = self.receive_response()
        if response:
            print("\n[BUFFERED SCAN COMPLETE]")
            self.handle_response_frame(response)
        else:
            print("\n[WARNING] No response received (Timeout).")
    
    def obtain_inventory_buffer(self, address=0x00):
        """
        Command 0x72: Tag Inventory with Memory Buffer
        """
        print("\nRequesting Inventory with Memory Buffer...")
        self.send_command(0x72, address=address)
        response_frame = self.receive_response()
        self.handle_response_frame(response_frame)
    
    def clear_memory_buffer(self, address=0x00):
        """
        Command 0x73: Clear memory buffer
        """
        print("\nClearing Memory Buffer...")
        self.send_command(0x73, address=address)
        # We expect a simple success response
        response = self.receive_response()
        self.handle_response_frame(response)
    
    def obtain_tag_amount(self, address=0x00):
        """
        Command 0x74: Obtain Memory Buffer Tag Amount
        """
        print("\nRequesting Tag Amount...")
        self.send_command(0x74, address=address)
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
    
    def find_tags_all_antennas(self, address=0x00):
        print("\n--- PORT SCANNER MODE ---")
        print("Cycling through Antennas 1-4. Press Ctrl+C to stop.\n")
        
        # Antenna IDs for standard 4-port reader (CMD 0x01 format)
        # 0x80 = Ant 1, 0x81 = Ant 2, 0x82 = Ant 3, 0x83 = Ant 4
        antennas = [0x80, 0x81, 0x82, 0x83]
        
        try:
            while True:
                for ant_id in antennas:
                    # print(f"Scanning Antenna {ant_id - 0x80 + 1}...", end='\r')
                    
                    # Packet: Q=0, Session=0, Mask=None, Target=0, Ant=Variable, Time=Small
                    data = [
                        0x06, 0x00,             # Q=6, Session 0
                        0x01, 0x00, 0x00, 0x00, # No Mask
                        0x00, 0x00,             # No TID
                        0x00, ant_id, 0x05      # Target A, CURRENT ANT, 500ms Time
                    ]

                    self.send_command(0x01, data=data, address=address)
                    print("Scanning Antenna {}...".format(ant_id - 0x80 + 1), end='\r')
                    response = self.receive_response()
                    
                    if response and len(response) > 6:
                        raw_data = response[4:-2]
                        # Byte 0 is Ant ID, Byte 1 is Count
                        if len(raw_data) >= 2:
                            num_tags = raw_data[1]
                            if num_tags > 0:
                                print(f"\n [!!!] TAG FOUND ON ANTENNA {ant_id - 0x80 + 1}!")
                                print(f"       Raw Data: {raw_data.hex().upper()}")
                                # Optional: Return here if you just want to find the port
                    
                    time.sleep(0.05) # Fast cycle
                    
        except KeyboardInterrupt:
            print("\nStopped.")
    
    def check_antenna_health(self, address=0x00):
        print("\n--- ANTENNA RETURN LOSS CHECK ---")
        # Command 0x91: Measure Return Loss 
        # Data: TestFreq(4 bytes) + Ant(1 byte)
        # Test Frequency: 915 MHz (Middle of US Band) = 915,000 KHz
        # 915,000 in Hex = 0x0D F5 E0
        
        freq_hex = [0x00, 0x0D, 0xF5, 0xE0] # 4 bytes, MSB first
        antenna = 0x00 # 0x00 = Antenna 1, 0x01 = Antenna 2 (Note: 0x91 uses 0-based index, unlike 0x01)
        
        data = freq_hex + [antenna]
        
        self.send_command(0x91, data=data, address=address)
        response = self.receive_response()
        
        if response and len(response) > 5 and response[3] == 0x00:
            # Data byte 0 is the Return Loss in dB
            rl_db = response[4] 
            print(f"Antenna 1 Return Loss: {rl_db} dB")
            
            if rl_db > 10:
                print(" -> STATUS: EXCELLENT. Antenna is radiating well.")
            elif rl_db > 5:
                print(" -> STATUS: POOR. Check cable tightness and positioning.")
            else:
                print(" -> STATUS: FAIL. The antenna is rejecting the signal.")
                print("    Possible causes: Broken cable, wrong antenna frequency (EU vs US), or damage.")
        else:
            print("Failed to measure Return Loss.")
    
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
    usr_input = input("Select operation:\n1 - Get Reader Info\n2 - Perform Inventory Scan with Buffer\n3 - Perform Inventory Scan with Manual Timing\n4 - Set Scan Time\n5 - Modify Antenna Power\n6 - Obtain EPC Tags in Memory Buffer Inventory\n7 - Obtain Tag Amount in Memory Buffer\nEnter choice (as a number) or 'exit' to quit: ")
    print("="*60)
    if reader.connect():
        while usr_input != "exit":
            match usr_input:
                case "1":
                    # Get Reader Info
                    reader.get_info(address=READER_ADDRESS)
                case "2":
                    # Perform Inventory Scan
                    try:
                        scan_duration = float(input("Enter scan time in seconds (0.3 - 25.5): "))
                        if scan_duration < 0.3 or scan_duration > 25.5:
                            print("Invalid time.")
                            continue
                        
                        reader.inventory_with_buffer(address=READER_ADDRESS, scan_time_sec=scan_duration)
                        
                        # Automatically fetch the results after the scan finishes
                        # print("\nRetrieving tag amount results...")
                        # reader.obtain_tag_amount(address=READER_ADDRESS) # Command 0x74
                        # reader.obtain_inventory_buffer(address=READER_ADDRESS) # Command 0x72
                    
                    except ValueError:
                        print("Invalid number format.")
                    except KeyboardInterrupt:
                        print("\n User ctrl+c pressed, stopping...")
                case "3":
                    scan_duration = float(input("Enter scan time in seconds (0.3 - 25.5): "))
                    if scan_duration < 0.3 or scan_duration > 25.5:
                        print("Invalid time.")
                        continue
                    
                    reader.inventory(
                        address=READER_ADDRESS,
                        q_value=0b00000110,   # 0x04 = 0b00000100 (No Stats, Standard Strategy, No FastID, No Phase Info, Q=4)
                        session=0x00,  # Smart session
                        mask_mem=0x01,
                        mask_adr=0x0000,
                        mask_len=0x00,
                        adr_tid=0x00,
                        len_tid=0x00,
                        target=0x00,  # Target A
                        ant=0x80,  # Antenna 2 -> 0x80 = Ant 1, 0x81 = Ant 2, 0x82 = Ant 3, 0x83 = Ant 4
                        scan_time=int(scan_duration * 10),   # old: int(scan_duration * 10) scan time in 100ms units
                        scan_time_sec=scan_duration
                    )
                    
                    # tags = reader.inventory_continuous(address=READER_ADDRESS, duration_sec=scan_duration)

                
                case "4":
                    scn_time = float(input("Enter desired inventory scan time in seconds (valid range: 0.3-25.5): "))
                    if scn_time < 0.3 or scn_time > 25.5:
                        print("Error: Scan time must be between 0.3 and 25.5 seconds.")
                        continue
                    # Set reader inventory scan time persistently
                    reader.set_scan_time_persistent(scan_time=scn_time, address=READER_ADDRESS)
                case "5":
                    antenna_power = int(input("Enter desired antenna power level (0-30 dBm): "))
                    if antenna_power < 0 or antenna_power > 30:
                        print("Error: Antenna power level must be between 0 and 30 dBm.")
                        continue
                    # Modify Antenna Power
                    reader.modify_antenna_power(address=READER_ADDRESS, power_level=antenna_power)
                case "6":
                    # Obtain EPC Tags in Memory Buffer Inventory
                    reader.obtain_inventory_buffer(address=READER_ADDRESS)
                case "7":
                    # Obtain Tag Amount in Memory Buffer
                    reader.obtain_tag_amount(address=READER_ADDRESS)
                case "8":
                    # Misc / Test Commands
                    # reader.find_tags_all_antennas(address=READER_ADDRESS)
                    reader.check_antenna_health(address=READER_ADDRESS)
                
                case "exit":
                    print("Exiting...")
                    reader.close()

            time.sleep(0.5)
            print()
            print("="*60)
            usr_input = input("Select operation:\n1 - Get Reader Info\n2 - Perform Inventory Scan with Buffer\n3 - Perform Inventory Scan with Manual Timing\n4 - Set Scan Time\n5 - Modify Antenna Power\n6 - Obtain EPC Tags in Memory Buffer Inventory\n7 - Obtain Tag Amount in Memory Buffer\nEnter choice (as a number) or 'exit' to quit: ")
        reader.close()   
