import socket
import time
import binascii
import struct
import sys

class RFIDReaderTCP:
    def __init__(self, ip, port, debug=True):
        self.ip = ip
        self.port = port
        self.sock = None
        self.debug = debug
        self.buzzer_on = False

    def _debug_print(self, message):
        """ Only print debug messages if debug mode is enabled. """
        if self.debug:
            print(message)

    def _calculate_crc16(self, data):
        """
        Calculate CRC16 as used by the reader (matching the C implementation):
        - preset: 0xFFFF
        - polynomial: 0x8408 (reflected CRC-16-CCITT for LSB first)
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
            self._debug_print(f"Attempting to connect to {self.ip}:{self.port}...")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0) # 2 second timeout for connection
            self.sock.connect((self.ip, self.port))
            self._debug_print(f"Successfully connected to {self.ip}:{self.port}")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            self.sock = None
            return False

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None
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
        len_field_value = len(data) + 4

        # 2. Prepare data for CRC calculation
        crc_data_bytes = bytearray([len_field_value, address, command] + data)
        
        # 3. Calculate and split CRC16
        crc16 = self._calculate_crc16(crc_data_bytes)
        lsb_crc16 = crc16 & 0xFF
        msb_crc16 = (crc16 >> 8) & 0xFF

        # 4. Construct full packet
        full_packet = bytearray([len_field_value, address, command] + data + [lsb_crc16, msb_crc16])
        return full_packet

    def send_command(self, command, data=None, address=0x00):
        if not self.sock:
            print("Socket not connected.")
            return

        # 1. Create command frame
        frame = self.create_frame(command, data, address)
        
        # 2. Send frame over TCP
        try:
            self.sock.sendall(frame)
            self._debug_print(f"Sent: {binascii.hexlify(frame).decode().upper()}")
        except Exception as e:
            print(f"Error sending data: {e}")
            self.close()

    def _recv_exact(self, n):
        """
        Helper to read exactly n bytes from the socket.
        This prevents fragmentation errors where a packet is split across two reads.
        """
        data = bytearray()
        start_t = time.time()
        
        while len(data) < n:
            # Check for timeout manually to avoid blocking forever if partial packet received
            if time.time() - start_t > 2.0: 
                return None
                
            try:
                chunk = self.sock.recv(n - len(data))
                if not chunk:
                    return None # Connection closed by remote
                data.extend(chunk)
            except socket.timeout:
                return None
            except Exception as e:
                print(f"Socket error during recv: {e}")
                return None
        return data

    def receive_response(self):
        """
        Receives a single complete frame using fragmented reading.
        Returns bytearray or None.
        """
        if not self.sock:
            return None

        try:
            # 1. Read the first byte (Len)
            len_byte_data = self._recv_exact(1)
            if not len_byte_data: return None
            
            len_field_value = len_byte_data[0]

            # 2. Read the remaining bytes (len_field_value)
            remaining_frame_data = self._recv_exact(len_field_value)
            if not remaining_frame_data:
                print(f"Error: Incomplete frame. Expected {len_field_value} more bytes.")
                return None

            full_frame = len_byte_data + remaining_frame_data
            
            # 3. Verify CRC
            crc_check = self._calculate_crc16(full_frame)
            if crc_check == 0x0000:
                if len(full_frame) < 20 or self.debug: 
                    self._debug_print(f"Received (Valid CRC): {binascii.hexlify(full_frame).decode().upper()}")
                return full_frame
            else:
                print(f"CRC Mismatch: {binascii.hexlify(full_frame).decode().upper()}")
                return None
                
        except Exception as e:
            print(f"Error receiving data: {e}")
            return None

    def handle_response_frame(self, frame):
        """
        Processes a single valid response frame.
        """
        if frame is None: return None

        # Len is frame[0]
        adr = frame[1]
        re_cmd = frame[2]
        status = frame[3]
        data = frame[4:-2] 

        # Handle specific response commands
        match re_cmd:
            case 0x21: # Get Info
                print("[RESPONSE] Reader Information Received.")
                if status == 0x00:
                    self._print_reader_info(data)
            
            case 0x01: # Inventory
                pass # Handled by loop
            
            case 0x18: # Buffered Inventory
                if status == 0x00:
                    buf_count = int.from_bytes(data[0:2], 'big')
                    tag_num = int.from_bytes(data[2:4], 'big')
                    print(f"[SUCCESS] Buffered Inventory: {tag_num} tags found, {buf_count} total in buffer.")

            case 0x72: # Get Buffer
                pass # Handled by loop
                
            case 0x25: # Set Scan Time
                if status == 0x00: print("[SUCCESS] Scan time updated.")
            
            case 0x2F: # Set Power
                if status == 0x00: print("[SUCCESS] Power level updated.")
                
            case 0x40: # Buzzer
                if status == 0x00: print("[SUCCESS] Buzzer mode updated.")
            
            case 0x91: # Antenna Check
                if status == 0x00:
                    rl_db = data[0]
                    print(f"\n[RESULT] Return Loss: {rl_db} dB")
                    if rl_db > 10: print(" -> STATUS: EXCELLENT")
                    elif rl_db > 5: print(" -> STATUS: POOR")
                    else: print(" -> STATUS: FAIL/DISCONNECTED")
            
            case _:
                if self.debug:
                    print(f"CMD {re_cmd:02X} Status {status:02X} Data: {binascii.hexlify(data).decode().upper()}")
        
        return status

    def _print_reader_info(self, data):
        print("\n" + "-"*60)
        print("READER INFORMATION [cite: 1489]")
        print("-"*60)
        version = f"{data[0]}.{data[1]}"
        print(f"  Version: {version}")
        print(f"  Power: {data[6]} dBm")
        print(f"  Antenna Config: 0b{data[8]:08b}")
        print("-"*60 + "\n")

    # =============================================================
    #  INVENTORY LOGIC
    # =============================================================

    def inventory_continuous_async(self, address=0x00, duration_sec=5.0):
        """
        Handles the 'Violent Flash' logic where reader sends multiple packets (Status 0x03)
        """
        print(f"\n=== ASYNC SCAN MODE (Run for {duration_sec}s) ===")
        script_start_time = time.time()
        unique_tags = {}
        scan_count = 0
        
        # 0x05 = 500ms timeout [cite: 1552]
        scan_time_hex = 0x05 
        
        try:
            while time.time() - script_start_time < duration_sec:
                scan_count += 1
                
                # 1. SEND COMMAND (Inventory 0x01)
                data = [0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, scan_time_hex]
                self.send_command(0x01, data=data, address=address)
                
                # 2. RECEIVE LOOP
                round_active = True
                while round_active:
                    response = self.receive_response()
                    if not response: break 
                    
                    if len(response) < 6: continue
                    status = response[3]
                    payload = response[4:-2]
                    
                    if status == 0x01 or status == 0x03 or status == 0x04:
                        self._parse_tag_payload(payload, unique_tags)
                        if status == 0x01: round_active = False
                    elif status == 0x02: # Timeout
                        round_active = False 
                    else:
                        round_active = False

                elapsed = time.time() - script_start_time
                print(f"Time: {elapsed:.1f}s | Scans: {scan_count} | Unique Tags: {len(unique_tags)}", end="\r")

        except KeyboardInterrupt:
            print("\nStopped by user.")
            
        print(f"\n\n=== SCAN COMPLETE ===")
        print(f"Total Unique Tags: {len(unique_tags)}")
        for epc in unique_tags:
            print(f"  > {epc} (Count: {unique_tags[epc]})")
        return list(unique_tags.keys())

    def _parse_tag_payload(self, payload, unique_tags):
        if len(payload) < 2: return
        num_tags = payload[1] # [cite: 266]
        idx = 2
        for _ in range(num_tags):
            if idx >= len(payload): break
            epc_len_word = payload[idx]
            epc_len_byte = epc_len_word * 2
            idx += 1
            if idx + epc_len_byte > len(payload): break
            epc_data = payload[idx : idx + epc_len_byte]
            idx += epc_len_byte
            idx += 1 # RSSI
            
            epc_hex = binascii.hexlify(epc_data).decode().upper()
            if epc_hex not in unique_tags: unique_tags[epc_hex] = 0
            unique_tags[epc_hex] += 1

    # =============================================================
    #  EXTRA FUNCTIONS (PORTED)
    # =============================================================

    def set_scan_time_persistent(self, scan_time=0x64, address=0x00):
        """ Command 0x25: Modify reader inventory time [cite: 1547] """
        print(f"\nSetting persistent scan time to {scan_time}s...")
        val = int(scan_time * 10)
        if val > 255: val = 255
        self.send_command(0x25, data=[val], address=address)
        resp = self.receive_response()
        self.handle_response_frame(resp)

    def check_antenna_health(self, address=0x00):
        """ Command 0x91: Measure Return Loss [cite: 2432] """
        print("\n--- ANTENNA RETURN LOSS CHECK ---")
        # Test Frequency: 915 MHz = 915,000 KHz = 0x0D F5 E0
        freq_hex = [0x00, 0x0D, 0xF5, 0xE0] 
        antenna = 0x00 # 0x00 = Antenna 1 [cite: 2440]
        data = freq_hex + [antenna]
        
        self.send_command(0x91, data=data, address=address)
        resp = self.receive_response()
        self.handle_response_frame(resp)

    def set_buzzer_mode(self, address=0x00, mode=0x00):
        """ Command 0x40: Set Buzzer Mode [cite: 1730] """
        state = "ON" if mode == 1 else "OFF"
        print(f"\nSetting Buzzer to {state}...")
        self.send_command(0x40, data=[mode], address=address)
        resp = self.receive_response()
        self.handle_response_frame(resp)

    def find_tags_all_antennas(self, address=0x00):
        """ Cycles through Antennas 1-4 looking for tags """
        print("\n--- PORT SCANNER MODE (Ctrl+C to Stop) ---")
        # 0x80 = Ant 1, 0x81 = Ant 2, 0x82 = Ant 3, 0x83 = Ant 4 [cite: 236]
        antennas = [0x80, 0x81, 0x82, 0x83]
        
        try:
            while True:
                for ant_id in antennas:
                    data = [0x06, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, ant_id, 0x05]
                    self.send_command(0x01, data=data, address=address)
                    print(f"Scanning Antenna {ant_id - 0x80 + 1}...", end='\r')
                    
                    resp = self.receive_response()
                    if resp and len(resp) > 6:
                        # If response Status is 0x01, 0x03, 0x04 and NumTags > 0
                        status = resp[3]
                        if status in [0x01, 0x03, 0x04]:
                            num_tags = resp[5]
                            if num_tags > 0:
                                print(f"\n [!!!] TAG FOUND ON ANTENNA {ant_id - 0x80 + 1}!")
                    time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nStopped.")

    # =============================================================
    #  STANDARD COMMANDS
    # =============================================================

    def get_info(self, address=0x00):
        print("\nRequesting Reader Info...")
        self.send_command(0x21, address=address)
        resp = self.receive_response()
        self.handle_response_frame(resp)

    def inventory_with_buffer(self, address=0x00, scan_time_sec=5.0):
        self.clear_memory_buffer(address)
        print(f"\nStarting Buffered Inventory for {scan_time_sec}s...")
        scan_time_hex = int(scan_time_sec * 10)
        if scan_time_hex > 255: scan_time_hex = 255
        # Q=6, Sess=0, MaskMem=1, Tgt=0, Ant=80, Time=...
        data = [0x06, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, scan_time_hex]
        self.send_command(0x18, data=data, address=address)
        
        # Simple progress bar
        steps = int(scan_time_sec * 10)
        print(f"Scanning: [", end="", flush=True)
        for i in range(steps):
            time.sleep(0.1)
            if i % (steps // 20 + 1) == 0: print("#", end="", flush=True)
        print("] Done!")
        
        resp = self.receive_response()
        self.handle_response_frame(resp)

    def clear_memory_buffer(self, address=0x00):
        self.send_command(0x73, address=address)
        self.receive_response()

    def obtain_tag_amount(self, address=0x00):
        print("\nRequesting Tag Amount...")
        self.send_command(0x74, address=address)
        resp = self.receive_response()
        if resp:
            data = resp[4:-2]
            count = int.from_bytes(data, 'big')
            print(f"  Buffer Count: {count}")

    def obtain_inventory_buffer(self, address=0x00):
        print("\nRequesting Buffer Data...")
        self.send_command(0x72, address=address)
        reading = True
        total = 0
        while reading:
            resp = self.receive_response()
            if not resp: break
            status = resp[3]
            if status == 0x01: reading = False
            elif status == 0x03: continue
            else: reading = False
            total += 1
        print(f"Finished. Received {total} packets.")

    def modify_antenna_power(self, address=0x00, power_level=0x14):
        print(f"\nSetting Power to {power_level} dBm...")
        self.send_command(0x2f, data=[power_level], address=address)
        resp = self.receive_response()
        self.handle_response_frame(resp)

    def debug_raw_traffic(self, address=0x00):
        print("\n=== RAW TRAFFIC (Ctrl+C to Stop) ===")
        try:
            while True:
                resp = self.receive_response()
                if resp:
                    print(f"RX: {binascii.hexlify(resp).decode().upper()}")
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("Stopped.")

if __name__ == "__main__":
    READER_IP = "192.168.1.200"
    READER_PORT = 2022 
    READER_ADDRESS = 0x00

    reader = RFIDReaderTCP(READER_IP, READER_PORT, debug=True)
    
    print("="*60)
    print("CHAFON CF815 RFID Reader TCP Interface (Socket Version)")
    print("="*60)
    
    if reader.connect():
        while True:
            try:
                print("\n1 - Get Reader Info")
                print("2 - Inventory Async (Violent Flash)")
                print("3 - Inventory Buffered")
                print("4 - Set Persistent Scan Time")
                print("5 - Set Antenna Power")
                print("6 - Get Buffer Count")
                print("7 - Get Buffer Data")
                print("8 - Check Antenna Health")
                print("9 - Toggle Buzzer")
                print("10 - Scan All Antennas (Port Check)")
                print("11 - Raw Monitor")
                print("Exit")
                
                choice = input("Enter choice: ").strip().lower()
                
                if choice == '1':
                    reader.get_info(READER_ADDRESS)
                elif choice == '2':
                    t = float(input("Duration (sec): "))
                    reader.inventory_continuous_async(READER_ADDRESS, t)
                elif choice == '3':
                    t = float(input("Scan Time (sec): "))
                    reader.inventory_with_buffer(READER_ADDRESS, t)
                elif choice == '4':
                    t = float(input("Scan Time (sec): "))
                    reader.set_scan_time_persistent(t, READER_ADDRESS)
                elif choice == '5':
                    p = int(input("Power (0-30): "))
                    reader.modify_antenna_power(READER_ADDRESS, p)
                elif choice == '6':
                    reader.obtain_tag_amount(READER_ADDRESS)
                elif choice == '7':
                    reader.obtain_inventory_buffer(READER_ADDRESS)
                elif choice == '8':
                    reader.check_antenna_health(READER_ADDRESS)
                elif choice == '9':
                    if reader.buzzer_on:
                        reader.set_buzzer_mode(READER_ADDRESS, 0)
                        reader.buzzer_on = False
                    else:
                        reader.set_buzzer_mode(READER_ADDRESS, 1)
                        reader.buzzer_on = True
                elif choice == '10':
                    reader.find_tags_all_antennas(READER_ADDRESS)
                elif choice == '11':
                    reader.debug_raw_traffic(READER_ADDRESS)
                elif choice == 'exit':
                    break
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")
                
        reader.close()
        