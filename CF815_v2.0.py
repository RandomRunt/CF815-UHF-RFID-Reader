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
        Calculate CRC16 (Polynomial 0x8408)
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
        """ Constructs command frame [Len][Adr][Cmd][Data...][CRC16] """
        if data is None: data = []
        len_field_value = len(data) + 4
        crc_data_bytes = bytearray([len_field_value, address, command] + data)
        crc16 = self._calculate_crc16(crc_data_bytes)
        full_packet = bytearray([len_field_value, address, command] + data + [crc16 & 0xFF, (crc16 >> 8) & 0xFF])
        return full_packet

    def send_command(self, command, data=None, address=0x00):
        if not self.sock:
            print("Socket not connected.")
            return
        frame = self.create_frame(command, data, address)
        try:
            self.sock.sendall(frame)
            self._debug_print(f"Sent: {binascii.hexlify(frame).decode().upper()}")
        except Exception as e:
            print(f"Error sending data: {e}")
            self.close()

    def _recv_exact(self, n):
        """ Helper to read exactly n bytes (prevents TCP fragmentation) """
        data = bytearray()
        start_t = time.time()
        while len(data) < n:
            if time.time() - start_t > 3.0: return None
            try:
                chunk = self.sock.recv(n - len(data))
                if not chunk: return None
                data.extend(chunk)
            except socket.timeout: return None
            except Exception as e:
                print(f"Socket error: {e}")
                return None
        return data

    def receive_response(self):
        """ Receives a single complete frame. """
        if not self.sock: return None
        try:
            # 1. Read Len
            head = self._recv_exact(1)
            if not head: return None
            length = head[0]
            # 2. Read Body
            body = self._recv_exact(length)
            if not body: return None
            full = head + body
            # 3. Check CRC
            if self._calculate_crc16(full) == 0x0000:
                if len(full) < 20 or self.debug: 
                    self._debug_print(f"Received: {binascii.hexlify(full).decode().upper()}")
                return full
            else:
                print(f"CRC Mismatch: {binascii.hexlify(full).decode().upper()}")
                return None
        except Exception as e:
            print(f"Rx Error: {e}")
            return None

    def handle_response_frame(self, frame):
        """ Processes response frames and prints human-readable info """
        if frame is None: return None
        adr, re_cmd, status = frame[1], frame[2], frame[3]
        data = frame[4:-2] 

        match re_cmd:
            case 0x21: # Get Info
                if status == 0x00: self._print_reader_info(data)
            case 0x18: # Buffered Inventory
                if status == 0x00:
                    print(f"[SUCCESS] Buffered Inventory: {int.from_bytes(data[2:4], 'big')} tags found.")
            case 0x76: # Work Mode
                if status == 0x00: print("[INIT] Reader Mode Set Successfully.")
            case 0x3F: # Antenna
                if status == 0x00: print("[INIT] Antenna Port Enabled.")
            case 0x2F: # Power
                if status == 0x00: print("[INIT] RF Power Set.")
            case 0x91: # Antenna Check
                if status == 0x00: print(f"[RESULT] Return Loss: {data[0]} dB")
            case _:
                if self.debug and status != 0x03:
                    print(f"CMD {re_cmd:02X} Status {status:02X} Data: {binascii.hexlify(data).decode().upper()}")
        return status

    def _print_reader_info(self, data):
        print("\n" + "-"*60)
        print("READER INFORMATION")
        print(f"  Version: {data[0]}.{data[1]}")
        print(f"  Power: {data[6]} dBm")
        print(f"  Antenna Config: 0b{data[8]:08b}")
        print("-"*60 + "\n")

    # =============================================================
    #  INITIALIZATION ROUTINE
    # =============================================================
    def initialize_reader(self, address=0x00):
        """
        Critical Setup: Forces Answer Mode, Enables Antenna, Sets Power.
        Call this before starting any inventory to ensure sync.
        """
        print("\n=== INITIALIZING READER ===")
        
        # 1. Set Working Mode -> Answer Mode (0x00) [Cmd 0x76]
        # This prevents the reader from auto-scanning and confusing the socket
        print("1. Setting Answer Mode (0x00)...")
        self.send_command(0x76, data=[0x00], address=address)
        self.receive_response() # Consume response

        # 2. Enable Antenna 1 [Cmd 0x3F]
        # Data: 0x01 (Enable Ant 1)
        print("2. Enabling Antenna Port 1...")
        self.send_command(0x3F, data=[0x01], address=address)
        self.receive_response()

        # 3. Set RF Power to Max (30dBm) [Cmd 0x2F]
        # Data: 0x1E (30 decimal)
        print("3. Setting RF Power to 30dBm...")
        self.send_command(0x2F, data=[0x1E], address=address)
        self.receive_response()
        
        print("=== INITIALIZATION COMPLETE ===\n")

    # =============================================================
    #  INVENTORY LOGIC
    # =============================================================
    def inventory_continuous_async(self, address=0x00, duration_sec=5.0):
        print(f"\n=== ASYNC SCAN MODE (Run for {duration_sec}s) ===")
        
        script_start_time = time.time()
        unique_tags = {}
        scan_count = 0
        
        # --- CONFIGURATION MATCHING WINDOWS APP ---
        # 1. Scan Time: Windows uses 50 (5.0s). We will use 0x14 (2.0s) for better responsiveness
        #    but long enough to catch tags. 0x05 (0.5s) is too short.
        scan_time_hex = 0x32
        
        # 2. Session: Windows uses "Auto". 
        #    Manual [cite: 214] says 0xFF = "Smart configuration" (Auto).
        #    This is safer than 0x00 (Session 0).
        session_val = 0xFF 
        
        try:
            while time.time() - script_start_time < duration_sec:
                scan_count += 1
                
                # Construct Command 0x01 (Inventory)
                # [Q, Session, MaskMem, MaskAdr(2), MaskLen, AdrTID, LenTID, Target, Ant, Time]
                
                # Note: We toggle Target A (0x00) and Target B (0x01) every scan
                # to catch tags that might be stuck in "B" state (Read) from previous sessions.
                target_val = 0x00 if (scan_count % 2 == 0) else 0x01
                
                cmd_data = [
                    0x04,           # Q=4 (Matches Windows App) [cite: 1489]
                    session_val,    # Session=0xFF (Auto/Smart) [cite: 214]
                    0x01,           # MaskMem=EPC
                    0x00, 0x00,     # MaskAdr
                    0x00,           # MaskLen
                    0x00,           # AdrTID
                    0x00,           # LenTID
                    target_val,     # Target (Flips A/B) [cite: 233]
                    0x80,           # Antenna 1 [cite: 237]
                    scan_time_hex   # ScanTime (2.0s)
                ]
                
                self.send_command(0x01, data=cmd_data, address=address)
                
                # --- RECEIVE LOOP ---
                round_active = True
                while round_active:
                    response = self.receive_response()
                    if not response: break 
                    
                    if len(response) < 6: continue
                    status = response[3]
                    payload = response[4:-2]
                    
                    # Status 0x01 (Success), 0x03 (More Data), 0x04 (Mem Full) all contain tags
                    if status in [0x01, 0x03, 0x04]:
                        self._parse_tags(payload, unique_tags)
                        # If 0x01, the reader is declaring "I am done with this round"
                        if status == 0x01: round_active = False 
                        
                    elif status == 0x02: # Timeout (No tags found in this window)
                        round_active = False
                        
                    else: # Other errors (e.g., Antenna Error 0xF8)
                        if status == 0xF8: print("[!] Antenna Disconnected?")
                        round_active = False

                elapsed = time.time() - script_start_time
                print(f"Time: {elapsed:.1f}s | Mode: {'Target A' if target_val==0 else 'Target B'} | Tags: {len(unique_tags)}", end="\r")

                # Small cool-down to let the reader reset RF field
                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\nStopped by user.")
            
        print(f"\n\n=== SCAN COMPLETE ===")
        print(f"Total Unique Tags: {len(unique_tags)}")
        for epc in unique_tags:
            # We can't access count easily with current dict structure, just printing EPC
            print(f"  > {epc}")
        return list(unique_tags.keys())

    def _parse_tags(self, payload, tags):
        if len(payload) < 2: return
        count = payload[1]
        idx = 2
        for _ in range(count):
            if idx >= len(payload): break
            epc_len = payload[idx] * 2
            idx += 1
            epc = payload[idx : idx + epc_len]
            idx += epc_len + 1 # +1 for RSSI
            
            epc_str = binascii.hexlify(epc).decode().upper()
            tags[epc_str] = 1

    # =============================================================
    #  UTILITY FUNCTIONS
    # =============================================================
    def get_info(self, address=0x00):
        print("\nRequesting Info...")
        self.send_command(0x21, address=address)
        self.handle_response_frame(self.receive_response())

    def inventory_with_buffer(self, address=0x00, scan_time_sec=5.0):
        self.send_command(0x73, address=address) # Clear buffer
        self.receive_response()
        
        print(f"\nBuffered Scan ({scan_time_sec}s)...")
        time_hex = min(255, int(scan_time_sec * 10))
        data = [0x06, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, time_hex]
        self.send_command(0x18, data=data, address=address)
        
        for _ in range(int(scan_time_sec*10)):
            time.sleep(0.1)
            print(".", end="", flush=True)
        print(" Done!")
        self.handle_response_frame(self.receive_response())

    def obtain_tag_amount(self, address=0x00):
        self.send_command(0x74, address=address)
        resp = self.receive_response()
        if resp and resp[3] == 0x00:
            print(f"  Buffer Count: {int.from_bytes(resp[4:6], 'big')}")

    def obtain_inventory_buffer(self, address=0x00):
        self.send_command(0x72, address=address)
        reading = True
        total = 0
        while reading:
            resp = self.receive_response()
            if not resp or resp[3] == 0x01: reading = False
            if resp: total += 1
        print(f"Received {total} tag packets.")

    def modify_antenna_power(self, address=0x00, power_level=30):
        print(f"\nSetting Power to {power_level}...")
        self.send_command(0x2f, data=[power_level], address=address)
        self.handle_response_frame(self.receive_response())

    def set_scan_time_persistent(self, val, address=0x00):
        print(f"\nSetting persistent time to {val}s...")
        self.send_command(0x25, data=[int(val*10)], address=address)
        self.handle_response_frame(self.receive_response())

    def check_antenna_health(self, address=0x00):
        print("\nChecking Antenna...")
        self.send_command(0x91, data=[0x00, 0x0D, 0xF5, 0xE0, 0x00], address=address)
        self.handle_response_frame(self.receive_response())

    def set_buzzer_mode(self, address=0x00, mode=0x00):
        print(f"\nSetting Buzzer {mode}...")
        self.send_command(0x40, data=[mode], address=address)
        self.handle_response_frame(self.receive_response())

    def find_tags_all_antennas(self, address=0x00):
        print("\nCycling Antennas (Ctrl+C to stop)...")
        try:
            while True:
                for ant in [0x80, 0x81, 0x82, 0x83]:
                    self.send_command(0x01, data=[0x04,0,1,0,0,0,0,0,0,ant,0x05], address=address)
                    resp = self.receive_response()
                    if resp and len(resp)>6 and resp[5]>0:
                        print(f"TAG FOUND ON ANT {ant-0x7F}!")
                    time.sleep(0.1)
        except KeyboardInterrupt: pass

    def debug_raw_traffic(self, address=0x00):
        print("\nRaw Monitor (Ctrl+C to stop)...")
        try:
            while True:
                r = self.receive_response()
                if r: print(f"RX: {binascii.hexlify(r).decode().upper()}")
        except KeyboardInterrupt: pass

if __name__ == "__main__":
    READER_IP = "192.168.1.200"
    READER_PORT = 2022
    ADDR = 0x00

    reader = RFIDReaderTCP(READER_IP, READER_PORT, debug=True)
    print("="*60)
    print("CHAFON CF815 Socket Controller v3 (Auto-Init Enabled)")
    print("="*60)

    if reader.connect():
        # --- AUTO INITIALIZATION ---
        reader.initialize_reader(ADDR)
        
        while True:
            try:
                print("\n1-Info 2-AsyncScan 3-BufScan 4-PersistentTime 5-Power")
                print("6-BufCount 7-BufData 8-AntHealth 9-Buzzer 10-PortScan 11-Raw")
                c = input("Choice: ")
                
                if c=='1': reader.get_info(ADDR)
                elif c=='2': reader.inventory_continuous_async(ADDR, float(input("Sec: ")))
                elif c=='3': reader.inventory_with_buffer(ADDR, float(input("Sec: ")))
                elif c=='4': reader.set_scan_time_persistent(float(input("Sec: ")), ADDR)
                elif c=='5': reader.modify_antenna_power(ADDR, int(input("dBm: ")))
                elif c=='6': reader.obtain_tag_amount(ADDR)
                elif c=='7': reader.obtain_inventory_buffer(ADDR)
                elif c=='8': reader.check_antenna_health(ADDR)
                elif c=='9': reader.set_buzzer_mode(ADDR, int(input("1=On, 0=Off: ")))
                elif c=='10': reader.find_tags_all_antennas(ADDR)
                elif c=='11': reader.debug_raw_traffic(ADDR)
            except KeyboardInterrupt: break
            except ValueError: print("Invalid Input")
        reader.close()
        