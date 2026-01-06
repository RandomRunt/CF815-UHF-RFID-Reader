import serial
import time
import threading
import sys
import binascii

# ===============================================================
# CONFIGURATION
# ===============================================================
# The port the physical reader is plugged into
REAL_PORT = 'COM7'       

# The end of the virtual pair that THIS script connects to.
# (The Windows App will connect to the OTHER end, e.g., COM20)
VIRTUAL_PORT = 'COM21'   

# Match this to your reader's actual settings (usually 57600 or 115200)
BAUD_RATE = 57600        
# ===============================================================

def forward(source, destination, direction_label, color_code):
    """
    Continuous loop that reads from source and writes to destination.
    """
    try:
        while True:
            # Check if data is waiting
            if source.in_waiting > 0:
                # Read everything available
                data = source.read(source.in_waiting)
                
                # 1. PRINT IT (The Sniffing Part)
                hex_str = binascii.hexlify(data).decode().upper()
                # Format nicely with spaces: A0 04 01...
                formatted_hex = " ".join([hex_str[i:i+2] for i in range(0, len(hex_str), 2)])
                
                print(f"{color_code}[{direction_label}] {formatted_hex}\033[0m")
                
                # 2. FORWARD IT (The Bridge Part)
                destination.write(data)
                destination.flush()
            
            # Tiny sleep to prevent 100% CPU usage
            time.sleep(0.001)
            
    except serial.SerialException as e:
        print(f"\n[!] Serial Error in {direction_label} thread: {e}")
        # We don't exit here, we let the main thread handle cleanup
    except Exception as e:
        print(f"\n[!] Error: {e}")

def main():
    print("=== SERIAL SNIFFER BRIDGE ===")
    print(f"1. Opening Real Hardware: {REAL_PORT}...")
    print(f"2. Opening Virtual Side : {VIRTUAL_PORT}...")
    
    try:
        real_ser = serial.Serial(REAL_PORT, BAUD_RATE, timeout=0.1)
        virt_ser = serial.Serial(VIRTUAL_PORT, BAUD_RATE, timeout=0.1)
        
        print("\nbridge Active! Connect your Windows App to the PARTNER of " + VIRTUAL_PORT)
        print("(For example, if you are using COM21, connect App to COM20)")
        print("\n\033[92mGREEN = APP -> READER (Command)\033[0m")
        print("\033[96mCYAN  = READER -> APP (Response)\033[0m")
        print("-" * 60)

        # Thread 1: Listen to Virtual (App), Forward to Real (Reader)
        # Color \033[92m is Green
        t1 = threading.Thread(target=forward, 
                              args=(virt_ser, real_ser, "APP -> RDR", "\033[92m"))
        t1.daemon = True
        t1.start()

        # Thread 2: Listen to Real (Reader), Forward to Virtual (App)
        # Color \033[96m is Cyan
        t2 = threading.Thread(target=forward, 
                              args=(real_ser, virt_ser, "RDR -> APP", "\033[96m"))
        t2.daemon = True
        t2.start()

        # Keep main script running until Ctrl+C
        while True:
            time.sleep(1)

    except serial.SerialException as e:
        print(f"\n[CRITICAL] Could not open ports: {e}")
        print("Check: Is the Windows App already connected to COM7? Disconnect it first!")
    except KeyboardInterrupt:
        print("\nStopping bridge...")
    finally:
        # Cleanup
        if 'real_ser' in locals() and real_ser.is_open: real_ser.close()
        if 'virt_ser' in locals() and virt_ser.is_open: virt_ser.close()

if __name__ == "__main__":
    main()
    