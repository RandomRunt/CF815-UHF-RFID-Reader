# CF815-UHF-RFID-Reader

This repository contains Python scripts for interfacing with a CHAFON CF815 UHF RFID Reader over TCP/IP.

## Overview

The software provided in this repository enables communication with the CF815 reader. It includes functionality for establishing a network connection, sending commands, and processing responses from the RFID reader.

## File Descriptions

The repository consists of the following three Python files:

1.  **CF815_v2.0.py**
    This script serves as the primary controller for the reader using Python's native `socket` library for direct TCP/IP communication. It features a command-line interface (CLI) that allows users to initialize the reader, perform asynchronous and buffered inventory scans, and configure device settings such as RF power and scan duration.

2.  **CF815_v1.0.py**
    This script is an implementation that utilizes the `pyserial` library to manage the connection (using `socket://` URLs). It offers similar functionality to v2.0, including tag inventory and parameter configuration, but relies on the serial abstraction layer rather than raw sockets.

3.  **serial_bridge.py**
    This diagnostic utility acts as a bridge between a physical serial port (connected to the reader) and a virtual serial port (connected to control software). It intercepts and logs all hexadecimal traffic passing between the hardware and the application, facilitating protocol analysis and debugging.

## Prerequisites

*   Python 3.9 or higher for pyserial support
*   Network connectivity to the CHAFON CF815 Reader

## Usage

### 1. CF815_v2.0.py (Socket Implementation)

Recommended for production use — connects directly over TCP/IP using Python's native `socket` library, and handles low-level TCP fragmentation/buffering without a serial wrapper.

**Key features**
*   Native socket communication (`socket.AF_INET`) for robust networking
*   Auto-initialization: sets Answer Mode, enables Antenna 1, and sets RF power to 30 dBm on connect
*   Asynchronous inventory: continuous scanning loop that toggles target bits (A/B) to maximize tag detection
*   Buffered inventory: supports the reader's internal memory buffer mode (Command `0x18`)

**Configuration**
Set connection details in the `if __name__ == "__main__":` block at the bottom of the script:
*   `READER_IP` — default `"192.168.1.200"`
*   `READER_PORT` — default `2022`

**Running it**
Run the script from the command line. A menu will appear with the following options:

| # | Option | Description |
|---|--------|-------------|
| 1 | Info | Get firmware version and hardware details |
| 2 | AsyncScan | Continuous inventory mode (recommended for testing detection) |
| 3 | BufScan | Buffered inventory mode (reader stores tags internally, then transmits) |
| 4 | PersistentTime | Set the scan duration in non-volatile memory |
| 5 | Power | Adjust RF output power (0-30 dBm) |
| 6 | BufCount/BufData | Retrieve data from the reader's internal buffer |
| 7 | AntHealth | Check antenna return loss (VSWR) |
| 8 | Buzzer | Toggle the hardware buzzer |
| 9 | PortScan | Cycle through antenna ports 1-4 |
| 10 | Raw | Monitor raw hexadecimal traffic |

### 2. CF815_v1.0.py (PySerial Implementation)

Provides similar functionality to v2.0 but uses `pyserial`'s `socket://` URL handler, abstracting the TCP connection as a serial port object — useful for integrating with legacy applications that expect a serial interface rather than a raw socket.

**Key features**
*   Serial abstraction: treats the TCP connection as a standard serial port (`self.ser.read()`)
*   CRC calculation: implements the CRC16 algorithm (polynomial `0x8408`) required by the CF815 protocol
*   Detailed parsing of reader information, frequency bands, and protocol support

**Configuration**
*   `READER_IP` — default `"192.168.1.200"`
*   `READER_PORT` — default `2022`
*   `BAUDRATE` — default `57600` (virtual baud rate for the socket handler)

**Running it**
Run the script from the command line. The menu is similar to v2.0 but focuses on:

1. Get Reader Info — displays detailed protocol and frequency support
2. Inventory Scan with Buffer — performs a timed scan using internal memory
3. Manual Timing Scan — performs an immediate inventory scan
4. Set Scan Time — configures the inventory duration
5. Modify Antenna Power — sets the gain

### 3. serial_bridge.py (Diagnostic Bridge)

A man-in-the-middle (MitM) diagnostic tool that sits between the physical reader hardware and a control application (such as the official Windows demo software). It intercepts, logs, and forwards serial traffic in real time.

**Architecture**
*   **Real Port** — connects to the physical hardware (e.g. the reader plugged in over USB/Serial)
*   **Virtual Port** — connects to a virtual serial pair (e.g. `com0com`) that the control application connects to

**Key features**
*   Bi-directional logging: logs traffic from App→Reader and Reader→App separately
*   Color coding: terminal colors distinguish direction (green for commands, cyan for responses)
*   Hex output: all data printed in uppercase hex for easy protocol analysis

**Configuration**
Edit the configuration section at the top of the file:
*   `REAL_PORT` — the COM port of the physical reader (e.g. `'COM7'`)
*   `VIRTUAL_PORT` — one end of a virtual COM pair (e.g. `'COM21'`)
*   `BAUDRATE` — match the reader's baud rate (usually `57600`)

**Running it**
1. Ensure the control application is closed.
2. Configure a virtual COM pair (e.g. COM20 <-> COM21).
3. Set `VIRTUAL_PORT` in the script to `'COM21'`.
4. Run the script.
5. Open the control application and connect it to `'COM20'`.
6. Observe the traffic log in the terminal.
