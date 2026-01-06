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

Check Notion for usage instructions.
[https://www.notion.so/hullbot/Odoo-IoT-UHF-RFID-Integration-24de63357ee78099ad2dcc605eb043e9?source=copy_link#2c9e63357ee78098911cd5713ebdf60e]

