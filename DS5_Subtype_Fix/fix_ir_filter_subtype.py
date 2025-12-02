#!/usr/bin/env python3
"""
RealSense Device Command Tool
-----------------------------------
This script connects to RealSense devices, lists them, and for devices of type
D435, D435i, and D435if allows to adjust to assembly ID

WARNING: The script modifies the device identification ID.
Use with caution.
"""

import sys
import struct
import zlib     # CRC
sys.path.append(r"C:\Work\Git\eraikhel_fork\lrs_sandbox1\build\Debug")
import pyrealsense2 as rs


# ANSI color codes
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
BOLD = "\033[1m"
RESET = "\033[0m"

udw_command = 0x91
udr_command = 0x92
rst_command = 0x20


def list_devices(ctx) -> list[rs.device]:
    devices = ctx.query_devices()
    print("\nConnected RealSense Devices:")
    for i, dev in enumerate(devices):
        name = dev.get_info(rs.camera_info.name)
        serial = dev.get_info(rs.camera_info.serial_number)
        print(f"[{i}] {name} (Serial: {serial})")
    return devices

def build_subtype_table(ir_filter: bool) -> bytes:
    filter_byte = 0x01 if ir_filter else 0x00
    sub_type_data = bytes([filter_byte, 0x00, 0x00, 0x00])
    crc32_val = zlib.crc32(sub_type_data) & 0xFFFFFFFF
    crc_bytes = struct.pack('<I', crc32_val)
    header = struct.pack('<HHI', 0x0001, 0x000A, 4) + b'\xFF\xFF\xFF\xFF' + crc_bytes
    return header + sub_type_data

def update_device_subtype(device, command, irFilter=None) -> None:
    hw_monitor = device.as_debug_protocol()

    print(f"\nSending command: {command}")

    try:
        # Build the command payload
        subtype_table = build_subtype_table(irFilter)
        # Convert to list[int]
        st1 = [int(b) for b in subtype_table]  # Ensure integers 0–255
        payload = hw_monitor.build_command(opcode=udw_command,
                                       param1=0,
                                       param2=0, 
                                       param3=0,
                                       param4=0,
                                       data=st1)
        response = hw_monitor.send_and_receive_raw_data(payload)

        print(f"Response (hex): {[hex(x) for x in response]}")
    except Exception as e:
        print(f"Error sending command: {e}")

    if (len(response) == 4 and response[0]==udw_command):
        print(f" {BOLD}{YELLOW}Device ID in flash NVM was modified{RESET}, re-reading the device sub-type info for confirmation:")
        parse_current_type(device)
        input(f"Update was completed successfully. Press any key to reset device and continue")
        try: # Devices with default sub-type have no data written in NVM
            response = hw_monitor.send_and_receive_raw_data(hw_monitor.build_command(opcode=rst_command))
        except Exception as e:
            pass
    else:
        print(f"Response (hex): {[hex(x) for x in response]}")
        print(f"{RED}updating Flash NVM has failed. Copy and share the above response for analysis{RESET}")

def parse_current_type(device) -> None:
    # Query sub-type
    hw_monitor = device.as_debug_protocol()
    response=[]
    payload = hw_monitor.build_command(opcode=udr_command)
    try: # Devices with default sub-type have no data written in NVM
        response = hw_monitor.send_and_receive_raw_data(payload)
    except Exception as e:
        print(f"Error sending command: {e}")

    # Analyze response
    if not response or len(response) == 0:
        print("No data in NVM → Camera type: DEFAULT")
    else:
        # Check length
        if len(response) < 24:
            print(f"FW Error: Expected 24 bytes, got {len(response)} bytes")
        else:
            # Parse byte 20 (0-based index)
            ir_cut_filter = response[20]
            ir_cut_status = "Present" if ir_cut_filter == 1 else "Off"
            ir_cut_color = GREEN if ir_cut_filter == 1 else RED
            print(f"IR Cut Filter present: {ir_cut_color}{ir_cut_status}{RESET}")

def yes_no_prompt(prompt: str) -> bool:
    while True:
        answer = input(prompt).strip().lower()
        if answer in ['y', 'n']:
            return answer == 'y'
        print(f"{YELLOW}Invalid input{RESET}. Please enter 'y' or 'n'.")

def main():
    print("========================================")
    print("RealSense Device Command Tool")
    print("WARNING: This script modifies device identification ID.")
    print("Please make sure that only one RealSense Device D435(I/F) device is being connected")
    print("========================================")

    ctx = rs.context()
    devices = list_devices(ctx)

    if not devices:
        print("No RealSense devices found.")
        return

    # Filter for D435 series
    target_devices = []
    for dev in devices:
        name = dev.get_info(rs.camera_info.name).lower()
        if "d435" in name:
            target_devices.append(dev)

    if not target_devices:
        print("No D435 series devices found.")
        return

    print("\nTarget Devices:")
    for dev in target_devices:
        print(f"- {dev.get_info(rs.camera_info.name)} (Serial: {dev.get_info(rs.camera_info.serial_number)})")

    for dev in target_devices:
        parse_current_type(dev)
        # Ask user to interactively update the assembly type
        print(f"\nInspect the device appearance/labels for the presence of IR Filter")
        ir_filter = yes_no_prompt(f"{BOLD}{BLUE}Is the device equipped with IR Cut filter? [y/N]: {RESET}")
        ack = yes_no_prompt(f"{BOLD}{YELLOW}WARNING: This will modify device's Flash NVM. Confirm? [y/N]: {RESET}")
        if (ack):
            print(f"\nUpdating device type:")
            update_device_subtype(dev, "UDW", ir_filter)
        else:
            print(f"\nsub-type update is skipped")

    print("\nTask completed.")

if __name__ == "__main__":
    main()
