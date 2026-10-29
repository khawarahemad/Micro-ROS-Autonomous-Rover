#!/usr/bin/env python3
"""
Interactive Rover Hardware CLI & Sensor Monitor
Centurion University - Micro-ROS Autonomous Environmental Rover
"""

import sys
import time
import threading
import serial

PORT = "/dev/cu.usbserial-0001"

def detect_baud(port):
    for baud in [115200, 9600]:
        try:
            s = serial.Serial(port, baud, timeout=1)
            time.sleep(0.5)
            s.reset_input_buffer()
            t0 = time.time()
            while time.time() - t0 < 1.5:
                line = s.readline().decode('utf-8', errors='ignore').strip()
                if line and (',' in line or 'TELEM' in line or 'MICRO-ROS' in line):
                    s.close()
                    return baud
            s.close()
        except Exception:
            pass
    return 115200 # Default to 115200 for new diagnostic firmware

def listen_serial(ser, stop_event):
    while not stop_event.is_set():
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"\r[ESP32] {line}\n> ", end="", flush=True)
            else:
                time.sleep(0.05)
        except Exception:
            break

def main():
    print("=" * 65)
    print("🤖 MICRO-ROS AUTONOMOUS ROVER - INTERACTIVE CLI CONSOLE")
    print("=" * 65)
    
    baud = detect_baud(PORT)
    print(f"Connecting to {PORT} at {baud} baud...")

    try:
        ser = serial.Serial(PORT, baud, timeout=1)
    except Exception as e:
        print(f"❌ Error opening port: {e}")
        return

    print("✅ Connected!")
    print("\nAvailable Commands:")
    print("  f : Motor Forward 1s       |  b : Motor Backward 1s")
    print("  l : Turn Left 0.5s         |  r : Turn Right 0.5s")
    print("  x : Emergency Stop Motors  |  s : Sample All Sensors")
    print("  c : Toggle Telemetry Loop  |  q : Quit Console")
    print("-" * 65)

    stop_event = threading.Event()
    listener_thread = threading.Thread(target=listen_serial, args=(ser, stop_event), daemon=True)
    listener_thread.start()

    try:
        while True:
            cmd = input("> ").strip().lower()
            if not cmd:
                continue
            if cmd == 'q':
                print("Exiting...")
                break
            # Send character to ESP32
            ser.write(f"{cmd}\n".encode('utf-8'))
            time.sleep(0.1)
    except (KeyboardInterrupt, EOFError):
        print("\nStopping...")
    finally:
        stop_event.set()
        ser.close()

if __name__ == "__main__":
    main()

