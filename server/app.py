import time
import json
import threading
from flask import Flask, jsonify, request, send_from_directory
import serial

SERIAL_PORT = "/dev/cu.usbserial-0001"
BAUD_RATE = 115200

app = Flask(__name__, static_folder="static")

# Shared state
rover_data = {
    "temp": 28.5,
    "hum": 85.0,
    "light": 1,
    "ir": 1,
    "distance": -1.0,
    "action": "STOPPED",
    "mode": "manual",
    "pwm": 0,
    "speed": 200,
    "connected": False
}

ser_conn = None
ser_lock = threading.Lock()

def serial_reader_thread():
    global ser_conn, rover_data
    while True:
        try:
            if not ser_conn or not ser_conn.is_open:
                try:
                    ser_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
                    rover_data["connected"] = True
                    print(f"✅ Connected to ESP32 on {SERIAL_PORT}")
                except Exception as e:
                    rover_data["connected"] = False
                    time.sleep(2)
                    continue

            with ser_lock:
                if ser_conn.in_waiting:
                    line = ser_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        parse_telemetry(line)
                else:
                    time.sleep(0.04)
        except Exception as e:
            print("Serial loop error:", e)
            rover_data["connected"] = False
            time.sleep(1)

def parse_telemetry(line):
    global rover_data
    if "[TELEM]" in line:
        parts = line.split("|")
        for part in parts:
            part = part.strip()
            if "Temp:" in part:
                try:
                    val = part.split("Temp:")[1].replace("C", "").strip()
                    rover_data["temp"] = float(val)
                except: pass
            elif "Hum:" in part:
                try:
                    val = part.split("Hum:")[1].replace("%", "").strip()
                    rover_data["hum"] = float(val)
                except: pass
            elif "Sonar:" in part:
                try:
                    val = part.split("Sonar:")[1].replace("cm", "").strip()
                    rover_data["distance"] = float(val)
                except: pass
            elif "IR:" in part:
                try:
                    val = part.split("IR:")[1].strip()
                    rover_data["ir"] = int(val)
                except: pass
            elif "LDR:" in part:
                try:
                    val = part.split("LDR:")[1].strip()
                    rover_data["light"] = int(val)
                except: pass
            elif "PWM:" in part:
                try:
                    val = part.split("PWM:")[1].strip()
                    rover_data["pwm"] = int(val)
                except: pass
            elif "Spd:" in part:
                try:
                    val = part.split("Spd:")[1].strip()
                    rover_data["speed"] = int(val)
                except: pass
            elif "Act:" in part:
                try:
                    val = part.split("Act:")[1].strip()
                    rover_data["action"] = val
                except: pass
            elif "Mode:" in part:
                try:
                    val = part.split("Mode:")[1].strip()
                    rover_data["mode"] = val
                except: pass

@app.route("/")
def index():
    return send_from_directory("static", "index.html")

@app.route("/api/telemetry")
@app.route("/telemetry")
def get_telemetry():
    return jsonify(rover_data)

@app.route("/api/drive")
@app.route("/drive")
def drive_command():
    direction = request.args.get("dir", "x")
    speed = request.args.get("spd", str(rover_data.get("speed", 200)))
    with ser_lock:
        if ser_conn and ser_conn.is_open:
            try:
                cmd_str = f"{direction} {speed}\n" if direction != 'x' else "x\n"
                ser_conn.write(cmd_str.encode("utf-8"))
                if direction == 'f': rover_data["action"] = "FORWARD"
                elif direction == 'b': rover_data["action"] = "BACKWARD"
                elif direction == 'l': rover_data["action"] = "TURNING LEFT"
                elif direction == 'r': rover_data["action"] = "TURNING RIGHT"
                elif direction == 'x': rover_data["action"] = "STOPPED"
                return jsonify({"status": "ok", "action": rover_data["action"]})
            except Exception as e:
                return jsonify({"status": "error", "message": str(e)}), 500
    return jsonify({"status": "offline"}), 503

@app.route("/api/speed")
@app.route("/speed")
def set_speed():
    spd = request.args.get("v", "200")
    try:
        val = max(140, min(255, int(spd)))
        rover_data["speed"] = val
        with ser_lock:
            if ser_conn and ser_conn.is_open:
                ser_conn.write(f"spd {val}\n".encode("utf-8"))
    except: pass
    return jsonify({"status": "ok", "speed": rover_data["speed"]})

@app.route("/api/mode")
@app.route("/mode")
def set_mode():
    mode = request.args.get("m", "manual")
    rover_data["mode"] = mode
    with ser_lock:
        if ser_conn and ser_conn.is_open:
            try:
                ser_conn.write(f"m {mode}\n".encode("utf-8"))
            except: pass
    return jsonify({"status": "ok", "mode": mode})

if __name__ == "__main__":
    t = threading.Thread(target=serial_reader_thread, daemon=True)
    t.start()
    print("🚀 Starting Rover Web Dashboard at http://localhost:8080 and http://192.168.1.2:8080")
    app.run(host="0.0.0.0", port=8080, debug=False)
