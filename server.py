from flask import Flask, request, jsonify
from flask_cors import CORS
import time

app = Flask(__name__)
CORS(app)

# Store the latest command
current_command = {
    "direction": "neutral",
    "speed": 0,
    "steering": 0
}

# Track when the ESP32 last fetched data
last_esp_activity = 0
last_client_activity = 0

@app.before_request
def log_request_info():
    print(f"[{time.strftime('%X')}] Request: {request.method} {request.path} from {request.remote_addr}")

@app.route('/data', methods=['GET', 'POST'])
def handle_data():
    global current_command, last_esp_activity, last_client_activity
    
    if request.method == 'POST':
        # Web app sending new command
        data = request.json
        if data:
            current_command = data
            last_client_activity = time.time()
        return jsonify({"status": "ok"})
    
    elif request.method == 'GET':
        # Car polling for command - update heartbeat
        last_esp_activity = time.time()
        
        # FAILSAFE: If client hasn't sent data in 2 seconds, stop the car
        if time.time() - last_client_activity > 2.0:
            current_command = {
                "direction": "neutral",
                "speed": 0,
                "steering": 0
            }
            
        return jsonify(current_command)

@app.route('/status', methods=['GET'])
def get_status():
    # If ESP32 fetched data within the last 2 seconds, consider it connected
    is_connected = (time.time() - last_esp_activity) < 2.0
    return jsonify({"car_connected": is_connected})

if __name__ == '__main__':
    # Host='0.0.0.0' allows access from external devices (like the ESP32)
    app.run(host='0.0.0.0', port=5050)
