import React, { useState, useEffect, useRef } from "react";
import "./App.css";

function App() {
  // ----- State -----
  const [targetSpeed, setTargetSpeed] = useState(0);       // target speed from keys
  const [targetSteering, setTargetSteering] = useState(0); // target steering from keys
  const [speed, setSpeed] = useState(0);                   // actual smoothed speed
  const [steering, setSteering] = useState(0);             // actual smoothed steering
  const [direction, setDirection] = useState("neutral");
  const [isCarConnected, setIsCarConnected] = useState(false);

  const [maxSpeed, setMaxSpeed] = useState(1);           // slider to scale speed
  const [steeringSensitivity, setSteeringSensitivity] = useState(1); // slider scale steering

  const keysPressed = useRef({});

  const RAMP_RATE = 0.05; // change per update

  // ----- Handle key presses -----
  const handleKeyDown = (e) => { keysPressed.current[e.key] = true; };
  const handleKeyUp = (e) => { keysPressed.current[e.key] = false; };

  // ----- Poll for Connection Status -----
  useEffect(() => {
    const statusInterval = setInterval(() => {
      fetch("http://192.168.1.165:5050/status")
        .then(res => res.json())
        .then(data => setIsCarConnected(data.car_connected))
        .catch(err => setIsCarConnected(false));
    }, 1000); // Check every 1 second
    return () => clearInterval(statusInterval);
  }, []);

  // ----- Main update loop -----
  useEffect(() => {
    const interval = setInterval(() => {
      // --- Determine target speed & direction ---
      let dir = "neutral";
      let spdTarget = 0;

      if (keysPressed.current["ArrowUp"]) {
        dir = "forward";
        spdTarget = maxSpeed;
      } else if (keysPressed.current["ArrowDown"]) {
        dir = "reverse";
        spdTarget = maxSpeed;
      }

      setTargetSpeed(spdTarget);
      setDirection(dir);

      // --- Determine target steering ---
      let steerTarget = 0;
      if (keysPressed.current["ArrowLeft"]) steerTarget -= 1;
      if (keysPressed.current["ArrowRight"]) steerTarget += 1;
      steerTarget *= 0.5 * steeringSensitivity; // scale
      if (dir === "neutral") steerTarget = 0; // no steering when stopped

      setTargetSteering(steerTarget);

      // --- Smooth ramping ---
      setSpeed(prev => {
        if (prev < spdTarget) return Math.min(prev + RAMP_RATE, spdTarget);
        else if (prev > spdTarget) return Math.max(prev - RAMP_RATE, spdTarget);
        else return prev;
      });

      setSteering(prev => {
        if (prev < steerTarget) return Math.min(prev + RAMP_RATE, steerTarget);
        else if (prev > steerTarget) return Math.max(prev - RAMP_RATE, steerTarget);
        else return prev;
      });

      // --- Send JSON to ESP32 ---
      // ALWAYS send to ensure we send the 'neutral'/stop command
      fetch("http://192.168.1.165:5050/data", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ direction: dir, speed: speed, steering: steering }),
      }).catch(err => console.log("ESP32 not reachable:", err));

    }, 50); // 50ms update interval
    return () => clearInterval(interval);
  }, [maxSpeed, steeringSensitivity, speed, steering]);

  // ----- Key listeners -----
  useEffect(() => {
    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);
    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
    };
  }, []);

  return (
    <div className="App">
      <h1>ESP32 Smooth Motor Control</h1>

      {/* Connection Status Indicator */}
      <div style={{
        padding: "10px",
        marginBottom: "20px",
        backgroundColor: isCarConnected ? "#d4edda" : "#f8d7da",
        color: isCarConnected ? "#155724" : "#721c24",
        border: `1px solid ${isCarConnected ? "#c3e6cb" : "#f5c6cb"}`,
        borderRadius: "5px",
        display: "inline-block"
      }}>
        <strong>Status: </strong>
        {isCarConnected ? "Connected (Car Online)" : "Disconnected (Car Offline)"}
      </div>

      <p>Direction: {direction}</p>
      <p>Speed: {speed.toFixed(2)}</p>
      <p>Steering: {steering.toFixed(2)}</p>

      {/* Controls */}
      <div style={{ marginTop: "20px", width: "300px" }}>
        <div>
          <label>Max Speed: {maxSpeed.toFixed(2)}</label>
          <input type="range" min="0" max="1" step="0.01"
            value={maxSpeed} onChange={(e) => setMaxSpeed(parseFloat(e.target.value))} />
        </div>
        <div>
          <label>Steering Sensitivity: {steeringSensitivity.toFixed(2)}</label>
          <input type="range" min="0" max="1" step="0.01"
            value={steeringSensitivity} onChange={(e) => setSteeringSensitivity(parseFloat(e.target.value))} />
        </div>
      </div>

      {/* Visual bars */}
      <div style={{ marginTop: "20px", width: "300px" }}>
        <div style={{ width: `${speed * 100}%`, height: "20px", background: "green", marginBottom: "5px" }}>Speed</div>
        <div style={{ width: "100%", height: "20px", background: "#ddd", position: "relative" }}>
          <div style={{
            position: "absolute",
            left: `${(steering + 1) * 50}%`,
            top: 0,
            width: "5px",
            height: "100%",
            background: "red"
          }}></div>
          Steering
        </div>
      </div>
    </div>
  );
}

export default App;
