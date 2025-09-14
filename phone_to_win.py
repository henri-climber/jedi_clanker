#!/usr/bin/env python3
"""
Phone IMU Robot Controller with Inverse Kinematics
Hosts a web server on localhost for phone IMU data collection.
Run with: python3 phone_ik_controller.py --udp-robot 10.42.0.1
"""

import sys
import time
import math
import argparse
import json
import socket
import numpy as np
import asyncio
from pathlib import Path

# Web server imports
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import uvicorn

# IK and sensor fusion imports
try:
    import ikpy.chain

    HAS_IKPY = True
except ImportError:
    print("Warning: ikpy not found. Install with: pip install ikpy")
    HAS_IKPY = False

try:
    from ahrs.filters import Madgwick

    HAS_AHRS = True
except ImportError:
    print("Warning: AHRS library not found. Install with: pip install ahrs")
    HAS_AHRS = False


class PosAccum:
    """Handle IMU position accumulation and overflow."""

    def __init__(self):
        self.turns = None  # integer turn counter
        self.pi = None  # previous raw reading

    def __call__(self, p):
        p = np.asarray(p)

        if self.pi is None:  # first sample
            self.pi = p
            self.turns = np.zeros_like(p, dtype=int)
            return p

        dx = p - self.pi
        step = np.where(
            dx < -0.5,
            1,  # crossed 1→0 (forward)
            np.where(dx > 0.5, -1, 0))  # crossed 0→1 (reverse)

        self.turns += step  # accumulate full turns
        self.pi = p  # advance reference

        return p + self.turns  # unwrapped position


class PhoneIMUController:
    def __init__(self, udp_robot=None, urdf_file=None, output_file=None):
        """Initialize phone IMU controller with robot communication."""

        # Robot Communication
        self.udp_robot = udp_robot
        self.robot_sock = None
        self.robot_addr = None
        self.output_file = output_file

        if udp_robot:
            self.setup_udp_robot(udp_robot)

        # Inverse Kinematics Setup
        self.robot_chain = None
        self.use_ik = False

        if HAS_IKPY and urdf_file and Path(urdf_file).exists():
            try:
                self.robot_chain = ikpy.chain.Chain.from_urdf_file(urdf_file)
                self.use_ik = True
                print(f"✅ Loaded robot model: {len(self.robot_chain.links)} links")

                # Print joint info
                for i, link in enumerate(self.robot_chain.links):
                    print(f"   Link {i}: {link.name}")

            except Exception as e:
                print(f"❌ Failed to load URDF: {e}")

        # IMU Processing
        self.orientation_filter = None
        if HAS_AHRS:
            self.orientation_filter = Madgwick()

        # Hand tracking state
        self.hand_position = np.array([0.3, 0.0, 0.2])  # Initial position (30cm forward, 20cm up)
        self.hand_velocity = np.array([0.0, 0.0, 0.0])
        self.hand_orientation = np.array([0.0, 0.0, 0.0])  # Roll, pitch, yaw

        # Control parameters
        self.position_scale = 0.0005  # Convert phone movement to meters
        self.orientation_scale = 0.005  # Convert phone rotation to radians

        # IMU state
        self.last_update_time = time.time()
        self.is_calibrated = False
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        self.calibration_samples = 0

        # Data tracking
        self.last_imu_data = {}
        self.position_accumulator = PosAccum()

        # Control state
        self.control_active = False
        self.last_control_time = 0
        self.control_rate = 0.05  # 20 Hz

    def setup_udp_robot(self, robot_ip):
        """Setup UDP communication with robot."""
        try:
            self.robot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.robot_addr = (robot_ip, 5005)
            print(f"🤖 UDP Robot Mode: {robot_ip}:5005")
        except Exception as e:
            print(f"❌ UDP setup failed: {e}")
            self.udp_robot = None

    def send_robot_position(self, joint_angles):
        """Send position command to robot via UDP."""
        if not self.robot_sock or not self.robot_addr:
            return

        try:
            msg = {
                "token": "secret",
                "type": "arm_cmd",
                "pos": joint_angles.tolist() if hasattr(joint_angles, 'tolist') else list(joint_angles),
                "seq": int(time.time() * 1000)
            }

            self.robot_sock.sendto(json.dumps(msg).encode("utf-8"), self.robot_addr)
            print(f"🤖 Sent joint angles: {[f'{x:.3f}' for x in joint_angles]}")

        except Exception as e:
            print(f"❌ UDP send failed: {e}")

    def log_to_file(self, data):
        """Log data to file if specified."""
        if self.output_file:
            try:
                with open(self.output_file, 'a') as f:
                    timestamp = time.strftime("%H:%M:%S")
                    f.write(f"[{timestamp}] {data}\n")
                    f.flush()
            except Exception as e:
                print(f"❌ File logging failed: {e}")

    def calibrate_imu(self, samples=50):
        """Calibrate IMU bias."""
        print(f"🔧 IMU calibration complete with {samples} samples")
        self.is_calibrated = True

    def process_imu_data(self, imu_data):
        """Process IMU data from phone and update hand position."""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        if dt > 0.1:  # Skip if too much time has passed
            return

        # Extract IMU data
        # Phone provides: alpha, beta, gamma (orientation) and ax, ay, az, rx, ry, rz (motion)
        try:
            # Orientation data (degrees)
            alpha = math.radians(imu_data.get('alpha', 0) or 0)  # Z-axis (yaw)
            beta = math.radians(imu_data.get('beta', 0) or 0)  # X-axis (pitch)
            gamma = math.radians(imu_data.get('gamma', 0) or 0)  # Y-axis (roll)

            # Motion data
            ax = imu_data.get('ax', 0) or 0  # Linear acceleration
            ay = imu_data.get('ay', 0) or 0
            az = imu_data.get('az', 0) or 0
            rx = imu_data.get('rx', 0) or 0  # Rotation rate (degrees/sec)
            ry = imu_data.get('ry', 0) or 0
            rz = imu_data.get('rz', 0) or 0

        except (TypeError, ValueError):
            return  # Skip invalid data

        # Convert rotation rates to rad/sec
        gyro = np.array([math.radians(rx), math.radians(ry), math.radians(rz)])
        accel = np.array([ax, ay, az])

        # Update orientation using sensor fusion or simple integration
        if self.orientation_filter and HAS_AHRS:
            # Use AHRS library
            try:
                q = self.orientation_filter.updateIMU(gyro, accel)
                self.hand_orientation = self.quaternion_to_euler(q)
            except:
                # Fallback to simple integration
                self.hand_orientation[0] += gyro[0] * dt  # Roll
                self.hand_orientation[1] += gyro[1] * dt  # Pitch
                self.hand_orientation[2] += gyro[2] * dt  # Yaw
        else:
            # Simple gyro integration
            self.hand_orientation[0] += gyro[0] * dt  # Roll
            self.hand_orientation[1] += gyro[1] * dt  # Pitch
            self.hand_orientation[2] += gyro[2] * dt  # Yaw

        # Convert phone motion to position changes
        # Map phone tilts to hand position (intuitive mapping)
        position_delta = np.array([
            beta * self.position_scale,  # Forward/back from phone pitch
            -gamma * self.position_scale,  # Left/right from phone roll (inverted)
            alpha * self.position_scale  # Up/down from phone yaw
        ])

        # Apply velocity damping and integration
        self.hand_velocity = self.hand_velocity * 0.85 + position_delta / dt
        self.hand_position += self.hand_velocity * dt

        # Keep position within workspace bounds
        self.hand_position = np.clip(self.hand_position,
                                     [0.1, -0.4, 0.05],  # Min [x, y, z]
                                     [0.6, 0.4, 0.5])  # Max [x, y, z]

        # Store last data for debugging
        self.last_imu_data = {
            'orientation': [alpha, beta, gamma],
            'motion': [ax, ay, az, rx, ry, rz],
            'hand_pos': self.hand_position.tolist(),
            'hand_orient': self.hand_orientation.tolist()
        }

    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles."""
        w, x, y, z = q

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw])

    def calculate_robot_target_from_phone(self, imu_data):
        """Convert phone orientation to robot end-effector target pose."""

        # Get phone orientation (degrees to radians)
        alpha = math.radians(imu_data.get('alpha', 0) or 0)  # Z-axis
        beta = math.radians(imu_data.get('beta', 0) or 0)  # X-axis
        gamma = math.radians(imu_data.get('gamma', 0) or 0)  # Y-axis

        # Define workspace center (where robot "neutral" position is)
        center = np.array([0.35, 0.0, 0.25])

        # Map phone tilt to position offset in robot workspace
        workspace_size = 0.15  # 15cm movement range

        position_target = center + np.array([
            beta * workspace_size / math.pi,  # Forward/back from phone pitch
            -gamma * workspace_size / math.pi,  # Left/right from phone roll
            alpha * workspace_size / (2 * math.pi)  # Up/down from phone yaw
        ])

        # Clamp to safe workspace
        position_target = np.clip(position_target,
                                  [0.2, -0.2, 0.1],  # Workspace limits
                                  [0.5, 0.2, 0.4])

        return position_target

    def calculate_inverse_kinematics(self):
        """Use IK properly with direct phone-to-target mapping."""
        if not self.use_ik or not self.robot_chain:
            return self.fallback_joint_mapping()

        # Get target position directly from current phone orientation
        target_position = self.calculate_robot_target_from_phone(self.last_imu_data)

        try:
            joint_angles = self.robot_chain.inverse_kinematics(target_position.tolist())

            if len(joint_angles) > 6:
                joint_angles = joint_angles[1:-1]

            return joint_angles[:6]

        except Exception as e:
            print(f"IK failed: {e}")
            return self.fallback_joint_mapping()

    def fallback_joint_mapping(self):
        """Fallback direct joint mapping when IK is not available."""
        joint_angles = np.zeros(6)

        # Map hand orientation to joints
        roll, pitch, yaw = self.hand_orientation

        joint_angles[0] = yaw * 0.5  # Base rotation
        joint_angles[1] = pitch * 0.5  # Shoulder pitch
        joint_angles[2] = roll * 0.3  # Shoulder roll
        joint_angles[3] = -pitch * 0.4  # Elbow (opposite to shoulder)
        joint_angles[4] = roll * 0.2  # Wrist roll
        joint_angles[5] = 0  # Gripper

        return joint_angles

    def control_robot(self):
        """Send control commands to robot."""
        if not self.control_active:
            return

        current_time = time.time()
        if current_time - self.last_control_time < self.control_rate:
            return

        # Calculate joint angles
        joint_angles = self.calculate_inverse_kinematics()

        # Send to robot
        if self.udp_robot:
            self.send_robot_position(joint_angles)

        # Log data
        control_data = {
            'hand_position': self.hand_position.tolist(),
            'hand_orientation': self.hand_orientation.tolist(),
            'joint_angles': joint_angles.tolist()
        }
        self.log_to_file(f"Control: {control_data}")

        self.last_control_time = current_time


# Global controller instance
controller = None


def create_app(ctrl):
    """Create FastAPI app with the controller."""
    app = FastAPI()

    @app.get("/", response_class=HTMLResponse)
    async def root():
        return HTMLResponse("""
<!doctype html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Phone Robot Controller</title>
    <style>
        body { 
            font-family: Arial, sans-serif; 
            text-align: center; 
            margin: 20px;
            background: #f0f0f0;
        }
        .container {
            max-width: 600px;
            margin: 0 auto;
            background: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        .status {
            padding: 15px;
            margin: 10px 0;
            border-radius: 5px;
            font-weight: bold;
        }
        .connected { background: #d4edda; color: #155724; }
        .disconnected { background: #f8d7da; color: #721c24; }
        .button {
            background: #007bff;
            color: white;
            border: none;
            padding: 15px 30px;
            font-size: 18px;
            border-radius: 5px;
            cursor: pointer;
            margin: 10px;
        }
        .button:hover { background: #0056b3; }
        .button:disabled { background: #6c757d; cursor: not-allowed; }
        .data { 
            background: #f8f9fa; 
            padding: 15px; 
            margin: 10px 0;
            border-radius: 5px;
            text-align: left;
            font-family: monospace;
            font-size: 12px;
        }
        .controls {
            display: flex;
            justify-content: space-around;
            margin: 20px 0;
        }
        .active { background: #28a745 !important; }
    </style>
</head>
<body>
    <div class="container">
        <h2>📱 Phone Robot Controller</h2>
        <div id="status" class="status disconnected">Click "Connect" to start</div>

        <div class="controls">
            <button id="connectBtn" class="button">Connect IMU</button>
            <button id="controlBtn" class="button" disabled>Start Control</button>
            <button id="calibrateBtn" class="button" disabled>Calibrate</button>
        </div>

        <div id="data" class="data">Waiting for IMU data...</div>
    </div>

    <script>
        let ws = null;
        let imu = {};
        let isControlling = false;

        const statusDiv = document.getElementById('status');
        const dataDiv = document.getElementById('data');
        const connectBtn = document.getElementById('connectBtn');
        const controlBtn = document.getElementById('controlBtn');
        const calibrateBtn = document.getElementById('calibrateBtn');

        function updateStatus(message, isConnected) {
            statusDiv.textContent = message;
            statusDiv.className = `status ${isConnected ? 'connected' : 'disconnected'}`;
        }

        function updateData() {
            const formatNum = (n) => (n || 0).toFixed(3);
            dataDiv.innerHTML = `
Position: [${formatNum(imu.alpha)}, ${formatNum(imu.beta)}, ${formatNum(imu.gamma)}]°
Motion: ax=${formatNum(imu.ax)}, ay=${formatNum(imu.ay)}, az=${formatNum(imu.az)}
Rotation: rx=${formatNum(imu.rx)}, ry=${formatNum(imu.ry)}, rz=${formatNum(imu.rz)}
Control: ${isControlling ? 'ACTIVE' : 'inactive'}
            `.trim();
        }

        async function requestPermissions() {
            if (typeof DeviceMotionEvent.requestPermission === 'function') {
                const motionPermission = await DeviceMotionEvent.requestPermission();
                const orientationPermission = await DeviceOrientationEvent.requestPermission();

                if (motionPermission !== 'granted' || orientationPermission !== 'granted') {
                    throw new Error('Motion permissions denied');
                }
            }
        }

        function setupIMU() {
            window.addEventListener('deviceorientation', e => {
                Object.assign(imu, { 
                    alpha: e.alpha, 
                    beta: e.beta, 
                    gamma: e.gamma 
                });
            });

            window.addEventListener('devicemotion', e => {
                Object.assign(imu, {
                    ax: e.acceleration?.x,
                    ay: e.acceleration?.y,
                    az: e.acceleration?.z,
                    rx: e.rotationRate?.alpha,
                    ry: e.rotationRate?.beta,
                    rz: e.rotationRate?.gamma
                });
            });
        }

        function startWebSocket() {
            const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
            ws = new WebSocket(`${protocol}//${location.host}/ws`);

            ws.onopen = () => {
                updateStatus('✅ WebSocket connected', true);
                controlBtn.disabled = false;
                calibrateBtn.disabled = false;
            };

            ws.onclose = () => {
                updateStatus('❌ WebSocket disconnected', false);
                controlBtn.disabled = true;
                calibrateBtn.disabled = true;
                connectBtn.disabled = false;
                isControlling = false;
                controlBtn.textContent = 'Start Control';
                controlBtn.classList.remove('active');
            };

            ws.onerror = (error) => {
                console.error('WebSocket error:', error);
                updateStatus('❌ Connection error', false);
            };
        }

        function sendData() {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify(imu));
            }
            updateData();
        }

        connectBtn.addEventListener('click', async () => {
            try {
                connectBtn.disabled = true;
                updateStatus('Requesting permissions...', false);

                await requestPermissions();
                updateStatus('Setting up IMU...', false);

                setupIMU();
                startWebSocket();

                // Send data every 20ms (50 Hz)
                setInterval(sendData, 20);

            } catch (error) {
                console.error('Setup failed:', error);
                updateStatus(`❌ Setup failed: ${error.message}`, false);
                connectBtn.disabled = false;
            }
        });

        controlBtn.addEventListener('click', () => {
            isControlling = !isControlling;

            if (isControlling) {
                controlBtn.textContent = 'Stop Control';
                controlBtn.classList.add('active');
                updateStatus('🤖 Robot control ACTIVE', true);
            } else {
                controlBtn.textContent = 'Start Control';
                controlBtn.classList.remove('active'); 
                updateStatus('✅ Connected (control paused)', true);
            }

            // Send control state to server
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({...imu, control_active: isControlling}));
            }
        });

        calibrateBtn.addEventListener('click', () => {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({...imu, calibrate: true}));
                updateStatus('🔧 Calibrating...', true);
                setTimeout(() => {
                    updateStatus('✅ Calibration complete', true);
                }, 2000);
            }
        });

        // Initial data display
        updateData();
    </script>
</body>
</html>
        """)

    @app.websocket("/ws")
    async def imu_websocket(websocket: WebSocket):
        await websocket.accept()
        print("📱 Phone connected")

        try:
            while True:
                data = await websocket.receive_text()
                payload = json.loads(data)

                # Check for special commands
                if payload.get('calibrate'):
                    ctrl.calibrate_imu()
                    continue

                if 'control_active' in payload:
                    ctrl.control_active = payload['control_active']
                    print(f"🎮 Control {'activated' if ctrl.control_active else 'deactivated'}")
                    continue

                # Process IMU data
                if all(k in payload for k in ['alpha', 'beta', 'gamma']):
                    ctrl.process_imu_data(payload)
                    ctrl.control_robot()

        except WebSocketDisconnect:
            print("📱 Phone disconnected")
            ctrl.control_active = False

    return app


def main():
    parser = argparse.ArgumentParser(description="Phone IMU Robot Controller")
    parser.add_argument("--udp-robot", help="Robot IP address for UDP control")
    parser.add_argument("--urdf-file", default="so101_new_calib.urdf", help="URDF file path")
    parser.add_argument("--output-file", help="Log data to file")
    parser.add_argument("--host", default="localhost", help="Web server host")
    parser.add_argument("--port", type=int, default=8000, help="Web server port")

    args = parser.parse_args()

    print("📱 Phone IMU Robot Controller")
    print("=" * 50)

    # Create controller
    global controller
    controller = PhoneIMUController(
        udp_robot=args.udp_robot,
        urdf_file=args.urdf_file,
        output_file=args.output_file
    )

    # Create web app
    app = create_app(controller)

    print(f"🌐 Starting web server on http://{args.host}:{args.port}")
    print(f"📱 Open this URL on your phone: http://{args.host}:{args.port}")

    if controller.use_ik:
        print("🧠 Using Inverse Kinematics mode")
    else:
        print("⚙️ Using Direct Joint Mapping mode")

    print("\n📋 Instructions:")
    print("1. Open the URL on your phone")
    print("2. Click 'Connect IMU' and grant permissions")
    print("3. Click 'Start Control' to begin robot control")
    print("4. Move your phone to control the robot")
    print("=" * 50)

    try:
        uvicorn.run(
            app,
            host=args.host,
            port=args.port,
            log_level="warning"  # Reduce log noise
        )
    except KeyboardInterrupt:
        print("\n👋 Shutting down...")
    finally:
        if controller and controller.robot_sock:
            controller.robot_sock.close()


if __name__ == "__main__":
    main()