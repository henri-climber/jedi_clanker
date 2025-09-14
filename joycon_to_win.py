#!/usr/bin/env python3
"""
Enhanced Joy-Con Robot Controller with Inverse Kinematics
Run with: python3 joycon_ik_controller.py --ssh-host HOSTNAME --ssh-user USERNAME
Or use: python3 joycon_ik_controller.py --output-file robot_commands.txt
Or use: python3 joycon_ik_controller.py --udp-robot IP_ADDRESS (for direct position control)
"""

import sys
import time
import math
import subprocess
import argparse
import os
import json
import socket
import numpy as np

# Add these imports to your existing script
try:
    from pyjoycon import JoyCon, get_R_id, get_L_id
except ImportError as e:
    print(f"Error importing pyjoycon: {e}")
    print("Make sure you installed it with: pip install joycon-python hidapi pyglm")
    sys.exit(1)

try:
    import ikpy.chain
    import ikpy.utils.plot as plot_utils

    HAS_IKPY = True
except ImportError:
    print("Warning: ikpy not found. Install with: pip install ikpy")
    print("Falling back to direct joint mapping mode")
    HAS_IKPY = False

try:
    from ahrs.filters import Madgwick
    import quaternion

    HAS_AHRS = True
except ImportError:
    print("Warning: AHRS library not found. Install with: pip install ahrs numpy-quaternion")
    print("Using simplified orientation tracking")
    HAS_AHRS = False


class JoyConIKController:
    def __init__(self, ssh_host=None, ssh_user=None, output_file=None, output_mode="console",
                 udp_robot=None, urdf_file=None):
        """Initialize with SSH, file, or UDP robot output capabilities."""
        self.left_joycon = None
        self.right_joycon = None
        self.running = True

        # Output Configuration
        self.ssh_host = ssh_host
        self.ssh_user = ssh_user
        self.ssh_port = 22
        self.output_file = output_file
        self.output_mode = output_mode
        self.udp_robot = udp_robot

        # UDP Robot Communication (for direct position control)
        self.robot_sock = None
        self.robot_addr = None
        if udp_robot:
            self.setup_udp_robot(udp_robot)

        # Inverse Kinematics Setup
        self.robot_chain = None
        self.use_ik = False
        if HAS_IKPY and urdf_file and os.path.exists(urdf_file):
            try:
                self.robot_chain = ikpy.chain.Chain.from_urdf_file(urdf_file)
                self.use_ik = True
                print(f"✅ Loaded robot model: {len(self.robot_chain.links)} links")
            except Exception as e:
                print(f"❌ Failed to load URDF: {e}")

        # IMU Processing
        self.orientation_filter = None
        if HAS_AHRS:
            self.orientation_filter = Madgwick()

        # Hand position tracking
        self.hand_position = np.array([0.3, 0.0, 0.2])  # Initial position (30cm forward, 20cm up)
        self.hand_velocity = np.array([0.0, 0.0, 0.0])
        self.hand_orientation = np.array([0.0, 0.0, 0.0])  # Roll, pitch, yaw

        # Joint to keyboard mapping for SO-101 ARM (fallback mode)
        self.joint_keys = {
            1: {"up": "q", "down": "a", "name": "Base"},
            2: {"up": "w", "down": "s", "name": "Shoulder"},
            3: {"up": "e", "down": "d", "name": "Elbow"},
            4: {"up": "r", "down": "f", "name": "Wrist_Pitch"},
            5: {"up": "t", "down": "g", "name": "Wrist_Roll"},
            6: {"up": "y", "down": "h", "name": "Gripper"}
        }

        # Control parameters
        self.sensitivity = 1.0
        self.position_scale = 0.001  # Convert Joy-Con movement to meters
        self.orientation_scale = 0.01  # Convert Joy-Con rotation to radians
        self.threshold = 5  # Minimum movement threshold

        # Tracking state
        self.last_gyro = {"x": 0, "y": 0, "z": 0}
        self.last_accel = {"x": 0, "y": 0, "z": 0}
        self.last_update_time = time.time()
        self.is_calibrated = False
        self.calibration_samples = 0
        self.gyro_bias = {"x": 0, "y": 0, "z": 0}

        # Setup output method
        self.setup_output()
        self.connect_joycons()

    def setup_udp_robot(self, robot_ip):
        """Setup UDP communication with robot."""
        try:
            self.robot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.robot_addr = (robot_ip, 5005)  # Your existing robot port
            print(f"🤖 UDP Robot Mode: {robot_ip}:5005")
        except Exception as e:
            print(f"❌ UDP setup failed: {e}")
            self.udp_robot = None

    def send_robot_position(self, joint_angles):
        """Send position command directly to robot via UDP."""
        if not self.robot_sock or not self.robot_addr:
            return

        try:
            msg = {
                "token": "secret",  # Your existing token
                "type": "arm_cmd",
                "pos": joint_angles.tolist() if hasattr(joint_angles, 'tolist') else list(joint_angles),
                "seq": int(time.time() * 1000)
            }

            self.robot_sock.sendto(json.dumps(msg).encode("utf-8"), self.robot_addr)
            print(f"🤖 UDP: Sent joint angles: {[f'{x:.3f}' for x in joint_angles]}")

        except Exception as e:
            print(f"❌ UDP send failed: {e}")

    def setup_output(self):
        """Configure the output method."""
        if self.output_mode == "ssh" and self.ssh_host and self.ssh_user:
            print(f"🌐 SSH Output Mode: {self.ssh_user}@{self.ssh_host}")
            # Test SSH connection
            try:
                result = subprocess.run([
                    "ssh", "-o", "ConnectTimeout=5",
                    f"{self.ssh_user}@{self.ssh_host}",
                    "echo 'SSH test successful'"
                ], capture_output=True, text=True, timeout=10)
                if result.returncode == 0:
                    print("✅ SSH connection test successful")
                else:
                    print(f"❌ SSH connection failed: {result.stderr}")
                    self.output_mode = "console"
            except Exception as e:
                print(f"❌ SSH setup failed: {e}")
                self.output_mode = "console"

        elif self.output_mode == "file" and self.output_file:
            print(f"📁 File Output Mode: {self.output_file}")
            try:
                with open(self.output_file, 'w') as f:
                    f.write(f"# Joy-Con Commands - {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                print("✅ Output file ready")
            except Exception as e:
                print(f"❌ File setup failed: {e}")
                self.output_mode = "console"
        else:
            print("🖥️  Console Output Mode")
            self.output_mode = "console"

    def connect_joycons(self):
        """Connect to Joy-Con controllers."""
        try:
            # Try to connect to right Joy-Con (the one with IMU data)
            joycon_id = get_R_id()
            if joycon_id:
                self.right_joycon = JoyCon(*joycon_id)
                print("✅ Connected to Right Joy-Con")
            else:
                print("❌ Right Joy-Con not found")

        except Exception as e:
            print(f"❌ Joy-Con connection failed: {e}")
            sys.exit(1)

    def calibrate_imu(self, samples=100):
        """Calibrate IMU by averaging bias over several samples."""
        if not self.right_joycon:
            return

        print(f"🔧 Calibrating IMU... Keep Joy-Con still for {samples} samples")

        gyro_sum = {"x": 0, "y": 0, "z": 0}

        for i in range(samples):
            status = self.right_joycon.get_status()
            if 'gyro' in status:
                gyro = status['gyro']
                gyro_sum["x"] += gyro["x"]
                gyro_sum["y"] += gyro["y"]
                gyro_sum["z"] += gyro["z"]
            time.sleep(0.01)

            if i % 20 == 0:
                print(f"📊 Calibration progress: {i}/{samples}")

        # Calculate bias
        self.gyro_bias["x"] = gyro_sum["x"] / samples
        self.gyro_bias["y"] = gyro_sum["y"] / samples
        self.gyro_bias["z"] = gyro_sum["z"] / samples

        self.is_calibrated = True
        print(f"✅ IMU calibrated. Bias: {self.gyro_bias}")

    def process_imu_data(self, gyro, accel):
        """Process IMU data to estimate hand position and orientation."""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        if dt > 0.1:  # Skip if too much time has passed
            return

        # Remove bias from gyro
        if self.is_calibrated:
            gyro_corrected = {
                "x": gyro["x"] - self.gyro_bias["x"],
                "y": gyro["y"] - self.gyro_bias["y"],
                "z": gyro["z"] - self.gyro_bias["z"]
            }
        else:
            gyro_corrected = gyro

        # Update orientation using sensor fusion or simple integration
        if self.orientation_filter and HAS_AHRS:
            # Use AHRS library for better orientation tracking
            gyro_array = np.array([gyro_corrected["x"], gyro_corrected["y"], gyro_corrected["z"]])
            accel_array = np.array([accel["x"], accel["y"], accel["z"]])

            # Convert to proper units (degrees/sec to rad/sec)
            gyro_rad = np.deg2rad(gyro_array)

            # Update filter
            q = self.orientation_filter.updateIMU(gyro_rad, accel_array)

            # Convert quaternion to Euler angles
            # This is a simplified conversion - you might want a more robust one
            self.hand_orientation = self.quaternion_to_euler(q)

        else:
            # Simple gyro integration for orientation
            self.hand_orientation[0] += gyro_corrected["x"] * dt * self.orientation_scale  # Roll
            self.hand_orientation[1] += gyro_corrected["y"] * dt * self.orientation_scale  # Pitch
            self.hand_orientation[2] += gyro_corrected["z"] * dt * self.orientation_scale  # Yaw

        # Estimate position from accelerometer (simplified approach)
        # Remove gravity and integrate to get velocity, then position

        # For now, use a simplified approach: map gyro rates to position changes
        # This gives relative motion control rather than absolute positioning

        # Map Joy-Con rotations to hand position changes
        position_delta = np.array([
            -gyro_corrected["y"] * self.position_scale,  # Forward/back from pitch
            gyro_corrected["z"] * self.position_scale,  # Left/right from yaw
            gyro_corrected["x"] * self.position_scale  # Up/down from roll
        ])

        # Apply velocity damping to prevent runaway
        self.hand_velocity = self.hand_velocity * 0.9 + position_delta
        self.hand_position += self.hand_velocity * dt

        # Keep position within reasonable bounds
        self.hand_position = np.clip(self.hand_position, [0.1, -0.3, 0.05], [0.6, 0.3, 0.4])

    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        # Simplified conversion - replace with proper library function if needed
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

    def calculate_inverse_kinematics(self):
        """Calculate joint angles using inverse kinematics."""
        if not self.use_ik or not self.robot_chain:
            return None

        try:
            # Create target pose combining position and orientation
            target_position = self.hand_position.tolist()

            # For now, only use position IK (orientation IK is more complex)
            joint_angles = self.robot_chain.inverse_kinematics(target_position)

            # Remove the fixed joints (first and last are usually fixed in IKPy)
            if len(joint_angles) > 6:
                joint_angles = joint_angles[1:-1]  # Remove first and last

            return joint_angles[:6]  # Ensure we have exactly 6 joints

        except Exception as e:
            print(f"❌ IK calculation failed: {e}")
            return None

    def fallback_joint_mapping(self, gyro):
        """Fallback direct joint mapping when IK is not available."""
        # Your original mapping approach, but improved
        joint_angles = np.zeros(6)

        # Map gyro data to joints with better scaling
        joint_angles[0] = gyro["z"] * 0.001  # Base rotation from yaw
        joint_angles[1] = gyro["y"] * 0.001  # Shoulder from pitch
        joint_angles[3] = gyro["x"] * 0.001  # Elbow from roll

        return joint_angles

    def send_keyboard_command(self, key: str, joint_id: int, direction: str, movement: float):
        """Send keyboard command via the configured output method."""
        timestamp = time.strftime("%H:%M:%S")
        command_info = f"[{timestamp}] Joint {joint_id} ({self.joint_keys[joint_id]['name']}): {direction} -> '{key.upper()}' ({movement:.3f})"

        if self.output_mode == "ssh":
            try:
                ssh_command = f"echo '{key}'"
                subprocess.run([
                    "ssh", f"{self.ssh_user}@{self.ssh_host}", ssh_command
                ], capture_output=True, timeout=1)
                print(f"📡 SSH: {command_info}")
            except Exception as e:
                print(f"❌ SSH send failed: {e}")

        elif self.output_mode == "file":
            try:
                with open(self.output_file, 'a') as f:
                    f.write(f"{command_info}\n")
                    f.flush()
                print(f"📁 FILE: {command_info}")
            except Exception as e:
                print(f"❌ File write failed: {e}")

        else:  # console mode
            print(f"🤖 CONSOLE: {command_info}")

    def run(self):
        """Main control loop."""
        if not self.right_joycon:
            print("❌ No Joy-Con connected")
            return

        print("\n🎮 Joy-Con IK Controller Started")
        print("=" * 50)

        if self.use_ik:
            print("🧠 Using Inverse Kinematics mode")
        else:
            print("⚙️  Using Direct Joint Mapping mode")

        print("📋 Instructions:")
        print("- Hold ZL and move Joy-Con to control robot")
        print("- Press + to recalibrate IMU")
        print("- Press - to quit")
        print("=" * 50)

        # Initial calibration
        if not self.is_calibrated:
            self.calibrate_imu()

        last_output_time = 0
        output_interval = 0.05  # 20 Hz output rate

        try:
            while self.running:
                # Get Joy-Con status
                status = self.right_joycon.get_status()

                if not status:
                    time.sleep(0.01)
                    continue

                # Check for button presses
                buttons = status.get('buttons', {})

                if buttons.get('plus'):
                    print("🔄 Recalibrating IMU...")
                    self.calibrate_imu()
                    time.sleep(0.5)  # Debounce

                if buttons.get('minus'):
                    print("👋 Exiting...")
                    break

                # Process IMU data when ZL is held
                zl_pressed = buttons.get('zl', False)

                if zl_pressed and 'gyro' in status and 'accel' in status:
                    gyro = status['gyro']
                    accel = status['accel']

                    # Process IMU data to update hand position
                    self.process_imu_data(gyro, accel)

                    # Control output rate
                    current_time = time.time()
                    if current_time - last_output_time >= output_interval:

                        if self.use_ik:
                            # Use inverse kinematics
                            joint_angles = self.calculate_inverse_kinematics()

                            if joint_angles is not None:
                                if self.udp_robot:
                                    # Send directly to robot via UDP
                                    self.send_robot_position(joint_angles)
                                else:
                                    # Convert to keyboard commands (limited usefulness)
                                    print(f"🎯 Target Position: {self.hand_position}")
                                    print(f"🔧 Joint Angles: {[f'{x:.3f}' for x in joint_angles]}")

                        else:
                            # Use fallback direct mapping
                            joint_angles = self.fallback_joint_mapping(gyro)

                            if self.udp_robot:
                                self.send_robot_position(joint_angles)
                            else:
                                # Send keyboard commands for significant movements
                                for i, angle in enumerate(joint_angles[:6]):
                                    if abs(angle) > 0.005:  # Threshold for keyboard output
                                        joint_id = i + 1
                                        if angle > 0:
                                            key = self.joint_keys[joint_id]["up"]
                                            direction = "UP"
                                        else:
                                            key = self.joint_keys[joint_id]["down"]
                                            direction = "DOWN"

                                        self.send_keyboard_command(key, joint_id, direction, angle)

                        last_output_time = current_time

                time.sleep(0.01)  # 100 Hz loop

        except KeyboardInterrupt:
            print("\n👋 Interrupted by user")
        finally:
            if self.right_joycon:
                self.right_joycon.disconnect()
            if self.robot_sock:
                self.robot_sock.close()
            print("✅ Cleanup complete")


def main():
    """Main function with command line arguments."""
    parser = argparse.ArgumentParser(description="Joy-Con Robot Controller with Inverse Kinematics")
    parser.add_argument("--ssh-host", help="SSH hostname or IP address")
    parser.add_argument("--ssh-user", help="SSH username")
    parser.add_argument("--ssh-port", type=int, default=22, help="SSH port (default: 22)")
    parser.add_argument("--output-file", help="Output commands to file")
    parser.add_argument("--mode", choices=["console", "ssh", "file"], default="console", help="Output mode")
    parser.add_argument("--udp-robot", help="Robot IP address for direct UDP control")
    parser.add_argument("--urdf-file", default="so101_new_calib.urdf", help="URDF file for robot model")

    args = parser.parse_args()

    # Determine output mode
    if args.udp_robot:
        output_mode = "udp"
    elif args.mode == "ssh" or (args.ssh_host and args.ssh_user):
        if not args.ssh_host or not args.ssh_user:
            print("❌ SSH mode requires --ssh-host and --ssh-user")
            return
        output_mode = "ssh"
    elif args.mode == "file" or args.output_file:
        if not args.output_file:
            args.output_file = "robot_commands.txt"
        output_mode = "file"
    else:
        output_mode = "console"

    print("🤖 Joy-Con IK Robot Controller")
    print("=" * 50)

    controller = JoyConIKController(
        ssh_host=args.ssh_host,
        ssh_user=args.ssh_user,
        output_file=args.output_file,
        output_mode=output_mode,
        udp_robot=args.udp_robot,
        urdf_file=args.urdf_file
    )
    controller.run()


if __name__ == "__main__":
    main()
