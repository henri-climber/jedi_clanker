#!/usr/bin/env python3
"""
Enhanced Joy-Con Robot Controller with SSH Output
Run with: python3 joycon_ssh_controller.py --ssh-host HOSTNAME --ssh-user USERNAME
Or use: python3 joycon_ssh_controller.py --output-file robot_commands.txt
"""

import sys
import time
import math
import subprocess
import argparse
import matplotlib.pyplot as plt
from pyjoycon import JoyCon, get_R_id


class JoyConSSHController:
    def __init__(self, ssh_host=None, ssh_user=None, output_file=None, output_mode="console"):
        """Initialize with SSH or file output capabilities."""
        self.right_joycon = None
        self.running = True
        
        # SSH Configuration
        self.ssh_host = ssh_host
        self.ssh_user = ssh_user  
        self.ssh_port = 22 # Default SSH port
        self.output_file = output_file
        self.output_mode = output_mode  # "console", "ssh", "file", "pipe"
        
        # Joint to keyboard mapping for SO-101 ARM
        self.joint_keys = {
            1: {"up": "q", "down": "a", "name": "Shoulder"},
            2: {"up": "w", "down": "s", "name": "Elbow"}, 
            3: {"up": "e", "down": "d", "name": "Wrist"},
            4: {"up": "r", "down": "f", "name": "Wrist_Rotate"},
            5: {"up": "t", "down": "g", "name": "Gripper"},
            6: {"up": "y", "down": "h", "name": "Claw"}
        }
        
        # Control parameters
        self.sensitivity = 2.0
        self.threshold = 10
        self.zero_reference = {}
        self.is_zeroed = False
        self.last_output_time = 0
        self.output_delay = 0.1
        
        # ZL hold control
        self.zl_pressed = False
        self.zl_initial_position = {}
        self.joint_active = {}
        
        # Current joint states
        self.joint_positions = {}
        for joint_id in self.joint_keys:
            self.joint_positions[joint_id] = 0
        
        # Setup output method
        self.setup_output()
        self.connect_joycons()
    
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
    
    def send_keyboard_command(self, key: str, joint_id: int, direction: str, movement: float):
        """Send keyboard command via the configured output method."""
        timestamp = time.strftime("%H:%M:%S")
        command_info = f"[{timestamp}] Joint {joint_id} ({self.joint_keys[joint_id]['name']}): {direction} -> '{key.upper()}' ({movement:.3f})"
        
        if self.output_mode == "ssh":
            try:
                # Send key press command via SSH
                ssh_command = f"echo '{key}' > /dev/stdin"  # You may need to modify this based on your robot control software
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
        
        elif self.output_mode == "pipe":
            try:
                with open("/tmp/joycon_output", 'w') as f:
                    f.write(f"{key}\n")
                    f.flush()
                print(f"🔄 PIPE: {command_info}")
            except Exception as e:
                print(f"❌ Pipe write failed: {e}")
        
        else:  # console mode
            print(f"🤖 CONSOLE: {command_info}")

    def output_keyboard_command(self, joint_id, movement):
        """Enhanced output method with SSH/file support."""
        if abs(movement) < self.threshold * 0.001:
            return
        
        joint_keys = self.joint_keys[joint_id]
        
        if movement > 0:
            key = joint_keys["up"]
            direction = "UP"
        else:
            key = joint_keys["down"] 
            direction = "DOWN"
        
        timestamp = time.strftime("%H:%M:%S")
        command_info = f"[{timestamp}] Joint {joint_id} ({joint_keys['name']}): {direction} -> '{key.upper()}' ({movement:.3f})"
        
        # Send command based on output mode
        if self.output_mode == "ssh":
            self.send_ssh_command(key, command_info)
        elif self.output_mode == "file":
            self.send_file_command(key, command_info)
        else:
            print(f"🤖 {command_info}")
    
    def send_ssh_command(self, key, command_info):
        """Send keyboard command via SSH."""
        try:
            # Option 1: Send to a specific program on remote host
            # Modify this command based on your robot control software
            ssh_cmd = f"ssh {self.ssh_user}@{self.ssh_host} 'echo \"{key}\"'"
            
            # Option 2: Send to a named pipe on remote host
            # ssh_cmd = f"ssh {self.ssh_user}@{self.ssh_host} 'echo \"{key}\" > /tmp/robot_commands'"
            
            # Option 3: Send to robot control program directly
            # ssh_cmd = f"ssh {self.ssh_user}@{self.ssh_host} 'echo \"{key}\" | your_robot_program'"
            
            subprocess.run(ssh_cmd, shell=True, timeout=1)
            print(f"📡 SSH: {command_info}")
            
        except Exception as e:
            print(f"❌ SSH send failed: {e}")
    
    def send_file_command(self, key, command_info):
        """Send keyboard command to file."""
        try:
            with open(self.output_file, 'a') as f:
                f.write(f"{key}\n")
                f.flush()
            print(f"📁 FILE: {command_info}")
        except Exception as e:
            print(f"❌ File write failed: {e}")

    def _get_stick_position(self, joycon):
        """Get stick position from the JoyCon object using get_stick_right_horizontal/vertical methods."""
        try:
            stick_x = joycon.get_stick_right_horizontal()
            stick_y = joycon.get_stick_right_vertical()
            # These methods may return None if the stick is not moved; default to 0
            stick_x = stick_x if stick_x is not None else 0
            stick_y = stick_y if stick_y is not None else 0
            return {"horizontal": stick_x, "vertical": stick_y}
        except Exception as e:
            print(f"[DEBUG] Could not get stick data using get_stick_right_horizontal/vertical: {e}")
            print("[DEBUG] JoyCon attributes:")
            print(dir(joycon))
            print("[DEBUG] JoyCon __dict__:")
            print(getattr(joycon, '__dict__', {}))
            raise AttributeError("Could not get stick data from JoyCon object. See debug output above.")

    def visualize_joycon(self):
        """Visualize Right Joy-Con stick and accelerometer (orientation) data in real time. Show gyro as text only."""
        if not self.right_joycon:
            print("❌ No Right Joy-Con connected for visualization. Please pair it via Bluetooth and try again.")
            return
        joycon = self.right_joycon
        print("\n🕹️  Right Joy-Con Visualization Mode")
        print("- Move the stick and the Joy-Con to see real-time data.")
        print("- Close the window or press Ctrl+C to exit.\n")
        plt.ion()
        fig = plt.figure(figsize=(12, 6))
        ax_stick = fig.add_subplot(1, 2, 1)
        ax_3d = fig.add_subplot(1, 2, 2, projection='3d')
        stick_scatter = ax_stick.scatter([], [], c='b')
        ax_stick.set_xlim(-1, 1)
        ax_stick.set_ylim(-1, 1)
        ax_stick.set_title('Stick Position')
        ax_stick.set_xlabel('X')
        ax_stick.set_ylabel('Y')
        ax_stick.grid(True)
        quiver_accel = None
        ax_3d.set_xlim(-5, 5)
        ax_3d.set_ylim(-5, 5)
        ax_3d.set_zlim(-5, 5)
        ax_3d.set_title('Accel (green, gravity direction)')
        ax_3d.set_xlabel('X')
        ax_3d.set_ylabel('Y')
        ax_3d.set_zlabel('Z')
        legend = ax_3d.legend(['Accel'], loc='upper left')
        text_box = fig.text(0.05, 0.92, '', fontsize=10, va='top', family='monospace')
        gyro_text = fig.text(0.55, 0.92, '', fontsize=10, va='top', family='monospace', color='red')
        while True:
            try:
                stick = self._get_stick_position(joycon)
                stick_x = stick.get("horizontal", 0) / 32767.0
                stick_y = stick.get("vertical", 0) / 32767.0
                accel = (
                    joycon.get_accel_x(),
                    joycon.get_accel_y(),
                    joycon.get_accel_z()
                )
                gyro = (
                    joycon.get_gyro_x(),
                    joycon.get_gyro_y(),
                    joycon.get_gyro_z()
                )
                stick_scatter.remove()
                stick_scatter = ax_stick.scatter([stick_x], [stick_y], c='b')
                ax_, ay, az = accel[0], accel[1], accel[2]
                if quiver_accel:
                    quiver_accel.remove()
                quiver_accel = ax_3d.quiver(0, 0, 0, ax_, ay, az, length=1, color='g', label='Accel')
                text_box.set_text(
                    f"Stick: x={stick_x:.2f}, y={stick_y:.2f}\n"
                    f"Accel (gravity): x={ax_:.2f}, y={ay:.2f}, z={az:.2f}"
                )
                gx, gy, gz = gyro[0], gyro[1], gyro[2]
                gyro_text.set_text(
                    f"Gyro (angular velocity):\n x={gx:.2f}, y={gy:.2f}, z={gz:.2f}"
                )
                plt.pause(0.01)
            except KeyboardInterrupt:
                print("\n👋 Visualization stopped by user.")
                break
            except Exception as e:
                print(f"Visualization error: {e}")
                break
        plt.ioff()
        plt.show()

    def connect_joycons(self):
        """Connect to the right Joy-Con only."""
        try:
            right_id = get_R_id()
            if right_id:
                self.right_joycon = JoyCon(*right_id)
                print("✅ Connected to Right Joy-Con")
            else:
                print("❌ Right Joy-Con not found. Please pair it via Bluetooth and try again.")
                self.right_joycon = None
        except Exception as e:
            print(f"❌ Error connecting to Right Joy-Con: {e}")
            self.right_joycon = None

    def run(self):
        print("No run() implementation yet. Exiting.")
        return

def main():
    """Main function with command line arguments."""
    parser = argparse.ArgumentParser(description="Joy-Con Robot Controller with SSH Output")
    parser.add_argument("--ssh-host", help="SSH hostname or IP address")
    parser.add_argument("--ssh-user", help="SSH username") 
    parser.add_argument("--ssh-port", type=int, default=22, help="SSH port (default: 22)")
    parser.add_argument("--output-file", help="Output commands to file")
    parser.add_argument("--mode", choices=["console", "ssh", "file"], default="console", help="Output mode")
    parser.add_argument("--visualize", action="store_true", help="Visualize Joy-Con position and angles")

    args = parser.parse_args()
    
    # Determine output mode
    if args.mode == "ssh" or (args.ssh_host and args.ssh_user):
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
    
    print("🤖 Joy-Con SSH Robot Controller")
    print("=" * 50)
    
    if args.visualize:
        controller = JoyConSSHController(
            ssh_host=args.ssh_host,
            ssh_user=args.ssh_user,
            output_file=args.output_file,
            output_mode=output_mode
        )
        controller.visualize_joycon()
        return

    controller = JoyConSSHController(
        ssh_host=args.ssh_host,
        ssh_user=args.ssh_user,
        output_file=args.output_file,
        output_mode=output_mode
    )
    controller.run()

if __name__ == "__main__":
    main()
