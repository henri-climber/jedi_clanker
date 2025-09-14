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
import os

# Add these imports to your existing script
try:
    from pyjoycon import JoyCon, get_R_id, get_L_id
except ImportError as e:
    print(f"❌ Error importing pyjoycon: {e}")
    print("Make sure you installed it with: pip install joycon-python hidapi pyglm")
    sys.exit(1)

class JoyConSSHController:
    def __init__(self, ssh_host=None, ssh_user=None, output_file=None, output_mode="console"):
        """Initialize with SSH or file output capabilities."""
        self.left_joycon = None
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

def main():
    """Main function with command line arguments."""
    parser = argparse.ArgumentParser(description="Joy-Con Robot Controller with SSH Output")
    parser.add_argument("--ssh-host", help="SSH hostname or IP address")
    parser.add_argument("--ssh-user", help="SSH username") 
    parser.add_argument("--ssh-port", type=int, default=22, help="SSH port (default: 22)")
    parser.add_argument("--output-file", help="Output commands to file")
    parser.add_argument("--mode", choices=["console", "ssh", "file"], default="console", help="Output mode")
    
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
    
    controller = JoyConSSHController(
        ssh_host=args.ssh_host,
        ssh_user=args.ssh_user,
        output_file=args.output_file,
        output_mode=output_mode
    )
    controller.run()

if __name__ == "__main__":
    main()
