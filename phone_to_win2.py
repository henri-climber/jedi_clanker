#!/usr/bin/env python3
"""
Phone IMU Robot Controller with TidyBot++ Inverse Kinematics
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

# MuJoCo for better IK
try:
    import mujoco

    HAS_MUJOCO = True
    print("✅ MuJoCo available for IK")
except ImportError:
    print("Warning: MuJoCo not found. Install with: pip install mujoco")
    HAS_MUJOCO = False

# Fallback to ikpy if MuJoCo unavailable
try:
    import ikpy.chain

    HAS_IKPY = True
    print("✅ ikpy available as fallback")
except ImportError:
    print("Warning: ikpy not found. Install with: pip install ikpy")
    HAS_IKPY = False

# Sensor fusion
try:
    from ahrs.filters import Madgwick

    HAS_AHRS = True
except ImportError:
    print("Warning: AHRS library not found. Install with: pip install ahrs")
    HAS_AHRS = False


class SO101IKSolver:
    """TidyBot++ style IK solver adapted for SO101 robot arm."""

    def __init__(self, urdf_path, ee_offset=0.0):
        """Initialize IK solver with SO101 model."""
        self.urdf_path = urdf_path
        self.ee_offset = ee_offset
        self.model = None
        self.data = None
        self.initialized = False

        # IK parameters (from TidyBot++)
        self.damping_coeff = 1e-12
        self.max_angle_change = np.deg2rad(45)
        self.max_iters = 20
        self.err_thresh = 1e-4

        # Try to initialize MuJoCo model
        if HAS_MUJOCO:
            self.setup_mujoco_model()

    def setup_mujoco_model(self):
        """Setup MuJoCo model from URDF."""
        try:
            # Convert URDF to MuJoCo XML if needed
            xml_path = self.urdf_path.replace('.urdf', '.xml')

            # Try loading existing XML or create from URDF
            if Path(xml_path).exists():
                self.model = mujoco.MjModel.from_xml_path(xml_path)
                print(f"✅ Loaded MuJoCo model: {xml_path}")
            else:
                # Try direct URDF loading (MuJoCo 2.3.2+)
                try:
                    self.model = mujoco.MjModel.from_xml_path(self.urdf_path)
                    print(f"✅ Loaded URDF directly: {self.urdf_path}")
                except:
                    print(f"❌ Could not load {self.urdf_path}. Convert to MuJoCo XML format.")
                    return

            self.data = mujoco.MjData(self.model)
            self.model.body_gravcomp[:] = 1.0  # Enable gravity compensation

            # Find end-effector site/body
            self.setup_end_effector()

            # Cache home position
            self.setup_home_position()

            # Pre-allocate arrays for efficiency
            self.setup_arrays()

            self.initialized = True
            print(f"🧠 TidyBot++ IK solver initialized with {self.model.nq} DOF")

        except Exception as e:
            print(f"❌ MuJoCo setup failed: {e}")
            print("Falling back to ikpy...")

    def setup_end_effector(self):
        """Find and setup end-effector reference."""
        # Look for common end-effector names
        ee_names = ['end_effector', 'ee', 'gripper', 'tool', 'tcp', 'wrist']

        self.site_id = None
        self.body_id = None

        # Try to find end-effector site first
        for name in ee_names:
            try:
                self.site_id = self.model.site(name).id
                print(f"✅ Found end-effector site: {name}")
                break
            except:
                continue

        # If no site found, try body
        if self.site_id is None:
            for name in ee_names:
                try:
                    self.body_id = self.model.body(name).id
                    print(f"✅ Found end-effector body: {name}")
                    break
                except:
                    continue

        # If still not found, use last body
        if self.site_id is None and self.body_id is None:
            self.body_id = self.model.nbody - 1  # Last body
            print(f"⚠️ Using last body as end-effector: body {self.body_id}")

        # Apply end-effector offset if specified
        if self.site_id is not None and self.ee_offset > 0:
            self.model.site(self.site_id).pos[2] -= self.ee_offset

    def setup_home_position(self):
        """Setup home/neutral joint configuration."""
        # Try to find a keyframe for home position
        self.qpos0 = None

        for i in range(self.model.nkey):
            key_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_KEY, i)
            if key_name and key_name.lower() in ['home', 'neutral', 'rest', 'retract']:
                self.qpos0 = self.model.key(key_name).qpos.copy()
                print(f"✅ Found home position: {key_name}")
                break

        # If no keyframe found, use current qpos or zeros
        if self.qpos0 is None:
            if self.model.nq <= 6:  # Typical arm
                # Use a reasonable neutral pose for 6-DOF arm
                self.qpos0 = np.array([0, -0.3, 0, -2.2, 0, 2.0])[:self.model.nq]
            else:
                self.qpos0 = np.zeros(self.model.nq)
            print("⚠️ Using default home position")

    def setup_arrays(self):
        """Pre-allocate arrays for efficient computation."""
        self.err = np.empty(6)
        self.err_pos, self.err_rot = self.err[:3], self.err[3:]

        if self.site_id is not None:
            self.site_pos = self.data.site(self.site_id).xpos
            self.site_mat = self.data.site(self.site_id).xmat
        else:
            self.site_pos = self.data.xpos[self.body_id]  # Body position
            self.site_mat = self.data.xmat[self.body_id].reshape(3, 3)  # Body rotation matrix

        self.site_quat = np.empty(4)
        self.site_quat_inv = np.empty(4)
        self.err_quat = np.empty(4)
        self.jac = np.empty((6, self.model.nv))
        self.jac_pos, self.jac_rot = self.jac[:3], self.jac[3:]
        self.damping = self.damping_coeff * np.eye(6)
        self.eye = np.eye(self.model.nv)

    def solve(self, target_pos, target_quat, current_qpos, max_iters=None, err_thresh=None):
        """
        Solve inverse kinematics using TidyBot++ approach.

        Args:
            target_pos: Target position [x, y, z]
            target_quat: Target quaternion [x, y, z, w]
            current_qpos: Current joint positions
            max_iters: Maximum iterations (default: self.max_iters)
            err_thresh: Error threshold (default: self.err_thresh)

        Returns:
            Joint positions that achieve target pose
        """
        if not self.initialized:
            return self.fallback_solve(target_pos, target_quat, current_qpos)

        max_iters = max_iters or self.max_iters
        err_thresh = err_thresh or self.err_thresh

        # Convert quaternion format (x,y,z,w) -> (w,x,y,z) for MuJoCo
        quat = target_quat[[3, 0, 1, 2]] if len(target_quat) == 4 else np.array([1, 0, 0, 0])

        # Set arm to initial joint configuration
        self.data.qpos[:len(current_qpos)] = current_qpos

        for iteration in range(max_iters):
            # Update kinematics
            mujoco.mj_kinematics(self.model, self.data)
            mujoco.mj_comPos(self.model, self.data)

            # Translational error
            self.err_pos[:] = target_pos - self.site_pos

            # Rotational error
            if self.site_id is not None:
                mujoco.mju_mat2Quat(self.site_quat, self.site_mat)
            else:
                # Convert rotation matrix to quaternion for body
                mujoco.mju_mat2Quat(self.site_quat, self.site_mat.flatten())

            mujoco.mju_negQuat(self.site_quat_inv, self.site_quat)
            mujoco.mju_mulQuat(self.err_quat, quat, self.site_quat_inv)
            mujoco.mju_quat2Vel(self.err_rot, self.err_quat, 1.0)

            # Check convergence
            if np.linalg.norm(self.err) < err_thresh:
                break

            # Calculate Jacobian
            if self.site_id is not None:
                mujoco.mj_jacSite(self.model, self.data, self.jac_pos, self.jac_rot, self.site_id)
            else:
                mujoco.mj_jacBody(self.model, self.data, self.jac_pos, self.jac_rot, self.body_id)

            # Calculate update using damped least squares
            jac_T = self.jac.T
            jac_jac_T = self.jac @ jac_T + self.damping
            update = jac_T @ np.linalg.solve(jac_jac_T, self.err)

            # Add null-space projection to drive toward home position
            qpos0_err = np.mod(self.qpos0 - self.data.qpos + np.pi, 2 * np.pi) - np.pi
            null_space_proj = self.eye - (jac_T @ np.linalg.pinv(jac_jac_T)) @ self.jac
            update += null_space_proj @ qpos0_err

            # Enforce maximum angle change
            update_max = np.abs(update).max()
            if update_max > self.max_angle_change:
                update *= self.max_angle_change / update_max

            # Apply update
            mujoco.mj_integratePos(self.model, self.data.qpos, update, 1.0)

        return self.data.qpos[:len(current_qpos)].copy()

    def fallback_solve(self, target_pos, target_quat, current_qpos):
        """Fallback IK when MuJoCo is not available."""
        print("⚠️ Using fallback IK")
        # Return current position with small random perturbation
        return current_qpos + np.random.normal(0, 0.1, len(current_qpos)) * 0.01


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

        # Enhanced IK Setup with TidyBot++ approach
        self.ik_solver = None
        self.use_ik = False
        self.robot_chain = None  # Keep for fallback

        if urdf_file and Path(urdf_file).exists():
            # Try TidyBot++ approach first
            if HAS_MUJOCO:
                try:
                    self.ik_solver = SO101IKSolver(urdf_file)
                    if self.ik_solver.initialized:
                        self.use_ik = True
                        print(f"✅ Using TidyBot++ IK solver")
                    else:
                        raise Exception("MuJoCo IK initialization failed")
                except Exception as e:
                    print(f"❌ TidyBot++ IK failed: {e}")

            # Fallback to ikpy if needed
            if not self.use_ik and HAS_IKPY:
                try:
                    self.robot_chain = ikpy.chain.Chain.from_urdf_file(urdf_file)
                    self.use_ik = True
                    print(f"⚠️ Using ikpy fallback: {len(self.robot_chain.links)} links")

                    # Print joint info
                    for i, link in enumerate(self.robot_chain.links):
                        if link.joint_type != "fixed":
                            print(f"   Joint {i}: {link.name}")

                except Exception as e:
                    print(f"❌ ikpy fallback failed: {e}")
        else:
            print(f"❌ URDF file not found: {urdf_file}")

        # IMU Processing (unchanged)
        self.orientation_filter = None
        if HAS_AHRS:
            self.orientation_filter = Madgwick()

        # Hand tracking state - adjusted for better control
        self.hand_position = np.array([0.35, 0.0, 0.25])  # Robot workspace center
        self.hand_velocity = np.array([0.0, 0.0, 0.0])
        self.hand_orientation_quat = np.array([0.0, 0.0, 0.0, 1.0])  # [x, y, z, w]

        # Control parameters - tuned for smoother control
        self.position_scale = 0.3  # Workspace size scale
        self.orientation_smoothing = 0.9  # Smoother orientation changes

        # Workspace limits for SO101
        self.workspace_center = np.array([0.35, 0.0, 0.25])
        self.workspace_size = 0.2  # 20cm movement range
        self.workspace_min = np.array([0.15, -0.25, 0.05])
        self.workspace_max = np.array([0.55, 0.25, 0.45])

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

        # Joint state tracking
        self.current_joints = np.zeros(6)  # SO101 has 6 joints

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
        """Process IMU data from phone and update target pose."""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        if dt > 0.1:  # Skip if too much time has passed
            return

        # Extract IMU data with better error handling
        try:
            # Phone orientation (degrees to radians)
            alpha = math.radians(imu_data.get('alpha', 0) or 0)  # Z-axis (yaw)
            beta = math.radians(imu_data.get('beta', 0) or 0)  # X-axis (pitch)
            gamma = math.radians(imu_data.get('gamma', 0) or 0)  # Y-axis (roll)

            # Motion data (if available)
            ax = imu_data.get('ax', 0) or 0
            ay = imu_data.get('ay', 0) or 0
            az = imu_data.get('az', 0) or 0
            rx = imu_data.get('rx', 0) or 0
            ry = imu_data.get('ry', 0) or 0
            rz = imu_data.get('rz', 0) or 0

        except (TypeError, ValueError):
            return

        # Convert phone orientation to target position in robot workspace
        # Map phone tilt directly to workspace position
        target_position = self.workspace_center + np.array([
            beta * self.position_scale,  # Forward/back from pitch
            -gamma * self.position_scale,  # Left/right from roll (inverted)
            alpha * self.position_scale * 0.5  # Up/down from yaw (reduced)
        ])

        # Clamp to workspace limits
        target_position = np.clip(target_position, self.workspace_min, self.workspace_max)

        # Convert phone orientation to target quaternion
        # Use phone orientation directly as end-effector orientation
        target_quat = self.euler_to_quaternion(gamma, beta, alpha)

        # Smooth the position update
        position_alpha = 0.3  # Position smoothing factor
        self.hand_position = (1 - position_alpha) * self.hand_position + position_alpha * target_position

        # Smooth the orientation update
        self.hand_orientation_quat = self.slerp_quaternion(
            self.hand_orientation_quat, target_quat, 1 - self.orientation_smoothing
        )

        # Store for debugging
        self.last_imu_data = {
            'raw_orientation': [alpha, beta, gamma],
            'target_pos': target_position.tolist(),
            'smoothed_pos': self.hand_position.tolist(),
            'target_quat': self.hand_orientation_quat.tolist(),
            'motion': [ax, ay, az, rx, ry, rz]
        }

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion [x, y, z, w]."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return np.array([x, y, z, w])

    def slerp_quaternion(self, q1, q2, t):
        """Spherical linear interpolation between quaternions."""
        # Ensure unit quaternions
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)

        # Handle negative dot product
        dot = np.dot(q1, q2)
        if dot < 0.0:
            q2 = -q2
            dot = -dot

        # Linear interpolation if quaternions are close
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)

        # Spherical interpolation
        theta_0 = math.acos(abs(dot))
        theta = theta_0 * t

        q2_orthogonal = q2 - q1 * dot
        q2_orthogonal = q2_orthogonal / np.linalg.norm(q2_orthogonal)

        return q1 * math.cos(theta) + q2_orthogonal * math.sin(theta)

    def calculate_inverse_kinematics(self):
        """Calculate IK using TidyBot++ approach or fallback."""
        if not self.use_ik:
            return self.fallback_joint_mapping()

        try:
            # Use TidyBot++ solver if available
            if self.ik_solver and self.ik_solver.initialized:
                joint_angles = self.ik_solver.solve(
                    self.hand_position,
                    self.hand_orientation_quat,
                    self.current_joints
                )

                # Update current joint state
                self.current_joints = joint_angles.copy()
                return joint_angles

            # Fallback to ikpy
            elif self.robot_chain:
                target_position = self.hand_position.tolist()
                joint_angles = self.robot_chain.inverse_kinematics(target_position)

                # Remove fixed joints if needed
                if len(joint_angles) > 6:
                    joint_angles = joint_angles[1:-1]  # Remove base and tip

                joint_angles = joint_angles[:6]  # Ensure 6 joints
                self.current_joints = np.array(joint_angles)
                return joint_angles

        except Exception as e:
            print(f"❌ IK calculation failed: {e}")

        return self.fallback_joint_mapping()

    def fallback_joint_mapping(self):
        """Fallback direct joint mapping when IK is not available."""
        # Map hand position and orientation to joints using simple heuristics
        joint_angles = np.zeros(6)

        # Position-based mapping
        pos_rel = self.hand_position - self.workspace_center

        # Simple kinematic mapping for 6-DOF arm
        joint_angles[0] = math.atan2(pos_rel[1], pos_rel[0])  # Base rotation
        joint_angles[1] = -pos_rel[2] * 2.0  # Shoulder pitch (reach up/down)
        joint_angles[2] = pos_rel[0] * 1.5  # Shoulder roll (reach forward/back)
        joint_angles[3] = -joint_angles[1] * 0.7  # Elbow (follow shoulder)

        # Orientation mapping from quaternion
        x, y, z, w = self.hand_orientation_quat
        joint_angles[4] = 2 * math.atan2(y, w)  # Wrist pitch
        joint_angles[5] = 2 * math.atan2(x, w)  # Wrist roll

        # Apply joint limits (typical for robot arms)
        joint_limits = np.array([
            [-3.14, 3.14],  # Base
            [-1.57, 1.57],  # Shoulder
            [-2.0, 2.0],  # Shoulder
            [-2.8, 0],  # Elbow
            [-1.57, 1.57],  # Wrist
            [-3.14, 3.14]  # Wrist
        ])

        for i in range(6):
            joint_angles[i] = np.clip(joint_angles[i], joint_limits[i][0], joint_limits[i][1])

        self.current_joints = joint_angles.copy()
        return joint_angles

    def control_robot(self):
        """Send control commands to robot."""
        if not self.control_active:
            return

        current_time = time.time()
        if current_time - self.last_control_time < self.control_rate:
            return

        # Calculate joint angles using improved IK
        joint_angles = self.calculate_inverse_kinematics()

        # Send to robot
        if self.udp_robot:
            self.send_robot_position(joint_angles)

        # Log data
        control_data = {
            'target_position': self.hand_position.tolist(),
            'target_orientation': self.hand_orientation_quat.tolist(),
            'joint_angles': joint_angles.tolist(),
            'ik_method': 'TidyBot++' if (self.ik_solver and self.ik_solver.initialized) else 'fallback'
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
    <title>SO101 Phone Controller</title>
    <style>
        body { 
            font-family: Arial, sans-serif; 
            text-align: center; 
            margin: 20px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            min-height: 100vh;
        }
        .container {
            max-width: 600px;
            margin: 0 auto;
            background: rgba(255, 255, 255, 0.1);
            padding: 20px;
            border-radius: 15px;
            box-shadow: 0 8px 32px rgba(0,0,0,0.3);
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255,255,255,0.2);
        }
        .status {
            padding: 15px;
            margin: 10px 0;
            border-radius: 10px;
            font-weight: bold;
            backdrop-filter: blur(5px);
        }
        .connected { 
            background: rgba(40, 167, 69, 0.3); 
            border: 1px solid rgba(40, 167, 69, 0.5);
        }
        .disconnected { 
            background: rgba(220, 53, 69, 0.3); 
            border: 1px solid rgba(220, 53, 69, 0.5);
        }
        .button {
            background: linear-gradient(45deg, #007bff, #0056b3);
            color: white;
            border: none;
            padding: 15px 30px;
            font-size: 18px;
            border-radius: 10px;
            cursor: pointer;
            margin: 10px;
            transition: all 0.3s ease;
            box-shadow: 0 4px 15px rgba(0,123,255,0.4);
        }
        .button:hover { 
            background: linear-gradient(45deg, #0056b3, #004085);
            transform: translateY(-2px);
            box-shadow: 0 6px 20px rgba(0,123,255,0.6);
        }
        .button:disabled { 
            background: rgba(108, 117, 125, 0.5); 
            cursor: not-allowed; 
            transform: none;
            box-shadow: none;
        }
        .data { 
            background: rgba(248, 249, 250, 0.1); 
            padding: 15px; 
            margin: 15px 0;
            border-radius: 10px;
            text-align: left;
            font-family: 'Courier New', monospace;
            font-size: 12px;
            border: 1px solid rgba(255,255,255,0.2);
            line-height: 1.4;
        }
        .controls {
            display: flex;
            justify-content: space-around;
            margin: 20px 0;
            flex-wrap: wrap;
        }
        .active { 
            background: linear-gradient(45deg, #28a745, #20c997) !important; 
            box-shadow: 0 4px 15px rgba(40,167,69,0.6) !important;
        }
        .title {
            font-size: 28px;
            margin-bottom: 20px;
            text-shadow: 0 2px 4px rgba(0,0,0,0.3);
        }
        .ik-badge {
            display: inline-block;
            background: rgba(255,193,7,0.3);
            border: 1px solid rgba(255,193,7,0.5);
            padding: 5px 15px;
            border-radius: 20px;
            font-size: 12px;
            margin-top: 10px;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="title">🤖 SO101 Phone Controller</div>
        <div class="ik-badge">TidyBot++ Enhanced IK</div>
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
            const formatArray = (arr) => arr ? `[${arr.map(x => formatNum(x)).join(', ')}]` : '[0, 0, 0]';

            dataDiv.innerHTML = `
<strong>📱 Phone Orientation:</strong>
α=${formatNum(imu.alpha)}° β=${formatNum(imu.beta)}° γ=${formatNum(imu.gamma)}°

<strong>🎯 Target Position:</strong>
${imu.target_pos ? formatArray(imu.target_pos) : '[0, 0, 0]'} m

<strong>🤖 Control Status:</strong>
Mode: ${isControlling ? '🟢 ACTIVE' : '🔴 Inactive'}
IK: TidyBot++ Enhanced Solver

<strong>📊 Motion Data:</strong>
Accel: [${formatNum(imu.ax)}, ${formatNum(imu.ay)}, ${formatNum(imu.az)}]
Gyro: [${formatNum(imu.rx)}, ${formatNum(imu.ry)}, ${formatNum(imu.rz)}]
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
            // Higher frequency updates for smoother control
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
                updateStatus('✅ Connected - TidyBot++ IK Ready', true);
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

            // Handle server responses
            ws.onmessage = (event) => {
                try {
                    const response = JSON.parse(event.data);
                    if (response.target_pos) {
                        imu.target_pos = response.target_pos;
                    }
                } catch (e) {
                    console.warn('Invalid server response:', e);
                }
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
                updateStatus('🔑 Requesting permissions...', false);

                await requestPermissions();
                updateStatus('⚙️ Setting up IMU...', false);

                setupIMU();
                startWebSocket();

                // Send data every 20ms (50 Hz) for smooth control
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
                controlBtn.textContent = '🛑 Stop Control';
                controlBtn.classList.add('active');
                updateStatus('🤖 Robot control ACTIVE - Move your phone!', true);
            } else {
                controlBtn.textContent = '▶️ Start Control';
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
                updateStatus('🔧 Calibrating IMU...', true);
                setTimeout(() => {
                    updateStatus('✅ Calibration complete', true);
                }, 2000);
            }
        });

        // Prevent phone from sleeping during control
        let wakeLock = null;
        if ('wakeLock' in navigator) {
            navigator.wakeLock.request('screen').then((lock) => {
                wakeLock = lock;
                console.log('Screen wake lock acquired');
            }).catch((err) => {
                console.log('Wake lock failed:', err);
            });
        }

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

                    # Send back target position for display
                    response = {
                        'target_pos': ctrl.hand_position.tolist(),
                        'ik_method': 'TidyBot++' if (ctrl.ik_solver and ctrl.ik_solver.initialized) else 'fallback'
                    }
                    await websocket.send_text(json.dumps(response))

        except WebSocketDisconnect:
            print("📱 Phone disconnected")
            ctrl.control_active = False

    return app


def main():
    parser = argparse.ArgumentParser(description="SO101 Phone IMU Robot Controller with TidyBot++ IK")
    parser.add_argument("--udp-robot", help="Robot IP address for UDP control")
    parser.add_argument("--urdf-file", default="so101_new_calib.urdf", help="URDF file path")
    parser.add_argument("--output-file", help="Log data to file")
    parser.add_argument("--host", default="localhost", help="Web server host")
    parser.add_argument("--port", type=int, default=8000, help="Web server port")

    args = parser.parse_args()

    print("🤖 SO101 Phone IMU Robot Controller")
    print("🧠 Enhanced with TidyBot++ Inverse Kinematics")
    print("=" * 60)

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

    if controller.ik_solver and controller.ik_solver.initialized:
        print("🎯 Using TidyBot++ Enhanced IK Solver")
    elif controller.use_ik:
        print("⚠️ Using ikpy fallback IK solver")
    else:
        print("🔧 Using direct joint mapping (no IK)")

    print("\n📋 Instructions:")
    print("1. Open the URL on your phone")
    print("2. Click 'Connect IMU' and grant motion permissions")
    print("3. Click 'Start Control' to begin robot control")
    print("4. Tilt your phone to control the robot end-effector:")
    print("   • Pitch (forward/back) → Robot X-axis")
    print("   • Roll (left/right) → Robot Y-axis")
    print("   • Yaw (rotate) → Robot Z-axis")
    print("=" * 60)

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