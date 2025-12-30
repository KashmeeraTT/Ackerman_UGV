#!/usr/bin/env python3
"""
Sensor Fusion GUI Status Display

Graphical window showing side-by-side comparison of:
- Left: What is COMMANDED (cmd_vel, steering intent)
- Right: What is PERCEIVED (IMU, VSLAM, steering feedback)

Red highlighting when values differ significantly.

Run: ros2 run sensor_fusion status_gui.py
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import math
import threading

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:
    print("Error: tkinter not found. Install with: sudo apt install python3-tk")
    exit(1)


class SensorData:
    """Thread-safe sensor data storage"""
    def __init__(self):
        self.lock = threading.Lock()
        
        # Commanded
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        
        # Perceived
        self.vslam_vx = 0.0
        self.vslam_x = 0.0
        self.vslam_y = 0.0
        self.vslam_yaw = 0.0
        
        self.steering_feedback = 0.0
        
        self.camera_imu_accel = 0.0
        self.camera_imu_gyro = 0.0
        
        self.body_imu_accel = 0.0
        self.body_imu_gyro = 0.0
        
        self.health_score = 100.0


class ROSNode(Node):
    """ROS2 node for subscribing to topics"""
    def __init__(self, data: SensorData):
        super().__init__('sensor_fusion_gui')
        self.data = data
        
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Odometry, '/odom', self.vslam_cb, 10)
        self.create_subscription(Float32, '/ugv/steering_angle', self.steering_cb, 10)
        self.create_subscription(Imu, '/camera/gyro_accel/sample', self.camera_imu_cb, 10)
        self.create_subscription(Imu, '/ugv/imu', self.body_imu_cb, 10)
        self.create_subscription(Float32, '/sensor_fusion/health_score', self.health_cb, 10)

    def cmd_vel_cb(self, msg):
        with self.data.lock:
            self.data.cmd_linear = msg.linear.x
            self.data.cmd_angular = msg.angular.z

    def vslam_cb(self, msg):
        with self.data.lock:
            self.data.vslam_x = msg.pose.pose.position.x
            self.data.vslam_y = msg.pose.pose.position.y
            self.data.vslam_vx = msg.twist.twist.linear.x
            q = msg.pose.pose.orientation
            self.data.vslam_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def steering_cb(self, msg):
        with self.data.lock:
            self.data.steering_feedback = msg.data

    def camera_imu_cb(self, msg):
        with self.data.lock:
            self.data.camera_imu_accel = math.sqrt(
                msg.linear_acceleration.x**2 +
                msg.linear_acceleration.y**2 +
                msg.linear_acceleration.z**2
            )
            self.data.camera_imu_gyro = math.sqrt(
                msg.angular_velocity.x**2 +
                msg.angular_velocity.y**2 +
                msg.angular_velocity.z**2
            )

    def body_imu_cb(self, msg):
        with self.data.lock:
            self.data.body_imu_accel = math.sqrt(
                msg.linear_acceleration.x**2 +
                msg.linear_acceleration.y**2 +
                msg.linear_acceleration.z**2
            )
            self.data.body_imu_gyro = math.sqrt(
                msg.angular_velocity.x**2 +
                msg.angular_velocity.y**2 +
                msg.angular_velocity.z**2
            )

    def health_cb(self, msg):
        with self.data.lock:
            self.data.health_score = msg.data


class StatusGUI:
    """Tkinter GUI for sensor status display"""
    
    def __init__(self, data: SensorData):
        self.data = data
        self.root = tk.Tk()
        self.root.title("Sensor Fusion Status")
        self.root.geometry("800x600")
        self.root.configure(bg='#1e1e1e')
        
        # Colors
        self.bg_color = '#1e1e1e'
        self.fg_color = '#ffffff'
        self.accent_color = '#3498db'
        self.good_color = '#2ecc71'
        self.warn_color = '#f39c12'
        self.bad_color = '#e74c3c'
        self.panel_bg = '#2d2d2d'
        
        self.setup_ui()
        
    def setup_ui(self):
        # Title
        title = tk.Label(self.root, text="SENSOR FUSION STATUS", 
                        font=('Helvetica', 18, 'bold'),
                        bg=self.bg_color, fg=self.accent_color)
        title.pack(pady=10)
        
        # Main container
        main_frame = tk.Frame(self.root, bg=self.bg_color)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        # Left panel - Commanded
        left_frame = tk.LabelFrame(main_frame, text=" COMMANDED ", 
                                   font=('Helvetica', 12, 'bold'),
                                   bg=self.panel_bg, fg=self.accent_color,
                                   padx=15, pady=10)
        left_frame.grid(row=0, column=0, sticky='nsew', padx=5)
        
        # Right panel - Perceived
        right_frame = tk.LabelFrame(main_frame, text=" PERCEIVED ", 
                                    font=('Helvetica', 12, 'bold'),
                                    bg=self.panel_bg, fg=self.accent_color,
                                    padx=15, pady=10)
        right_frame.grid(row=0, column=1, sticky='nsew', padx=5)
        
        main_frame.grid_columnconfigure(0, weight=1)
        main_frame.grid_columnconfigure(1, weight=1)
        main_frame.grid_rowconfigure(0, weight=1)
        
        # Create labels for commanded values
        self.cmd_labels = {}
        self.create_row(left_frame, 0, "Linear Velocity:", "cmd_linear", "m/s")
        self.create_row(left_frame, 1, "Angular Command:", "cmd_angular", "rad/s")
        self.create_row(left_frame, 2, "Steering Intent:", "steering_cmd", "")
        
        # Create labels for perceived values
        self.perc_labels = {}
        self.create_row(right_frame, 0, "VSLAM Velocity:", "vslam_vx", "m/s", is_perceived=True)
        self.create_row(right_frame, 1, "VSLAM Yaw:", "vslam_yaw", "°", is_perceived=True)
        self.create_row(right_frame, 2, "Steering Angle:", "steering_fb", "°", is_perceived=True)
        
        # Separator
        sep = ttk.Separator(right_frame, orient='horizontal')
        sep.grid(row=3, column=0, columnspan=2, sticky='ew', pady=10)
        
        self.create_row(right_frame, 4, "Camera IMU Accel:", "cam_accel", "m/s²", is_perceived=True)
        self.create_row(right_frame, 5, "Camera IMU Gyro:", "cam_gyro", "rad/s", is_perceived=True)
        self.create_row(right_frame, 6, "Body IMU Accel:", "body_accel", "m/s²", is_perceived=True)
        self.create_row(right_frame, 7, "Body IMU Gyro:", "body_gyro", "rad/s", is_perceived=True)
        
        # Position display
        pos_frame = tk.Frame(self.root, bg=self.bg_color)
        pos_frame.pack(fill=tk.X, padx=20, pady=5)
        
        self.pos_label = tk.Label(pos_frame, text="Position: (0.00, 0.00) m",
                                  font=('Helvetica', 11),
                                  bg=self.bg_color, fg=self.fg_color)
        self.pos_label.pack()
        
        # Health bar
        health_frame = tk.Frame(self.root, bg=self.bg_color)
        health_frame.pack(fill=tk.X, padx=20, pady=10)
        
        tk.Label(health_frame, text="System Health:", 
                font=('Helvetica', 11, 'bold'),
                bg=self.bg_color, fg=self.fg_color).pack(side=tk.LEFT)
        
        self.health_bar = ttk.Progressbar(health_frame, length=400, mode='determinate')
        self.health_bar.pack(side=tk.LEFT, padx=10)
        
        self.health_label = tk.Label(health_frame, text="100.0%",
                                     font=('Helvetica', 11),
                                     bg=self.bg_color, fg=self.good_color)
        self.health_label.pack(side=tk.LEFT)
        
        # Status indicator
        self.status_frame = tk.Frame(self.root, bg=self.bg_color)
        self.status_frame.pack(fill=tk.X, padx=20, pady=10)
        
        self.status_indicator = tk.Label(self.status_frame, text="● CONSISTENT",
                                         font=('Helvetica', 14, 'bold'),
                                         bg=self.bg_color, fg=self.good_color)
        self.status_indicator.pack()
        
    def create_row(self, parent, row, label_text, key, unit, is_perceived=False):
        """Create a label-value row"""
        label = tk.Label(parent, text=label_text,
                        font=('Helvetica', 10),
                        bg=self.panel_bg, fg='#aaaaaa',
                        anchor='w')
        label.grid(row=row, column=0, sticky='w', pady=3)
        
        value = tk.Label(parent, text="0.000 " + unit,
                        font=('Helvetica', 11, 'bold'),
                        bg=self.panel_bg, fg=self.fg_color,
                        anchor='e', width=15)
        value.grid(row=row, column=1, sticky='e', pady=3)
        
        if is_perceived:
            self.perc_labels[key] = value
        else:
            self.cmd_labels[key] = value
            
    def update_display(self):
        """Update all display values"""
        with self.data.lock:
            # Get current values
            cmd_linear = self.data.cmd_linear
            cmd_angular = self.data.cmd_angular
            vslam_vx = self.data.vslam_vx
            vslam_yaw = math.degrees(self.data.vslam_yaw)
            steering = self.data.steering_feedback
            cam_accel = self.data.camera_imu_accel
            cam_gyro = self.data.camera_imu_gyro
            body_accel = self.data.body_imu_accel
            body_gyro = self.data.body_imu_gyro
            health = self.data.health_score
            pos_x = self.data.vslam_x
            pos_y = self.data.vslam_y
        
        # Calculate expected steering angle from angular command
        # Using Ackermann: steering_angle ≈ atan(wheelbase * angular / linear)
        wheelbase = 0.6
        if abs(cmd_linear) > 0.01:
            expected_steering = math.degrees(math.atan(wheelbase * cmd_angular / cmd_linear))
        else:
            expected_steering = 0.0
        
        # Check for mismatches
        vel_mismatch = abs(cmd_linear - vslam_vx) > 0.1
        steer_mismatch = abs(expected_steering - steering) > 3.0  # 3 degree tolerance
        
        # Stationary IMU check: if commanded stationary but IMU shows motion
        is_stationary = abs(cmd_linear) < 0.02
        accel_deviation = abs(cam_accel - 9.81)  # Subtract gravity
        imu_mismatch = is_stationary and accel_deviation > 0.5
        
        # Update commanded labels
        self.cmd_labels['cmd_linear'].config(
            text=f"{cmd_linear:+.3f} m/s",
            fg=self.bad_color if vel_mismatch else self.fg_color
        )
        self.cmd_labels['cmd_angular'].config(text=f"{cmd_angular:+.3f} rad/s")
        steering_intent = "YES" if abs(cmd_angular) > 0.05 else "NO"
        self.cmd_labels['steering_cmd'].config(
            text=steering_intent,
            fg=self.warn_color if steering_intent == "YES" else self.fg_color
        )
        
        # Update perceived labels
        self.perc_labels['vslam_vx'].config(
            text=f"{vslam_vx:+.3f} m/s",
            fg=self.bad_color if vel_mismatch else self.fg_color
        )
        self.perc_labels['vslam_yaw'].config(text=f"{vslam_yaw:+.1f}°")
        self.perc_labels['steering_fb'].config(
            text=f"{steering:+.2f}°",
            fg=self.bad_color if steer_mismatch else self.fg_color
        )
        self.perc_labels['cam_accel'].config(
            text=f"{cam_accel:.2f} m/s²",
            fg=self.bad_color if imu_mismatch else self.fg_color
        )
        self.perc_labels['cam_gyro'].config(text=f"{cam_gyro:.3f} rad/s")
        self.perc_labels['body_accel'].config(
            text=f"{body_accel:.2f} m/s²",
            fg=self.bad_color if imu_mismatch else self.fg_color
        )
        self.perc_labels['body_gyro'].config(text=f"{body_gyro:.3f} rad/s")
        
        # Update position
        self.pos_label.config(text=f"Position: ({pos_x:+.2f}, {pos_y:+.2f}) m")
        
        # Update health bar
        self.health_bar['value'] = health
        if health > 70:
            health_color = self.good_color
        elif health > 40:
            health_color = self.warn_color
        else:
            health_color = self.bad_color
        self.health_label.config(text=f"{health:.1f}%", fg=health_color)
        
        # Update status indicator
        any_mismatch = vel_mismatch or steer_mismatch or imu_mismatch
        if any_mismatch:
            self.status_indicator.config(text="● MISMATCH DETECTED", fg=self.bad_color)
        else:
            self.status_indicator.config(text="● CONSISTENT", fg=self.good_color)
        
        # Schedule next update
        self.root.after(100, self.update_display)
        
    def run(self):
        """Start the GUI main loop"""
        self.update_display()
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    
    # Shared data
    data = SensorData()
    
    # Create ROS node
    node = ROSNode(data)
    
    # Create GUI
    gui = StatusGUI(data)
    
    # Run ROS in separate thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()
    
    # Run GUI in main thread
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
