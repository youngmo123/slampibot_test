# !/usr/bin/env/ python3
#
# Copyright 2023 EduRobotAILab CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Leo Cho

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, Pose, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster
import math
import serial
import time

class OdomPose(object):
    x = 0.0
    y = 0.0
    theta = 0.0
    
class Encoder(object):
    previous_data = 0
    current_data = 0
    delta_data = 0
    distance = 0.0

class CmdVel():
    lin_vel_x = 0.0
    ang_vel_z = 0.0

class Joint(object):
    joint_name = ['wheel_left_joint', 'wheel_right_joint']
    joint_pos = [0.0, 0.0]
    joint_vel = [0.0, 0.0]

class BringUp(Node):
        
    def __init__(self):
        super().__init__('bring_up')    
        
        self.pico_serial = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=6.0)    
        self.pico_serial.reset_input_buffer() 
        self.pico_serial.reset_output_buffer() 
        
        # parameters
        self.gear_ratio = 30
        self.wheel_separation = 0.175 # [m] 
        self.wheel_radius = 0.0335 # [m]
        self.enc_pulse = 44
        self.pulse_per_meter = (self.enc_pulse * self.gear_ratio) / (2 * math.pi * self.wheel_radius) 
        self.delay = 0.007 
        
        self.max_lin_vel_x = 1.0
        self.max_ang_vel_z = 1.0
        self.timestamp_previous = self.get_clock().now()
        
        self.enc_r = Encoder()
        self.enc_l = Encoder()
        self.odom_pose = OdomPose()
        self.joint = Joint()        
        self.cmd_vel = CmdVel()      

        # Set subscriber
        qos_profile = QoSProfile(depth = 20) # queue_size 
        self.sub_CmdVelMsg = self.create_subscription(Twist, 'cmd_vel', self.cbCmdVelMsg, qos_profile)

        # Set publisher
        self.pub_JointStates = self.create_publisher(JointState,'joint_states', qos_profile)
        self.pub_Odom = self.create_publisher(Odometry, 'odom', qos_profile)
        self.pub_OdomTF = TransformBroadcaster(self)    
        self.pub_Pose = self.create_publisher(Pose, 'pose', qos_profile)

        # Set timer
        self.timerProc = self.create_timer(0.1, self.update_robot)
             

    def cbCmdVelMsg(self, cmd_vel_msg):     
        
        self.pico_serial = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=6.0)        
        self.pico_serial.reset_input_buffer() 
        self.pico_serial.reset_output_buffer() 
                
        self.cmd_vel.lin_vel_x = cmd_vel_msg.linear.x # [m/s] 
        self.cmd_vel.ang_vel_z = cmd_vel_msg.angular.z # [rad/s]
                
        deadzone = 0.42 # 0.4 
        if self.cmd_vel.lin_vel_x > 0:
            self.cmd_vel.lin_vel_x = deadzone + self.cmd_vel.lin_vel_x
        elif self.cmd_vel.lin_vel_x < 0:
            self.cmd_vel.lin_vel_x = -deadzone + self.cmd_vel.lin_vel_x
        if self.cmd_vel.lin_vel_x == 0 and self.cmd_vel.ang_vel_z > 0:
            self.cmd_vel.ang_vel_z = deadzone + self.cmd_vel.ang_vel_z
        elif self.cmd_vel.lin_vel_x == 0 and self.cmd_vel.ang_vel_z < 0:
            self.cmd_vel.ang_vel_z = -deadzone + self.cmd_vel.ang_vel_z
        
        self.cmd_vel.lin_vel_x = max(-self.max_lin_vel_x, min(self.max_lin_vel_x, self.cmd_vel.lin_vel_x))
        self.cmd_vel.ang_vel_z = max(-self.max_ang_vel_z, min(self.max_ang_vel_z, self.cmd_vel.ang_vel_z))
                        
        factor_vmr = 1.0  
        # factor_vmr = 1.10 # adjust to compensate motor wheel speed difference, for example, if right wheel is slower
        # factor_vmr = 0.90 # adjust to compensate motor wheel speed difference, for example, if right wheel is faster
        factor_vml = 1.0
        factor_z =1.0
        
        vmr = (self.cmd_vel.lin_vel_x + self.cmd_vel.ang_vel_z*factor_z)*factor_vmr
        vml = (self.cmd_vel.lin_vel_x - self.cmd_vel.ang_vel_z*factor_z)*factor_vml
                
        motor_rf = "motor_r.forward({0:1f})\r".format(vmr)
        motor_lf = "motor_l.forward({0:1f})\r".format(vml)
        motor_rb = "motor_r.backward({0:1f})\r".format(abs(vmr))
        motor_lb = "motor_l.backward({0:1f})\r".format(abs(vml))
        motor_rs = "motor_r.stop({0:1f})\r".format(abs(0.0))
        motor_ls = "motor_l.stop({0:1f})\r".format(abs(0.0))
        
                
        if vmr >= 0 and vml >= 0:
            self.pico_serial.write(motor_rf.encode("utf-8"))
            self.pico_serial.flush()
            time.sleep(self.delay)
            echo = self.pico_serial.readline().strip()
            self.pico_serial.write(motor_lf.encode("utf-8"))
            self.pico_serial.flush()
            time.sleep(self.delay)
            echo = self.pico_serial.readline().strip()
            
        elif vmr >= 0 and vml < 0:
            self.pico_serial.write(motor_rf.encode("utf-8"))
            self.pico_serial.flush()
            time.sleep(self.delay)
            echo = self.pico_serial.readline().strip()
            self.pico_serial.write(motor_lb.encode("utf-8"))
            self.pico_serial.flush()  
            time.sleep(self.delay)              
            echo = self.pico_serial.readline().strip()
            
        elif vmr < 0 and vml >= 0:
            self.pico_serial.write(motor_rb.encode("utf-8"))
            self.pico_serial.flush()
            time.sleep(self.delay)
            echo = self.pico_serial.readline().strip()
            self.pico_serial.write(motor_lf.encode("utf-8"))
            self.pico_serial.flush()   
            time.sleep(self.delay)             
            echo = self.pico_serial.readline().strip()
            
        elif vmr < 0 and vml < 0:
            self.pico_serial.write(motor_rb.encode("utf-8"))
            self.pico_serial.flush()
            time.sleep(self.delay)
            echo = self.pico_serial.readline().strip()
            self.pico_serial.write(motor_lb.encode("utf-8"))
            self.pico_serial.flush()  
            time.sleep(self.delay)              
            echo = self.pico_serial.readline().strip()           
        
                    
    def update_robot(self):
        
        self.pico_serial = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=6.0) 
        self.pico_serial.reset_input_buffer() 
        self.pico_serial.reset_output_buffer() 
               
        self.timestamp_now = self.get_clock().now()
        dt = (self.timestamp_now - self.timestamp_previous).nanoseconds * 1e-9
        self.timestamp_previous = self.timestamp_now      
           
        self.pico_serial.write("enc_r.position()\r".encode("utf-8"))           
        self.pico_serial.flush()
        time.sleep(self.delay)
        echo = self.pico_serial.readline().strip()            
        data_enc_r = self.pico_serial.readline().strip().decode("utf-8")  
        self.enc_r.current_data = round(float(data_enc_r))
        self.enc_r.delta_data = self.enc_r.current_data - self.enc_r.previous_data
        self.enc_r.previous_data = self.enc_r.current_data        
                
        self.pico_serial.write("enc_l.position()\r".encode("utf-8"))        
        self.pico_serial.flush()
        time.sleep(self.delay)
        echo = self.pico_serial.readline().strip()        
        data_enc_l = self.pico_serial.readline().strip().decode("utf-8")          
        self.enc_l.current_data = round(float(data_enc_l))
        self.enc_l.delta_data = self.enc_l.current_data - self.enc_l.previous_data
        self.enc_l.previous_data = self.enc_l.current_data        
                        
        self.pico_serial.write("mpu9250_rpy()\r".encode("utf-8"))
        self.pico_serial.flush()
        time.sleep(self.delay)
        echo = self.pico_serial.readline().strip()        
        data_mpu9250_rpy = self.pico_serial.readline().strip().decode("utf-8")            
        data_mpu9250_rpy_r = data_mpu9250_rpy.split(" ")[0] 
        data_mpu9250_rpy_p = data_mpu9250_rpy.split(" ")[1] 
        data_mpu9250_rpy_y = data_mpu9250_rpy.split(" ")[2]                     
        data_mpu9250_rpy = [float(data_mpu9250_rpy_r), float(data_mpu9250_rpy_p) , float(data_mpu9250_rpy_y)]
        
        self.pico_serial.write("mpu9250_gyro()\r".encode("utf-8"))
        self.pico_serial.flush()
        time.sleep(self.delay)
        echo = self.pico_serial.readline().strip()            
        data_mpu9250_gyro = self.pico_serial.readline().strip().decode("utf-8")            
        data_mpu9250_gyro = data_mpu9250_gyro.replace("(","").replace(")","")
        data_mpu9250_gyro_x = data_mpu9250_gyro.split(",")[0] 
        data_mpu9250_gyro_y = data_mpu9250_gyro.split(",")[1] 
        data_mpu9250_gyro_z = data_mpu9250_gyro.split(",")[2] 
        data_mpu9250_gyro = [float(data_mpu9250_gyro_x), float(data_mpu9250_gyro_y), float(data_mpu9250_gyro_z)]      
        
        self.enc_r.distance = self.enc_r.delta_data / self.pulse_per_meter
        self.enc_l.distance = self.enc_l.delta_data / self.pulse_per_meter
        distance = (self.enc_r.distance + self.enc_l.distance) / 2
        theta = (self.enc_r.distance - self.enc_l.distance) / self.wheel_separation
        trans_vel = distance / dt
        orient_vel = theta / dt
        
        odo_r = self.enc_r.current_data / self.pulse_per_meter # [m]
        odo_l = self.enc_l.current_data / self.pulse_per_meter # [m] (=r*theta, max 2*pi*r)     
                       
        vel_z = data_mpu9250_gyro[2]      # [degree/sec]
        roll_imu = data_mpu9250_rpy[0]    # [degree]
        pitch_imu = data_mpu9250_rpy[1]   # [degree]
        yaw_imu = data_mpu9250_rpy[2]     # [degree]

        self.update_odometry(trans_vel, orient_vel, dt)
        self.update_JointStates(odo_l, odo_r, trans_vel, orient_vel)
        self.update_PoseStates(roll_imu, pitch_imu, yaw_imu, vel_z)

    def update_odometry(self, trans_vel, orient_vel, dt):
        
        self.odom_pose.theta += orient_vel * dt # [rad]

        d_x = trans_vel * math.cos(self.odom_pose.theta) # [m/s]
        d_y = trans_vel * math.sin(self.odom_pose.theta) # [m/s]
        self.odom_pose.x += d_x * dt   # [m]
        self.odom_pose.y += d_y * dt   # [m]
        
        q = quaternion_from_euler(0, 0, self.odom_pose.theta) # roll, pitch, yaw -> quaternion
        
        # Set odometry data
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"        
        odom.header.stamp = self.timestamp_now.to_msg()
        odom.pose.pose.position.x = self.odom_pose.x
        odom.pose.pose.position.y = self.odom_pose.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = trans_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = orient_vel

        self.pub_Odom.publish(odom)

        # Set odomTF data
        odom_tf = TransformStamped()
        odom_tf.header.frame_id = odom.header.frame_id
        odom_tf.child_frame_id = odom.child_frame_id        
        odom_tf.header.stamp = self.timestamp_now.to_msg()
        
        odom_tf.transform.translation.x = odom.pose.pose.position.x
        odom_tf.transform.translation.y = odom.pose.pose.position.y
        odom_tf.transform.translation.z = odom.pose.pose.position.z
        odom_tf.transform.rotation = odom.pose.pose.orientation
        self.pub_OdomTF.sendTransform(odom_tf)

    def update_JointStates(self, odo_l, odo_r, trans_vel, orient_vel):
        
        wheel_ang_left = odo_l / self.wheel_radius   
        wheel_ang_right = odo_r / self.wheel_radius

        wheel_ang_vel_left = (trans_vel - (self.wheel_separation / 2.0) * orient_vel) / self.wheel_radius   
        wheel_ang_vel_right = (trans_vel + (self.wheel_separation / 2.0) * orient_vel) / self.wheel_radius  

        self.joint.joint_pos = [wheel_ang_left, wheel_ang_right]
        self.joint.joint_vel = [wheel_ang_vel_left, wheel_ang_vel_right]
        
        joint_states = JointState()
        joint_states.header.frame_id = "base_link"      
        joint_states.header.stamp = self.timestamp_now.to_msg()
        joint_states.name = self.joint.joint_name
        joint_states.position = self.joint.joint_pos
        joint_states.velocity = self.joint.joint_vel
        joint_states.effort = []
        self.pub_JointStates.publish(joint_states)
    
    def update_PoseStates(self, roll, pitch, yaw, vel_z):
        pose = Pose()
        pose.orientation.x = roll
        pose.orientation.y = pitch
        # pose.orientation.z = yaw
        pose.orientation.z = vel_z
        self.pub_Pose.publish(pose)
       
def quaternion_from_euler(roll, pitch, yaw):
    
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * sr - sy * sp * cr
    q[1] = sy * cp * sr + cy * sp * cr
    q[2] = sy * cp * cr - cy * sp * sr
    q[3] = cy * cp * cr + sy * sp * sr

    return q

            
def main(args=None):
    rclpy.init(args=args)

    bringup_node = BringUp()

    try:
        rclpy.spin(bringup_node)
    except KeyboardInterrupt:
        bringup_node.get_logger().info('keyboard Interrupt')
    finally:
        pico_serial = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=6.0)             
        pico_serial.reset_input_buffer() 
        pico_serial.reset_output_buffer() 
        pico_serial.write("motor_r.stop({0:1f})\r".format(abs(0.0)).encode("utf-8"))
        pico_serial.flush()
        time.sleep(0.01)    
        echo = pico_serial.readline().strip()
        pico_serial.write("motor_l.stop({0:1f})\r".format(abs(0.0)).encode("utf-8"))
        pico_serial.flush()
        time.sleep(0.01)     
        echo = pico_serial.readline().strip()
        pico_serial.close()        
        bringup_node.destroy_node()
        rclpy.shutdown()
       
if __name__ == '__main__':
    main()
