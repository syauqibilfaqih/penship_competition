#libraries and dependencies

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from ros_gz_interfaces.msg import ParamVec
from rcl_interfaces.msg import Parameter
from tf_transformations import euler_from_quaternion, quaternion_from_euler 
import time

class AccousticTracking(Node):
    thrustRight=0.0
    thrustLeft=0.0
    posRight=0.0
    posLeft=0.0
    kp_lin    = 10000
    kp_ang    = 10000
    velocityWAMV = 1000.0
    elevationDeg = 0
    bearingDeg =0
    rangeDist = 0


    def __init__(self):
        super().__init__('accoustic_tracking')
        self.subscriptionBeacon = self.create_subscription(ParamVec,'/wamv/sensors/acoustics/receiver/range_bearing', self.listener_callbackBeacon,10)
        self.subscriptionBeacon
        self.thrustRightPublisher = self.create_publisher(Float64,'/wamv/thrusters/right/thrust',10)
        self.thrustLeftPublisher = self.create_publisher(Float64,'/wamv/thrusters/left/thrust',10)
        self.posRightPublisher = self.create_publisher(Float64,'/wamv/thrusters/right/pos',10)
        self.posLeftPublisher = self.create_publisher(Float64,'/wamv/thrusters/left/pos',10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float64()
        msg.data = self.thrustRight
        self.thrustRightPublisher.publish(msg)
        msg.data = self.thrustLeft
        self.thrustLeftPublisher.publish(msg)
        msg.data = self.posRight
        self.posRightPublisher.publish(msg)
        msg.data = self.posLeft
        self.posLeftPublisher.publish(msg)        

    def accoustictracking(self):
        if abs(self.bearingDeg)>0.2:
            if self.bearingDeg>0:
                self.thrustLeft=-200.0
                self.thrustRight=200.0
            else:
                self.thrustLeft=200.0
                self.thrustRight=-200.0
        else:
            if self.rangeDist > 0.2:
                self.thrustLeft=1500.0
                self.thrustRight=1500.0
            else:
                self.thrustLeft=0.0
                self.thrustRight=0.0
                exit(1)

    def listener_callbackBeacon(self, msg):
        message=msg
        i=0
        while i<3:
            if message.params[i].name == 'elevation':
                self.elevationDeg = msg.params[i].value.double_value
            if message.params[i].name == 'bearing':
                self.bearingDeg = msg.params[i].value.double_value
            if message.params[i].name == 'range':
                self.rangeDist = msg.params[i].value.double_value
            i=i+1
        self.get_logger().info('Elevation, Bearing, and Range : ' + str(self.elevationDeg) + ', ' + str(self.bearingDeg)+ ', ' + str(self.rangeDist))
        self.accoustictracking()

def main(args=None):
    rclpy.init(args=None)
    accoustic_tracking = AccousticTracking()
    rclpy.spin(accoustic_tracking)

if __name__ == '__main__':
    main()


# topic list

# /clock [rosgraph_msgs/msg/Clock]
# /parameter_events [rcl_interfaces/msg/ParameterEvent]
# /pinger/set_pinger_position [geometry_msgs/msg/Vector3]
# /rosout [rcl_interfaces/msg/Log]
# /tf [tf2_msgs/msg/TFMessage]
# /tf_static [tf2_msgs/msg/TFMessage]
# /vrx/contacts [ros_gz_interfaces/msg/Contacts]
# /vrx/debug/wind/direction [std_msgs/msg/Float32]
# /vrx/debug/wind/speed [std_msgs/msg/Float32]
# /vrx/scan_dock_deliver/color_sequence [ros_gz_interfaces/msg/StringVec]
# /vrx/task/info [ros_gz_interfaces/msg/ParamVec]
# /wamv/joint_states [sensor_msgs/msg/JointState]
# /wamv/pose [tf2_msgs/msg/TFMessage]
# /wamv/pose_static [tf2_msgs/msg/TFMessage]
# /wamv/robot_description [std_msgs/msg/String]
# /wamv/sensors/acoustics/receiver/range_bearing [ros_gz_interfaces/msg/ParamVec]
# /wamv/sensors/cameras/front_left_camera_sensor/camera_info [sensor_msgs/msg/CameraInfo]
# /wamv/sensors/cameras/front_left_camera_sensor/image_raw [sensor_msgs/msg/Image]
# /wamv/sensors/cameras/front_left_camera_sensor/optical/camera_info [sensor_msgs/msg/CameraInfo]
# /wamv/sensors/cameras/front_left_camera_sensor/optical/image_raw [sensor_msgs/msg/Image]
# /wamv/sensors/cameras/front_right_camera_sensor/camera_info [sensor_msgs/msg/CameraInfo]
# /wamv/sensors/cameras/front_right_camera_sensor/image_raw [sensor_msgs/msg/Image]
# /wamv/sensors/cameras/front_right_camera_sensor/optical/camera_info [sensor_msgs/msg/CameraInfo]
# /wamv/sensors/cameras/front_right_camera_sensor/optical/image_raw [sensor_msgs/msg/Image]
# /wamv/sensors/cameras/middle_right_camera_sensor/camera_info [sensor_msgs/msg/CameraInfo]
# /wamv/sensors/cameras/middle_right_camera_sensor/image_raw [sensor_msgs/msg/Image]
# /wamv/sensors/cameras/middle_right_camera_sensor/optical/camera_info [sensor_msgs/msg/CameraInfo]
# /wamv/sensors/cameras/middle_right_camera_sensor/optical/image_raw [sensor_msgs/msg/Image]
# /wamv/sensors/gps/gps/fix [sensor_msgs/msg/NavSatFix]
# /wamv/sensors/imu/imu/data [sensor_msgs/msg/Imu]
# /wamv/sensors/lidars/lidar_wamv_sensor/points [sensor_msgs/msg/PointCloud2]
# /wamv/sensors/lidars/lidar_wamv_sensor/scan [sensor_msgs/msg/LaserScan]
# /wamv/shooters/ball_shooter/fire [std_msgs/msg/Bool]
# /wamv/thrusters/left/pos [std_msgs/msg/Float64]
# /wamv/thrusters/left/thrust [std_msgs/msg/Float64]
# /wamv/thrusters/right/pos [std_msgs/msg/Float64]
# /wamv/thrusters/right/thrust [std_msgs/msg/Float64]

# header:
#   stamp:
#     sec: 9
#     nanosec: 0
#   frame_id: pinger
# params:
# - name: bearing
#   value:
#     type: 3
#     bool_value: false
#     integer_value: 0
#     double_value: 0.9905356737224978
#     string_value: ''
#     byte_array_value: []
#     bool_array_value: []
#     integer_array_value: []
#     double_array_value: []
#     string_array_value: []
# - name: range
#   value:
#     type: 3
#     bool_value: false
#     integer_value: 0
#     double_value: 27.03748105004062
#     string_value: ''
#     byte_array_value: []
#     bool_array_value: []
#     integer_array_value: []
#     double_array_value: []
#     string_array_value: []
# - name: elevation
#   value:
#     type: 3
#     bool_value: false
#     integer_value: 0
#     double_value: -0.10681549138677467
#     string_value: ''
#     byte_array_value: []
#     bool_array_value: []
#     integer_array_value: []
#     double_array_value: []
#     string_array_value: []