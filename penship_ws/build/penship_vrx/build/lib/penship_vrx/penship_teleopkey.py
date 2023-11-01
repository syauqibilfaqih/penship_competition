#libraries and dependencies

import math
# import numpy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion, quaternion_from_euler 
import keyboard

class TeleOpKey(Node):
    thrustRight=0.0
    thrustLeft=0.0
    posRight=0.0
    posLeft=0.0
    kp_lin    = 10000
    kp_ang    = 10000
    xOrientationWAMV = 0.0
    yOrientationWAMV = 0.0
    zOrientationWAMV = 0.0
    wOrientationWAMV = 0.0
    latitudeWAMV = 0.0
    longitudeWAMV = 0.0
    xGoalPos = 0.0
    yGoalPos = 0.0
    xGoalOri = 0.0
    yGoalOri = 0.0
    zGoalOri = 0.0
    wGoalOri = 0.0
    velocityWAMV = 1000.0

    def __init__(self):
        super().__init__('teleop_key')
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

    def teleopKey(self):
        if keyboard.is_pressed('w'):
            self.thrustRight = 1000.0
            self.thrustLeft = 1000.0
        if keyboard.is_pressed('right'):
            self.thrustRight = -1000.0
            self.thrustLeft = 1000.0
        if keyboard.is_pressed('left'):
            self.thrustRight = 1000.0
            self.thrustLeft = -1000.0
        if keyboard.is_pressed('down'):
            self.thrustRight = -1000.0
            self.thrustLeft = -1000.0

def main(args=None):
    rclpy.init(args=None)
    teleop_key = TeleOpKey()
    # station_keeping.thrustLeft=1000.0
    # station_keeping.thrustRight=-1000.0
    # rclpy.spin(station_keeping)
    rclpy.spin(teleop_key)
    #MotionPublisher(1000.0,1000.0,0.0,0.0)

if __name__ == '__main__':
    main()