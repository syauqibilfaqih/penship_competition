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
import time

class StationKeeping(Node):
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
        super().__init__('station_keeping')
        self.subscriptionGoalPos = self.create_subscription(PoseStamped, '/vrx/stationkeeping/goal', self.listener_callbackGoalPos, 10)
        self.subscriptionGoalPos
        self.subscriptionGPS = self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.listener_callbackGPS, 10)
        self.subscriptionGPS
        self.subscriptionIMU = self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.listener_callbackIMU, 10)
        self.subscriptionIMU
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
    
    def proportionalControl(self, angle):
        error_linear  = math.sqrt( pow((self.xGoalPos - self.latitudeWAMV),2) + pow((self.yGoalPos-self.longitudeWAMV),2) ) 
        #Ref_ang = math.atan2(self.yGoalPos-self.longitudeWAMV, self.xGoalPos - self.latitudeWAMV)
        q0_ref   = self.xGoalOri 
        q1_ref   = self.yGoalOri 
        q2_ref   = self.zGoalOri 
        q3_ref   = self.wGoalOri 
        (roll_ref, pitch_ref, yaw_ref)    =  (euler_from_quaternion([q0_ref,q1_ref,q2_ref,q3_ref]))
        Ref_ang = yaw_ref
        linear_input = self.kp_lin*error_linear
        angular_input = self.kp_ang*(Ref_ang - angle)
        return linear_input, angular_input, error_linear

    # def stationKeeping(self):
    #     dt        = 0.01 # sample time: 10 mili sec. 
    #     b         = 201.4  # distance between two wheels
    #     r         = 21.3  # radius of two wheel

    #     q0_imu   = self.xOrientationWAMV
    #     q1_imu   = self.yOrientationWAMV
    #     q2_imu   = self.zOrientationWAMV
    #     q3_imu   = self.wOrientationWAMV

    #     (roll_imu, pitch_imu, yaw_imu)    =  (euler_from_quaternion([q0_imu,q1_imu,q2_imu,q3_imu]))
    #     yaw = yaw_imu
    #     # Go to goal control
    #     lin_cmd, ang_cmd, e = self.proportionalControl(yaw)
    #     vel_lin=0.0
    #     vel_ang=0.0
    #     if lin_cmd < 0.02:
    #         vel_lin=0
    #         vel_ang=ang_cmd
    #     else:
    #         vel_lin=lin_cmd
    #         vel_ang=0
    #     # self.thrustRight = float(vel_lin*1000+vel_ang*100)
    #     # self.thrustLeft = float(vel_lin*1000-vel_ang*100)
    #     # self.thrustRight = float((vel_lin*1000)+(vel_ang))
    #     # self.thrustLeft = float((vel_lin*1000)-(vel_ang))
    #     self.thrustRight = float((vel_lin*500))#+(vel_ang/1000))
    #     self.thrustLeft = float((vel_lin*500))#-(vel_ang/1000))
    #     self.get_logger().info('Linear and Angular Velocities of WAMV : ' + str(lin_cmd) + ' and ' + str(ang_cmd))

    def stationKeeping(self):

        desiredTheta = math.atan2(self.yGoalPos-self.longitudeWAMV,self.xGoalPos - self.latitudeWAMV)

        q0_imu   = self.xOrientationWAMV
        q1_imu   = self.yOrientationWAMV
        q2_imu   = self.zOrientationWAMV
        q3_imu   = self.wOrientationWAMV

        (roll_imu, pitch_imu, yaw_imu)    =  (euler_from_quaternion([q0_imu,q1_imu,q2_imu,q3_imu]))
        yaw = yaw_imu
        
        error_linear  = math.sqrt( pow((self.xGoalPos - self.latitudeWAMV),2) + pow((self.yGoalPos-self.longitudeWAMV),2) )

        error_linear = error_linear*pow(10,4)
        self.velocityWAMV = 3000*error_linear

        # if self.velocityWAMV > 1000:
        #     self.velocityWAMV = 1000
        # else:
        #     self.velocityWAMV = self.velocityWAMV
        # self.velocityWAMV = 500.0
        omega = (desiredTheta - yaw_imu)
        vX = self.velocityWAMV*math.cos(omega)
        vY = self.velocityWAMV*math.sin(omega)

        omega = omega*(1000)

        vLeft=vX+vY-omega
        vRight=vX+vY+omega
        
        if math.fabs(self.xGoalPos*pow(10,4) - self.latitudeWAMV*pow(10,4))<0.01 and math.fabs(self.yGoalPos*pow(10,4)-self.longitudeWAMV*pow(10,4))<0.01:
            q0_ref   = self.xGoalOri 
            q1_ref   = self.yGoalOri 
            q2_ref   = self.zGoalOri 
            q3_ref   = self.wGoalOri 
            (roll_ref, pitch_ref, yaw_ref)    =  (euler_from_quaternion([q0_ref,q1_ref,q2_ref,q3_ref]))
            Ref_ang = yaw_ref
            
            rotateVel = yaw_ref-yaw_imu

            self.thrustRight = 0.0#rotateVel*1000
            self.thrustLeft = 0.0#-rotateVel*1000
        else:
            self.thrustRight = float(vRight)
            self.thrustLeft = float(vLeft)

        #self.get_logger().info('Error X and Y : ' + str(self.xGoalPos - self.latitudeWAMV) + ' and ' + str(self.yGoalPos-self.longitudeWAMV))
        self.get_logger().info('Error Linear : ' + str(error_linear))
        # self.get_logger().info('Nilai IMU : ' + str(yaw_imu))
        # self.get_logger().info('Nilai Theta : ' + str(desiredTheta))
        
        


    def listener_callbackGPS(self, msg):
        self.latitudeWAMV = msg.latitude
        self.longitudeWAMV = msg.longitude
        self.stationKeeping()
        #self.get_logger().info('Latitude and Longitude : ' + str(self.latitudeWAMV) + ', ' + str(self.longitudeWAMV))
    
    def listener_callbackIMU(self, msg):
        self.xOrientationWAMV = msg.orientation.x
        self.yOrientationWAMV = msg.orientation.y
        self.zOrientationWAMV = msg.orientation.z
        self.wOrientationWAMV = msg.orientation.w
        self.stationKeeping()
        #self.get_logger().info('Z orientation of WAMV : ' + str(self.zOrientationWAMV))
    
    def listener_callbackGoalPos(self, msg):
        self.xGoalPos = msg.pose.position.x
        self.yGoalPos = msg.pose.position.y
        self.xGoalOri = msg.pose.orientation.x
        self.yGoalOri = msg.pose.orientation.y
        self.zGoalOri = msg.pose.orientation.z
        self.wGoalOri = msg.pose.orientation.w
        self.stationKeeping()
        #self.get_logger().info('X and Y Goal Position : ' + str(self.xGoalPos) + ', ' + str(self.yGoalPos))

def main(args=None):
    rclpy.init(args=None)
    station_keeping = StationKeeping()
    # station_keeping.thrustLeft=1000.0
    # station_keeping.thrustRight=-1000.0
    # rclpy.spin(station_keeping)
    rclpy.spin(station_keeping)
    #MotionPublisher(1000.0,1000.0,0.0,0.0)

if __name__ == '__main__':
    main()