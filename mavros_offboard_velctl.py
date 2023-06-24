from __future__ import division

PKG = 'px4'

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from mavros_msgs.msg import ParamValue, PositionTarget
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import AutoFlightAvoidance as afa
from time import localtime, time, ctime
from sensor_msgs.msg import LaserScan, PointCloud


class MavrosOffboardVelctlTest(MavrosTestCommon):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    def setUp(self):
        super(MavrosOffboardVelctlTest, self).setUp()

        self.pos = PoseStamped()
        self.vel = Twist()
        self.obstacle = []
        self.radius = 0.7
        self.vel_list = [0, 0, 0]
        self.avoid_vec = [0, 0]
        self.obs_list = []
        self.dcm = []

        self.pos_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        
        self.vel_setpoint_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)

        # send setpoints in separate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        #self.pos_thread.start()

        self.vel_thread = Thread(target=self.send_vel, args=())
        self.vel_thread.daemon = True
        self.vel_thread.start()

        sub = rospy.Subscriber('/laser/scan', LaserScan, self.RPlidar)

        self.obs_thread = Thread(target=self.lidar_sim, args=())
        self.obs_thread.daemon = True
        #self.obs_thread.start()


    def quat_callback(self):
        q0 = self.imu_data.orientation.x
        q1 = self.imu_data.orientation.y
        q2 = self.imu_data.orientation.z
        q3 = self.imu_data.orientation.w
        dcm=np.array([[q0**2+q1**2-q2**2-q3**2,2.0*(q1*q2-q0*q3),2.0*(q1*q3+q0*q2)],
                    [2.0*(q1*q2+q0*q3),q0**2+q2**2-q1**2-q3**2,2.0*(q2*q3+q0*q1)],
                    [2.0*(q1*q3-q0*q2),2.0*(q2*q3+q0*q1),q0**2+q3**2-q1**2-q2**2]])
        qq = (q0, q1, q2, q3)
        
        self.dcm = dcm
        t0 = +2.0 *(q3 * q0 + q1 * q2)
        t1 = +1.0 - 2.0*(q0 * q0+ q1 * q1)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (q3 * q1 - q2 * q0)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)  

        t3 = +2.0 * (q3 * q2 + q0 * q1)
        t4 = +1.0 - 2.0 * (q1 * q1 + q2* q2)
        yaw = math.atan2(t3, t4)

        roll = (roll*180)/math.pi
        self.roll = roll
        pitch = (pitch*180)/math.pi
        self.pitch = pitch
        yaw = (yaw*180)/math.pi 
        self.yaw = yaw

    def RPlidar(self, msg):
        
        obs = []
        self.quat_callback()
        obstacle = []
        obstacle_original=[]
        obs2 = []
        obs2_range = []
        a=[]
        b=[]
        range_data = msg.ranges
        n = len(range_data)
        theta = 2 * math.pi / n
        rate = rospy.Rate(12) #Hz


        for i in range(n):
            if (range_data[i] > afa.max_range) or (range_data[i] < afa.minimum_distance):
                continue
            
            [_, _, yaw] = euler_from_quaternion([self.local_position.pose.orientation.x,
                self.local_position.pose.orientation.y,
                self.local_position.pose.orientation.z,
                self.local_position.pose.orientation.w])
            
            obs.append(theta*i*180/math.pi - 180 + yaw)
            x = range_data[i] * math.cos(theta * i)
            y = range_data[i] * math.sin(theta * i)
            obstacle.append(self.dcm@np.array([x, y, 0]).T)
            obstacle_original.append(a)
            obs2.append(np.array([x,y,0]))
            b.append(range_data[i])
            
        self.obstacle = obstacle
        self.obs_list = obs

        #[rospy.loginfo("AFA | Obs {:.4f} {:.4f}".format(obs[0], obs[1])) for obs in obstacle]

        try: 
            rate.sleep()
        except rospy.ROSInterruptException:
            pass

    def lidar_sim(self):
        rate = rospy.Rate(12) #Hz

        while not rospy.is_shutdown():  
                    
            curr_pos = [self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z,]
            curr_pos = [round(pos, 4) for pos in curr_pos]

            self.obstacle = afa.lidar_sim(curr_pos, afa.sim_obstacle)
        
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
        

    def tearDown(self):
        super(MavrosOffboardVelctlTest, self).tearDown()

    #
    # Helper methods
    #


    def reach_position(self, x, y, z, timeout):
        # self.pos <- PoseStamped 할당
        # std_msgs/Header header
        # geometry_msgs/Pose pose -> geometry_msgs/Point position -> float64 x, y, z
        # 따라서 self.pos.Pose.position.x로 float 할당

        # float64 x, y, z

        """timeout(int): seconds"""
        # set a position setpoint -> 이러면 쓰레드의 send_pose가 알아서 보냄
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z

        # For demo purposes we will lock yaw/heading to north. -> 요 정보도 보내기
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(reached, (
            "took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
            format(self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z, timeout)))

    def send_vel(self):
        # self.vel <- Twist
        # geometry_msgs/Vector3 linear -> float64 x, y, z only direction
        # geometry_msgs/Vector3 angular
        # self.vel.linear.x

        rate = rospy.Rate(50) 

        while not rospy.is_shutdown():
            self.vel_setpoint_pub.publish(self.vel)


            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass


    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        rospy.loginfo("================POS PUB working ===================")

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)

            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset
    
    def position_control(self, x, y, z):
        # self.pos <- PoseStamped 할당
        # std_msgs/Header header
        # geometry_msgs/Pose pose -> geometry_msgs/Point position -> float64 x, y, z
        # 따라서 self.pos.Pose.position.x로 float 할당

        # float64 x, y, z

        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z

        [x0, y0, z0] = [self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z]

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0 #(math.atan2(y-y0, x-x0)*180/math.pi+90)%360
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)
        
    #
    # Test method
    #
    def test_velctl(self):
        """Test offboard position control"""

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        #self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)

        self.log_topic_vars()
        # exempting failsafe from lost RC to allow offboard
        rcl_except = ParamValue(1<<2, 0.0)
        self.set_param("COM_RCL_EXCEPT", rcl_except, 5)

        self.set_mode("AUTO.TAKEOFF", 5)
        self.set_arm(True, 5)

        rospy.sleep(7)

        self.set_mode("OFFBOARD", 5)
        
        rospy.loginfo("=========== Auto Flight Avoidance Activate! ===========")

        wp_list = [
            [-18, 0, 3],
            [0, 0, 3]
        ]
        idx = 0
        t0 = time()
        total_time_step = 0
        avg_time = 0
        loop_freq = 50  # Hz
        rate = rospy.Rate(loop_freq)

        curr_wp = wp_list[idx]

        #self.reach_position(0, 0, 3, 10)
        #self.pos_thread.join()
        #self.position_control(curr_wp[0], curr_wp[1], curr_wp[2])

        #with open('afa_sim_data {}.txt'.format(ctime(time())[8:]), 'w') as f:

        while idx < len(wp_list):
                t1 = time()

                curr_wp = wp_list[idx]
                curr_pos = [self.local_position.pose.position.x,
                            self.local_position.pose.position.y,
                            self.local_position.pose.position.z,]
                
                curr_pos = [round(pos, 4) for pos in curr_pos]

                curr_obstacle = self.obstacle #afa.lidar_sim(curr_pos, afa.sim_obstacle)
                self.avoid_vec, polar_density = afa.VFH(curr_obstacle)
                avoid_vec = self.avoid_vec

                

                #rospy.loginfo("\nAFA | curr_obstacle: {}".format(curr_obstacle))
                rospy.loginfo("AFA | curr_pos: {}".format(curr_pos))
                rospy.loginfo("AFA | obs: {}".format(self.obs_list))
                rospy.loginfo("AFA | yaw: {}".format(self.yaw*180/3.14))
                #rospy.loginfo("AFA | curr_wp: {}".format(curr_wp))

                if (avoid_vec[0] != 0) and (avoid_vec[1] != 0):
                    rospy.loginfo("AFA | 장애물 발견, 회피 동작: {:.4f}, {:.4f}".format(avoid_vec[0], avoid_vec[1]))
                    if (abs(self.roll)>=5 or abs(self.pitch)>=5):
                        rospy.loginfo(self.obstacle)
                        rospy.loginfo(self.roll)  
                        rospy.loginfo(self.pitch)  
                rospy.loginfo("AFA | 폴라 히스토그램: {}".format(polar_density))    
                #rate.sleep()

                self.vel_list = afa.guidance(curr_pos, curr_wp, avoid_vec)
                vel_list = self.vel_list
                rospy.loginfo("AFA | 속도 제어 벡터 값 : {}".format(vel_list))

                self.vel.linear.x = vel_list[0]
                self.vel.linear.y = vel_list[1] 
                self.vel.linear.z = vel_list[2]

                total_time_step += 1
                im_time = (time() - t1)*1000
                avg_time += im_time
                #f.write("\n{:.4f} {:.4f} {:.4f} {:.4f}".format(t1-t0, curr_pos[0], curr_pos[1], curr_pos[2]))
                #f.write(" {:.4f} {:.4f}".format(avoid_vec[0], avoid_vec[1]))
                #[f.write(" {:.4f} {:.4f}".format(x, y)) for [x, y] in curr_obstacle]
                
                #rospy.loginfo("AFA | Calculation Time: {:.4f}ms".format(im_time))
 
                if self.is_at_position(curr_wp[0], curr_wp[1], curr_wp[2], self.radius):
                    rospy.loginfo("AFA | Waypoint reached!! ")
                    rospy.loginfo("AFA | Average Time: {}ms".format(avg_time/total_time_step))
                    idx += 1
                if rospy.is_shutdown():
                    break

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self.set_arm(False, 5)


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)

    rostest.rosrun(PKG, 'mavros_offboard_posctl_test',
                   MavrosOffboardVelctlTest)
