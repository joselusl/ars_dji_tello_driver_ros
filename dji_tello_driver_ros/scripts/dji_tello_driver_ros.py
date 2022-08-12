#! /usr/bin/env python

import numpy as np

import threading
import time
import av
import cv2


# ROS Imports
import rospy
import rospkg

from cv_bridge import CvBridge

from std_msgs.msg import Empty
from std_msgs.msg import Bool, Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped

from dji_tello_msgs.msg import FlightData, FlipControl


from tellopy import Tello


from utils import connect_wifi_device as cwd




class DjiTelloDriverRos(object):

    #
    _frame_skip = 300  # - skip the first 300 frames to avoid inital lag

    # Tello WiFi credentials
    tello_ssid = None
    tello_pw = None

    # Subscribers
    tello_vel_cmd_stamped_sub = None
    flag_tello_control_enabling_sub = None
    tello_takeoff_sub = None
    tello_land_sub = None
    tello_flip_controll_sub = None

    #
    flag_tello_control_enabled = None


    # - Timers
    _battery_percentage_display_timer = None
    # will display to screen battery percentage every 5 seconds
    _battery_percentage_timer_interval = 5.0

    # Params
    _connect_to_tello_wifi_auto = True
    tello_ssid = None
    tello_pw = None


    #
    _stop_request = None
    _video_thread = None


    #
    _flight_data = None
    #
    _current_battery_percentage = 0
    


    def __init__(self):

        # ROS OpenCV bridge
        self._cv_bridge = CvBridge()

        # connect to the drone
        self._tello = Tello()
        self._tello.set_loglevel(self._tello.LOG_WARN)

        # - Setting shutdown routine
        rospy.on_shutdown(self._shutdown_routine)

        #
        self.flag_tello_control_enabled = True


        # End
        return


    def init(self, node_name='dji_tello_driver_ros_node'):

        # Init ROS
        rospy.init_node(node_name, anonymous=True)

        
        # Package path
        pkg_path = rospkg.RosPack().get_path('dji_tello_driver_ros')
        

        #### READING PARAMETERS ###

        # Read params
        self.read_params()


        # Connect to Tello Network
        self._connect_to_tello_network()

        
        # End
        return


    def open(self):
        
        # Publishers
        #
        self._image_pub = rospy.Publisher("camera/image_raw", Image, queue_size=1)
        #
        self._flight_data_pub = rospy.Publisher("flight_data", FlightData, queue_size=1)


        # Subscribers
        #
        self.tello_vel_cmd_stamped_sub = rospy.Subscriber("cmd_vel_stamped", TwistStamped, self.tello_vel_cmd_stamped_callback)
        #
        self.flag_tello_control_enabling_sub = rospy.Subscriber("flag_tello_control_enabled", Bool, self.tello_control_enabling_callback)
        #
        self.tello_takeoff_sub = rospy.Subscriber("takeoff", Empty, self._takeoff_callback, queue_size=1)
        #
        self.tello_land_sub = rospy.Subscriber("land", Empty, self._land_callback, queue_size=1)
        #
        self.tello_flip_controll_sub = rospy.Subscriber("flip_control", FlipControl, self._flip_control_callback, queue_size=1)

        
        # Timers
        self._battery_percentage_display_timer = rospy.Timer(rospy.Duration(self._battery_percentage_timer_interval), self._battery_percentage_display_timer_callback)


        # Video thread
        self._start_video_threads()

        # Tello Flight data
        self._tello.subscribe(self._tello.EVENT_FLIGHT_DATA, self._flight_data_handler)


        # End
        return


    def read_params(self):
        print("[info] - Reading parameters")

        self.tello_ssid = rospy.get_param("/tello_driver_node/tello_ssid", default=self.tello_ssid)
        self.tello_pw = rospy.get_param("/tello_driver_node/tello_pw", default=self.tello_pw)
        self._connect_to_tello_wifi_auto = rospy.get_param("/tello_driver_node/connect_to_tello_wifi_auto", default=self._connect_to_tello_wifi_auto)
        
        print("[info] - Finished reading parameters")

        # End
        return


    def close(self):


        # End
        return


    def tello_vel_cmd_stamped_callback(self, twist_msg):

        #
        if(self.flag_tello_control_enabled is False):
            return

        # Cmd
        lin_vel_cmd = np.zeros((3,), dtype=float)
        lin_vel_cmd[0] = twist_msg.twist.linear.x
        lin_vel_cmd[1] = twist_msg.twist.linear.y
        lin_vel_cmd[2] = twist_msg.twist.linear.z

        alg_vel_cmd = np.zeros((3,), dtype=float)
        alg_vel_cmd[0] = twist_msg.twist.angular.x
        alg_vel_cmd[1] = twist_msg.twist.angular.y
        alg_vel_cmd[2] = twist_msg.twist.angular.z

        # Set
        self.set_cmd_vel(lin_vel_cmd, alg_vel_cmd)

        # End
        return


    def tello_control_enabling_callback(self, msg):

        self.flag_tello_control_enabled = msg.data

        if(self.flag_tello_control_enabled is False):
            self.set_cmd_hover()

        return


    def set_cmd_vel(self, lin_cmd_vel, ang_cmd_vel):
        self._tello.set_pitch(lin_cmd_vel[0])  # linear X value
        self._tello.set_roll(-lin_cmd_vel[1])  # linear Y value
        self._tello.set_throttle(lin_cmd_vel[2])  # linear Z value
        self._tello.set_yaw(-ang_cmd_vel[2])  # angular Z value

        # End
        return


    def set_cmd_hover(self):
        self._tello.set_pitch(0.0)  # linear X value
        self._tello.set_roll(0.0)  # linear Y value
        self._tello.set_throttle(0.0)  # linear Z value
        self._tello.set_yaw(0.0)  # angular Z value

        # End
        return


    # +--------------------+
    # | Start of Callbacks |
    # +--------------------+

    def _takeoff_callback(self, msg):
        msg  # - just for not having linting errors
        print("[info] [Tello_driver] - Taking off")
        self._tello.takeoff()

        # End
        return


    def _land_callback(self, msg):
        msg  # - just for not having linting errors
        print("[info] [Tello_driver] - Landing")
        self._tello.land()

        # End
        return


    def _flip_control_callback(self, msg):
        #
        if(self.flag_tello_control_enabled is False):
            return

        print("[info] [Tello_driver] - Performing a flip")
        if msg.flip_forward:
            self._tello.flip_forward()
        elif msg.flip_backward:
            self._tello.flip_back()
        elif msg.flip_left:
            self._tello.flip_left()
        elif msg.flip_right:
            self._tello.flip_right()

        # End
        return


    def _battery_percentage_display_timer_callback(self, msg):
        print(f"[info] [Tello_driver] - Drone's battery percentage is {self._current_battery_percentage}%")

        # End
        return


    def _flight_data_handler(self, event, sender, data):
        flight_data = FlightData()

        # - Battery data
        flight_data.battery_low = data.battery_low
        flight_data.battery_lower = data.battery_lower
        flight_data.battery_percentage = data.battery_percentage
        flight_data.drone_battery_left = data.drone_battery_left
        # flight_data.drone_fly_time_left = data.drone_fly_time_left

        # =========================================================================

        # - States
        flight_data.battery_state = data.battery_state
        flight_data.camera_state = data.camera_state
        flight_data.electrical_machinery_state = data.electrical_machinery_state
        flight_data.down_visual_state = data.down_visual_state
        flight_data.gravity_state = data.gravity_state
        flight_data.imu_calibration_state = data.imu_calibration_state
        flight_data.imu_state = data.imu_state
        flight_data.power_state = data.power_state
        flight_data.pressure_state = data.pressure_state
        flight_data.wind_state = data.wind_state

        # =========================================================================

        # - Stats
        flight_data.drone_hover = data.drone_hover
        flight_data.em_open = data.em_open
        flight_data.em_sky = data.em_sky
        flight_data.em_ground = data.em_ground
        flight_data.factory_mode = data.factory_mode
        flight_data.fly_mode = data.fly_mode
        # flight_data.fly_time = data.fly_time
        flight_data.front_in = data.front_in
        flight_data.front_lsc = data.front_lsc
        flight_data.front_out = data.front_out

        # =========================================================================

        # - Sensors
        flight_data.fly_speed = data.fly_speed
        flight_data.east_speed = data.east_speed
        flight_data.ground_speed = data.ground_speed
        flight_data.height = data.height
        flight_data.light_strength = data.light_strength
        flight_data.north_speed = data.north_speed
        flight_data.temperature_height = data.temperature_height

        # =========================================================================

        # - Other
        flight_data.outage_recording = data.outage_recording
        flight_data.smart_video_exit_mode = data.smart_video_exit_mode
        # flight_data.throw_fly_timer = data.throw_fly_timer

        # =========================================================================

        # - WiFi
        flight_data.wifi_disturb = data.wifi_disturb
        flight_data.wifi_strength = data.wifi_strength


        # Reading some parameters
        # Flight data
        self._flight_data = flight_data
        # Battery percentage
        self._current_battery_percentage = flight_data.battery_percentage


        # Publish Flight data
        self._flight_data_pub.publish(flight_data)

        # End
        return

    # +------------------+
    # | End of Callbacks |
    # +------------------+

    def _start_video_threads(self):
        # start video thread
        self._stop_request = threading.Event()
        self._video_thread = threading.Thread(target=self._video_worker_loop)
        self._video_thread.start()

        # End
        return


    def _video_worker_loop(self):
        # get video stream, open with PyAV
        video_stream = self._tello.get_video_stream()

        container = av.open(video_stream)

        print("[info] [Tello_driver] - video stream is starting")

        for frame in container.decode(video=0):
            if self._frame_skip > 0:
                self._frame_skip = self._frame_skip - 1

                continue

            # convert PyAV frame => PIL image => OpenCV image
            image = np.array(frame.to_image())

            # VIDEO RESOLUTION
            # original 960x720

            # Reduced image size to have less delay
            image = cv2.resize(image, (480, 360), interpolation=cv2.INTER_LINEAR)

            # convert OpenCV image => ROS Image message
            image = self._cv_bridge.cv2_to_imgmsg(image, "rgb8")

            self._image_pub.publish(image)

            # check for normal shutdown
            if self._stop_request.isSet():
                return

        # End
        return


    def _shutdown_routine(self):
        print("[info] Shuting down DJI Tello")

        # force a landing
        self._tello.land()

        # Wait until it lands
        time.sleep(2)

        # stop the video thread
        if(self._video_thread is not None):
            self._stop_request.set()
            self._video_thread.join(timeout=2)

        # shut down the drone
        self._tello.quit()

        # End
        return


    def _connect_to_tello_network(self):
        print("[info] [Tello_driver] - Connecting to drone")
        if self._connect_to_tello_wifi_auto:
            if not cwd.connect_device(self.tello_ssid, self.tello_pw, verbose=False):
                print("[error] [Tello_driver] - Connection to drone unsuccessful!")
                rospy.signal_shutdown(
                    "Not able to establish connection with Tello network"
                )
        self._tello.connect()
        self._tello.wait_for_connection(5)
        print("[info] [Tello_driver] - Connection to drone successfull")

        # End
        return
