#!/usr/bin/env python

import sys
import rospy


from dji_tello_driver_ros import DjiTelloDriverRos


def main():
    dji_tello_driver_ros = DjiTelloDriverRos()

    dji_tello_driver_ros.init()
    dji_tello_driver_ros.open()

    try:
        dji_tello_driver_ros.run()
    except rospy.ROSInterruptException:
        pass

    dji_tello_driver_ros.close()

    return 0


if __name__ == "__main__":
    main()
