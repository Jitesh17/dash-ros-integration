#!/usr/bin/env python
# A dash server along with a ROS node

import rospy

from dashboard import Dashboard


def main():
    rospy.init_node('clinical_dashboard', anonymous=True)
    dashboard = Dashboard()
    dashboard.start()
    # The start actually spins internally using dash


if __name__ == '__main__':
    main()
