#!/usr/bin/env python

import rospy
from gps_tcp_publisher import TrimbleBD990

if __name__ == "__main__":
    rospy.init_node('trimble-gps')

    TCP_port = rospy.get_param('~TCP_port', 5017)
    TCP_ip = rospy.get_param('~TCP_ip', "192.168.1.100")
    buffer_size = rospy.get_param('~buffer_size', 1024)
    baudrate = rospy.get_param('~baudrate', 38400)
    timeout = rospy.get_param('~timeout', 1)

    with TrimbleBD990(TCP_ip, TCP_port, buffer_size, baudrate, timeout) as gps:
        try:
            gps.nmea_stream()
        except KeyboardInterrupt:
            gps.preempt()
