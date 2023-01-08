#! /usr/bin/python3

import rospy

from nmea_socket_pub import TrimbleBD960
# executable
if __name__ == "__main__":
    rospy.init_node('trimble-gps')

    address = rospy.get_param('~address', "192.168.1.100")
    port = rospy.get_param('~port', 5017)
    #buffer_size = rospy.get_param('~buffer_size', 1024)
    #baudrate = rospy.get_param('~baudrate', 38400)
    timeout = rospy.get_param('~timeout', 1)

    with TrimbleBD960(address, port, timeout) as gps:
        try:
            gps.nmea_stream()
        except KeyboardInterrupt:
            gps.preempt()