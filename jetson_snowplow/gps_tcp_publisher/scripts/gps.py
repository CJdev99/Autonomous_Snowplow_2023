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
            
    """
    test:
    #!/usr/bin/env python

import rospy
import serial
import time
import socket
from std_msgs.msg import Header, Float64
from sensor_msgs.msg import NavSatFix, NavSatStatus
from TrimbleBD990 import TrimbleBD990

def main():
    # Initialize node
    rospy.init_node("trimble_bd990_node")

    # Create TrimbleBD990 object
    bd990 = TrimbleBD990("192.168.1.100", 5017, 1024, 9600, 1)

    # Open connection to device
    bd990.open()

    # Start NMEA stream
    bd990.nmea_stream()

    # Close connection to device
    bd990.close()


if __name__ == "__main__":
    main()
  """
    
