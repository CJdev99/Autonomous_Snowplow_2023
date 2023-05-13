
# publishes NavSatFix to /fix, which will be used with navsat_transform_node
# to convert lat,lon to robot's coordinate frame

import rospy
import socket
import serial
import time
import pynmea2

from std_msgs.msg import Header, Float64
from sensor_msgs.msg import NavSatFix, NavSatStatus


class TrimbleBD990(object):

    GGA_TAG = '$GPGGA'
    GST_TAG = '$GPGST'
    HDT_TAG = '$GPHDT'

    FIX_STATUS_DICT = {0: 'Fix not valid',
                       1: 'Valid GPS fix',
                       2: 'Valid DGPS fix'
    }

    def __init__(self, address, port, timeout):
        self._preempted = False
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.connect((address, port))
        self._sock.settimeout(timeout)
        rospy.loginfo('Connecting to address: ' + str(address) + ' port: ' + port)

        self.lat = 0
        self.long = 0
        self.alt = 0
        self.heading = 0
        self.covariance = 0
        self.covariance_type = 2
        self.fixStatus = 0

        self.navSatPub = rospy.Publisher('~fix', NavSatFix, queue_size=1)
        self.headingPub = rospy.Publisher('~heading', Float64, queue_size=1)

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *args, **kwargs):
        self.close()

    def open(self):
        if not self._sock:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.connect((self.address, self.port))
            self._sock.settimeout(self.timeout)

    def close(self):
        self._sock.close()

    def readline(self):
        return self._sock.recv(1024)

    def preempt(self):
        self._preempted = True
        self._sock.close()

    def read_sentence(self):
        response = self.readline()              # Get raw sentence
        response = response.strip()             # Strip whitespace
        if response:
            self.decode(response)

    def decode(self, sentence):
        try:
            msg = pynmea2.parse(sentence)
            if msg.sentence_type == 'GGA':
                self.decode_gga(msg)
            elif msg.sentence_type == 'GST':
                self.decode_gst(msg)
            elif msg.sentence_type == 'HDT':
                self.decode_hdt(msg)
            else:
                rospy.logwarn('Warning [TrimbleBD990.decode]: Sentence type not recognized')
        except pynmea2.ParseError as e:
            rospy.logerr('Error [TrimbleBD990.decode]: ' + str(e))
            
    def decode_gga(self, msg):
        self.lat = float(msg.latitude)
        if msg.lat_dir == 'S':
            self.lat = (self.lat * -1.0)
        self.long = float(msg.longitude)
        if msg.lon_dir == 'W':
            self.long = (self.long * -1.0)
        self.alt = float(msg.altitude)
        self.fixStatus = int(msg.gps_qual)
    
    def decode_gst(self, msg):
    # Covariance matrix diagonal values are the squares
    # of the individual standard deviations
        lat_covariance = (float(msg.lat_stddev) * float(msg.lat_stddev))
        long_covariance = (float(msg.lon_stddev) * float(msg.lon_stddev))
        alt_covariance = (float(msg.alt_stddev) * float(msg.alt_stddev))
    # Covariance Matrix
        self.covariance = [lat_covariance, 0.0, 0.0,
                        0.0, long_covariance, 0.0,
                        0.0, 0.0, alt_covariance]
    def decode_hdt(self, msg):
        if msg.heading:
            self.heading = float(msg.heading)
        
    def publish(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "gps_link"

        navStatus = NavSatStatus()
        navStatus.status = (self.fixStatus - 1)
        navStatus.service = (0b1111)

        gpsMsg = NavSatFix()
        gpsMsg.header = header
        gpsMsg.status = navStatus
        gpsMsg.latitude = self.lat
        gpsMsg.longitude = self.long
        gpsMsg.altitude = self.alt
        gpsMsg.position_covariance = self.covariance
        gpsMsg.position_covariance_type = self.covariance_type

        self.navSatPub.publish(gpsMsg)
        self.headingPub.publish(self.heading)

    def nmea_stream(self):
        self._preempted = False
        self._sock.close()

        while not self._preempted:
            if rospy.is_shutdown():
                self.preempt()
                return

            try:
                self.read_sentence()
                self.publish()
            except Exception as e:
                rospy.logerr_throttle(1.0, 'Error [TrimbleBD990.nmea_stream]: ' + str(e))
                continue


def main():
    # Initialize node
    rospy.init_node("trimble_bd960_node")
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