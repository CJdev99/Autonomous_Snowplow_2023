import rospy
import serial
import time
import socket

from std_msgs.msg import Header, Float64
from sensor_msgs.msg import NavSatFix, NavSatStatus

# publishes NavSatFix to /fix, which will be used with navsat_transform_node

class TrimbleBD990(object):


    GGA_TAG = '$GNGGA'
    GST_TAG = '$GNGST'
    HDT_TAG = '$GNHDT'

    FIX_STATUS_DICT = {0: 'Fix not valid',
                       1: 'Valid GPS fix',
                       2: 'Valid DGPS fix'
    }


    def __init__(self, TCP_IP, TCP_port, buffer_size, baudrate, timeout):

        #connect to socket
        self.TCP_IP = "192.168.1.100"
        self.TCP_port = 5017
        self.buffer_size = 1024
        self._preempted = False
        #self._ser = serial.Serial(port, baudrate, timeout=timeout)
        #initialize sock stream
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #connect to socket
        #self._sock.connect((TCP_IP, TCP_port))
        rospy.loginfo('Connecting to port: ' + TCP_port)

        self.lat = 0
        self.long = 0
        self.alt = 0
        self.heading = 0
        self.covariance = 0
        self.covariance_type = 2
        self.fixStatus = 0

        self.navSatPub = rospy.Publisher('~fix', NavSatFix, queue_size=1)
        #self.headingPub = rospy.Publisher('~heading', Float64, queue_size=1) # bd990 does not support heading


    def __enter__(self):
        self.open()
        return self
    
    def __exit__(self, *args, **kwargs):
        self.close()

    def open(self):
       
        self._sock.connect(self.TCP_IP,self.TCP_port)

        
    
    def close(self):
        self._sock.close()

    def readline(self):
        return self._sock.recv(self.buffer_size)

    def preempt(self):
        self._preempted = True
        #self._sock.send_break()
    """
    test:
    def read_sentence(self):
    response = self.readline()              # Get raw sentence
    response = response.strip()             # Strip whitespace
    if response:
        self.decode(response)
    """

    def read_sentence(self):
        response = self.readline()              # Get raw sentence
        response = response.strip().split('*')  # Split at checksum
        sentence = response[0].split(',')       # Split data
        if sentence:
            self.decode(sentence)

    def decode(self, sentence):
        tag = sentence[0]
        if tag == self.GGA_TAG:
            self.decode_gga(sentence)
        elif tag == self.GST_TAG:
            self.decode_gst(sentence)
        elif tag == self.HDT_TAG:
            self.decode_hdt(sentence)
        else:
            rospy.logwarn('Warning [TrimbleBD990.decode]: Sentence type not recognized')
    """
    test:
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
     """

    def decode_gga(self, sentence):
        self.lat = float(sentence[2])
        if sentence[3] == 'S':
            self.lat = (self.lat * -1.0)

        self.long = float(sentence[4])
        if sentence[5] == 'W':
            self.long = (self.long * -1.0)

        self.alt = float(sentence[9])
        self.fixStatus = int(sentence[6])

    def decode_gst(self, sentence):
        # Covariance matrix diagonal values are the squares
        # of the individual standard deviations
        lat_covariance = (float(sentence[6]) * float(sentence[6]))
        long_covariance = (float(sentence[7]) * float(sentence[7]))
        alt_covariance = (float(sentence[8]) * float(sentence[8]))

        # Covariance Matrix
        self.covariance = [lat_covariance, 0.0, 0.0,
                           0.0, long_covariance, 0.0,
                           0.0, 0.0, alt_covariance]

    def decode_hdt(self, sentence):
        if sentence[1]:
            self.heading = float(sentence[1])

    def publish(self):
        header = Header()
        header.stamp = rospy.Time.now()

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
        #self._sock.send_break()
        #self._sock.flush()

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
