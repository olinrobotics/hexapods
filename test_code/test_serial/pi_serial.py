import serial
import sys
import time
import rospy
from std_msgs.msg import String


class PiSerial(object):
    """ An object for handling serial communication between a
    Raspberry Pi and an Arduino.

    Listens to commands published to the /serial channel, which should be
    commands and arguments separated by colons and constrained by the restrictions
    described in the Hexapod Wiki. They do not need to be terminated in semicolons.


    """

    def __init__(self, port = 0, baud = 9600):
        """ Initializes the PiSerial object.

        kwargs: port (int) - Specifies the USB port of the device (default 0)
                baud (int) - Specifies the baud rate of communication (default 9600)
        """

        self.ser = serial.Serial("/dev/ttyACM%s" % port)

        print("Setting up serial controller.")

        # Set up ROS node
        channel = "serial"
        rospy.init_node('serial_controller', anonymous=True)
        self.ros_rate = rospy.Rate(30)  #   TODO choose optimal rate
        self.command_listener = rospy.Subscriber(channel,
                                                String,
                                                self.listener_callback,
                                                queue_size = 10)
        print("Serial controller listening on channel: %s" % channel)

        # Run loop until ros shuts down
        while not rospy.is_shutdown():
            self.listen()


    def listen(self):
        """ Not much you need to do here, sleep ros until you need to do something"""

        self.ros_rate.sleep()


    def listener_callback(self, data):
        """ Send any commands you hear to Arduino. """

        msg = str(data.data)
        print(msg)
        self.send(*(msg.split(":")))


    def send_raw(self, string):
        """ Writes a string to Serial. """
        self.ser.write(string)


    def send(self, command, *args):
        """ Sends a command over serial, with arguments separated by
        colons and ending with a semicolon (e.g. "MOVE:3.5;")

        command: string argument of command (e.g. MOVE)
        args:    associated arguments for command (e.g. 3.5)
        """

        send_str = command              #   Send command string
        for arg in args:
            send_str += ":" + str(arg)  #   Add arguments, separated by colons
        if send_str[-1] != ";":
            send_str += ";"             #   Terminate message with semicolon
        self.send_raw(send_str)         #   Push string to serial


def assert_py(version):
    """ Raises an exception if a version of python other than
    the specified version, as an int (e.g. 3). """

    if sys.version_info[0] != version:
        raise Exception("Use only Python %s." % version)


if __name__ == '__main__':
    assert_py(2);
    a = PiSerial(0, 9600);
    # while True:
    #     send_str = raw_input("Send something over Serial: ")
    #     a.send_raw(send_str)
