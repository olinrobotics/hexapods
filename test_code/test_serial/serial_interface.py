import serial
import sys
import time


class PiSerial(object):
    """ An object for handling serial communication between a
    Raspberry Pi and an Arduino. """

    def __init__(self, port = 0, baud = 9600):
        """ Initializes the PiSerial object.

        kwargs: port (int) - Specifies the USB port of the device (default 0)
                baud (int) - Specifies the baud rate of communication (default 9600)
        """

        try:
            self.ser = serial.Serial("/dev/ttyACM%s" % port)
        except:
            print("Serial port not found, or port is busy.")

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
        send_str += ";"                 #   Terminate message with semicolon
        self.send_raw(send_str)         #   Push string to serial


def assert_py(version):
    """ Raises an exception if a version of python other than
    the specified version, as an int (e.g. 3). """

    if sys.version_info[0] != version:
        raise Exception("Use only Python %s." % version)


if __name__ == '__main__':
    assert_py(2);
    a = PiSerial(0, 9600);
    while True:
        send_str = raw_input("Send something over Serial: ")
        a.send_raw(send_str)
