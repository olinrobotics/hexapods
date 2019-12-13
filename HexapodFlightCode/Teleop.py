import serial
import sys
import time
import pygame

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

        self.ser = serial.Serial("COM%s" % port)
        print("Serial connected")
        self.start_gui()
        if pygame.joystick.get_count():
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
        else:
            self.joystick = False
        self.x = 0
        self.y = 0
        self.strafe = False
        self.pivot = False
        self.dance = False
        self.run()

    def start_gui(self):
        pygame.init()
        pygame.display.set_caption("Hexapod Control Station")
        self.screen = pygame.display.set_mode([300, 300])

    def run(self):
        ''' Iteratively call update '''
        clock = pygame.time.Clock()
        while True:
            for event in pygame.event.get():
                if event.type is pygame.KEYDOWN:
                    self.send_raw(event.unicode)
                if event.type is pygame.JOYBUTTONDOWN:
                    if event.button == 1:
                        self.dance = not self.dance
                        if self.dance:
                            self.send_raw('b')
                        else:
                            self.send_raw(' ')
                    if event.button == 2:
                        self.pivot = not self.pivot
                        if self.pivot:
                            self.send_raw('m')
                        else:
                            self.send_raw('n')
                if event.type is pygame.JOYBUTTONUP:
                    if event.button == 1:
                        self.strafe = False
                if event.type is pygame.JOYAXISMOTION:
                    if event.axis == 0:
                        self.x = int(event.value)
                    elif event.axis == 1:
                        self.y = int(event.value)
                    if self.x > 0:
                        if self.strafe:
                            self.send_raw('d')
                        else:
                            self.send_raw('e')
                    elif self.x < 0:
                        if self.strafe:
                            self.send_raw('a')
                        else:
                            self.send_raw('q')
                    if self.y > 0:
                        self.send_raw('s')
                    elif self.y < 0:
                        self.send_raw('w')
                    else:
                        self.send_raw(' ')
                if event.type == pygame.QUIT:
                    pygame.display.quit()
                    sys.exit()
            dt = clock.tick(5) # Hz
            while self.ser.in_waiting:
                # Warning: do not remove this readline, even if you stop printing to console!
                print(f"{time.time():.3f}: {self.ser.readline().decode('utf-8').strip()}")
            pygame.display.update()

    def send_raw(self, string):
        """ Writes a string to Serial. """
        print("> " + string)
        self.ser.reset_output_buffer()
        self.ser.write(string.encode('utf-8'))


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
    a = PiSerial(18, 9600)