import unittest, serial, time

OPEN_EVSE_PORT = '/dev/tty.usbserial'
TESTER_PORT = '/dev/tty.usbmodem1412401'

baud = 115200

class OpenEVSETester(unittest.TestCase):

    def setEVState(self, state):
        self.serial_tester.write(bytes('SS {}\r\n'.format(state), 'ascii'))
    
    def setEVSECurrent(self, current):
        self.serial_open_evse.write(bytes('$SC {}\r\n'.format(current), 'ascii'))
    
    def initEVSE(self):
        self.serial_open_evse.write(b'$FF D 0') # disable diode chec
        time.sleep(0.5)
        self.serial_open_evse.write(b'$FF G 0') # disable ground check
        time.sleep(0.5)
        self.setEVSECurrent(6)
        time.sleep(0.5)

    def setUp(self):
        self.serial_open_evse = serial.Serial(OPEN_EVSE_PORT, baud)
        self.serial_tester = serial.Serial(TESTER_PORT, baud)

        time.sleep(1)

        # set state to A (not charging)
        self.setEVState('C')
        self.initEVSE()
    
    def test_basic(self):
        time.sleep(2)
        incoming = self.serial_tester.readline()
        while incoming:
            print(incoming)
            incoming = self.serial_tester.readline()
    
if __name__ == '__main__':
    unittest.main()