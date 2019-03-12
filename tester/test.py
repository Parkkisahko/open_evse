import unittest, serial, time, json

OPEN_EVSE_PORT = '/dev/tty.usbserial'
TESTER_PORT = '/dev/tty.usbmodem1412401'

baud = 115200

class OpenEVSETester(unittest.TestCase):

    def setEVState(self, state):
        self.serial_tester.write(bytes('SS {}\r\n'.format(state), 'ascii'))
    
    def setTesterStateSend(self, send):
        self.serial_tester.write(bytes('ST {}\r\n'.format(send), 'ascii'))

    def startRelayReleaseTest(self, state):
        self.serial_tester.write(bytes('TR {}\r\n'.format(state), 'ascii'))

    def setEVSECurrent(self, current):
        self.serial_open_evse.write(bytes('$SC {}\r\n'.format(current), 'ascii'))
        #print(self.serial_open_evse.readline())
    
    def initEVSE(self, current):
        self.serial_open_evse.write(b'$FF D 0') # disable diode chec
        #print(self.serial_open_evse.readline())
        time.sleep(0.5)
        self.serial_open_evse.write(b'$FF G 0') # disable ground check
        #print(self.serial_open_evse.readline())
        time.sleep(0.5)
        self.serial_open_evse.write(b'$SL 2')   # Set service level 2
        #print(self.serial_open_evse.readline())
        time.sleep(0.5)
        self.setEVSECurrent(6)
        time.sleep(0.5)

    def assert_freq(self, meas):
        self.assertTrue(980 <= meas['freq'] <= 1020, 'Frequency out of bounds: {} Hz'.format(meas['freq']))

    def assert_PWM(self, pwm, amps):
        if amps == 0:
            cycle = 0
        elif amps <= 51:
            cycle = amps / 0.6
        else:
            cycle = amps / 2.5 + 64

        self.assertTrue(cycle - 1.2 <= pwm/100.0 <= cycle + 1.2, 'PWM out of bounds, duty cycle: {} %'.format(round(pwm/100.0, 1)))

    def setUp(self):
        self.serial_open_evse = serial.Serial(OPEN_EVSE_PORT, baud)
        self.serial_tester = serial.Serial(TESTER_PORT, baud)

        time.sleep(1)

        # set state to A (not charging)
        self.setEVState('A')
        time.sleep(0.1)
        self.setTesterStateSend('0')
    
    def tearDown(self):
        self.setTesterStateSend('0')
        time.sleep(0.1)
        self.setEVState('A')
        

class PWMOnTests(OpenEVSETester):

    def setUp(self):
        super().setUp()
        self.initEVSE(6)
        self.serial_tester.reset_input_buffer()

    # Test starts with state A, relay should remain open
    def test_basic(self):
        time.sleep(0.5)
        self.setTesterStateSend('1')   
        for i in range(10):
            #incoming = self.serial_tester.readline()
            incoming = json.loads(self.serial_tester.readline())
            self.assertEqual(incoming['relay'], 0, "Relay closed even when should be open")
            self.assertEqual(incoming['freq'], 0, "PWM generates frequency even if should not")
            self.assertEqual(incoming['state'], 0)

    # Test going to state B (plugged in)
    def test_plug_in(self):
        print()
        print()
        print('Starting plugged in test')
        pwms = []
        time.sleep(0.5)
        self.setEVState('B')
        time.sleep(0.5)
        self.setTesterStateSend('1')
        for i in range(10):
            incoming = json.loads(self.serial_tester.readline())
            print(incoming)
            self.assertEqual(incoming['relay'], 0, "Relay closed even when should be open")
            self.assert_freq(incoming)
            pwms.append(incoming['PWM'])
            #self.assert_PWM(incoming, 6)
            #self.assertTrue(9 <= round(incoming['freq']/100.0) <= 11, 'Frequency failed, measured freq: {}'.format(round(incoming['freq']/100.0)))
            #self.assertAlmostEqual(round(incoming.freq/100.0), 10)
        self.assert_PWM(sum(pwms)/len(pwms), 6)
        print('Plugged in test finished with average PWM of: {} %'.format(round(sum(pwms)/len(pwms)/100.0, 1)))
    
    # Test pwm duty cycle at several amps
    def assert_pwm_amps(self, amps):
        print()
        print()
        print('Starting pwm test with amps: {} A'.format(amps))
        self.setEVSECurrent(amps)
        time.sleep(0.5)
        self.setEVState('C')
        time.sleep(1)
        pwms = []
        self.setTesterStateSend('1')
        for i in range(10):
            incoming = json.loads(self.serial_tester.readline())
            print(incoming)
            self.assert_freq(incoming)
            pwms.append(incoming['PWM'])
        self.assert_PWM(sum(pwms)/len(pwms), amps)
        print('PWM test finished with average PWM of: {} %'.format(round(sum(pwms)/len(pwms)/100.0, 1)))
    
    def test_12_A(self):
        self.assert_pwm_amps(12)
    
    def test_18_A(self):
        self.assert_pwm_amps(18)
    
    def test_30_A(self):
        self.assert_pwm_amps(30)
    
    def test_63_A(self):
        self.assert_pwm_amps(63)

    # Test instant start charging
    def test_relay_timing_start_charging(self):
        print()
        print()
        print("Starting relay close test")
        pwms = []
        time.sleep(0.5)
        self.setEVState('C')
        self.setTesterStateSend('1')
        relay_changed = False
        first_ts = None
        freq_start_ts = None
        last_ts = None
        relay_true_counter = 0
        while True:
            incoming = json.loads(self.serial_tester.readline())
            print(incoming)
            if first_ts == None:
                first_ts = incoming['time']
            if incoming['freq'] > 0:
                if freq_start_ts == None:
                    freq_start_ts = incoming['time']
                self.assert_freq(incoming)
                pwms.append(incoming['PWM'])
            if incoming['relay'] == 1 and not relay_changed:
                last_ts = incoming['time']
                relay_changed = True
            if relay_changed:
                self.assertEqual(incoming['relay'], 1)
                relay_true_counter += 1
            if relay_true_counter == 20:
                break
        
        self.assert_PWM(sum(pwms)/len(pwms), 6)
        self.assertTrue(last_ts - first_ts < 2800, 'Relay took too long to close: {} ms'.format(last_ts - first_ts))
        print('Relay close test finished, relay delay: {} ms, PWM start delay: {} ms, PWM duty cycle: {} %'.format(last_ts - first_ts, freq_start_ts - first_ts, round(sum(pwms)/len(pwms)/100.0, 1)))
    
    # Test relay off timing
    def assert_relay_release_change(self, new_state, max_time=100):
        print()
        print()
        print('Starting relay release test, changing state to: {}'.format(new_state))
        time.sleep(0.5)
        self.setEVState('C')
        self.setTesterStateSend('1')
        
        ts = time.time()
        incoming = json.loads(self.serial_tester.readline())
        while incoming['relay'] != 1:
            incoming = json.loads(self.serial_tester.readline())

        relay_delay = time.time() - ts

        print('Relay closed, with delay of {} s'.format(round(relay_delay, 2)))
        
        self.setTesterStateSend('0')
        self.serial_tester.reset_input_buffer()

        self.startRelayReleaseTest(new_state)
        #print(self.serial_tester.readline())
        incoming = json.loads(self.serial_tester.readline())
        self.assertLess(incoming['delay'], max_time)

        print('Relay release test finished with delay of: {} ms'.format(incoming['delay']))
    
    def test_relay_release_to_a(self):
        self.assert_relay_release_change('A')
    
    def test_relay_release_to_b(self):
        self.assert_relay_release_change('B', 3000)     # 3 sec according to SAE J1772 - 2016, 100 ms according to IEC 61851

    
if __name__ == '__main__':
    unittest.main()