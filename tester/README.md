# Basics
This testing library contains both an Arduino sketch, an example schema and python test cases to test relay and PWM timings to match 

# Python tests
## Setup
- Install pip first. ´sudo apt-get install python3-pip´
- Setup virtual environment: ´virtualenv -p python3 _py_env´
- Activate virtual environment: ´source _py_env/bin/activate´
- Install packages: ´pip install -r requirements.txt´

## Running tests
- Identify the serial ports used for both Arduino (testing device) and OpenEVSE
-- On OS X and Linux: ´ls /dev/tty*´
-- Replace OPEN_EVSE_PORT and TESTER_PORT with the correct ports in test.py
- Run tests by entering ´python test.py´

## Test cases
- TBD

# OpenEVSE tester
## Notes
- The schema provided is sensitive to board voltage due to selected optocoupler components not fully saturating at voltages below 5 V
- To overcome this issue, use a separate power supply for the Arduino

## Possible improvements
- Change optos to transistors
- Measure voltage at PWM high and low state
- Add circuits and corresponding code to provide for following automated test cases:
-- Diode check fail
-- Ground check fail
-- Stuck relay check (NOTE: Requires high voltages on board!)

# License
Open EVSE tester is licensed under the GNU license.
Copyright 2019 Santeri Oksanen / Parking Energy Ltd