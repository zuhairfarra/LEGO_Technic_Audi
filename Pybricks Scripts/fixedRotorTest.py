# Standard MicroPython modules
from usys import stdin, stdout
from uselect import poll
from uio import StringIO
from umath import fmod

from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor
from pybricks.tools import wait, StopWatch
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis

hub = TechnicHub()
steerMotor = Motor(Port.D)

# Optional: Register stdin for polling. This allows
# you to wait for incoming data without blocking.
keyboard = poll()
keyboard.register(stdin)

first_iteration = True

test_duty_cycles = [10,20,30,40,50]

while True:

    # Let the remote program know we are ready for a command.
    stdout.buffer.write(b"rdy")

    # Optional: Check available input.
    while not keyboard.poll(0):
        # Optional: Do something here.
        wait(10)

    # Read three bytes.
    cmd = stdin.buffer.read(1)

    if cmd == b"s":
        hub.light.blink(Color.MAGENTA,[500, 500])
        # Begin logging data
        dataOn = StopWatch()
        startTime = dataOn.time()
        for idx in range(5):
            # Collect stationary data
            while dataOn.time() < 2000+(5000*idx):
                time_apx = (dataOn.time()-startTime)/1000
                time_msg = b"{:.2f}".format(time_apx)
                time_array = bytearray(time_msg)

                curr_mA = hub.battery.current()  
                curr_msg = b"{:.0f}".format(curr_mA)
                curr_array = bytearray(curr_msg)

                stdout.buffer.write(time_array)
                stdout.buffer.write(b",")
                stdout.buffer.write(curr_array)
                
                wait(100)
            # Turn motors on
            steerMotor.dc(test_duty_cycles[idx])
            # Collect driving data 
            while dataOn.time() < 5000+(5000*idx):
                time_apx = (dataOn.time()-startTime)/1000
                time_msg = b"{:.2f}".format(time_apx)
                time_array = bytearray(time_msg)

                curr_mA = hub.battery.current()  
                curr_msg = b"{:.0f}".format(curr_mA)
                curr_array = bytearray(curr_msg)

                stdout.buffer.write(time_array)
                stdout.buffer.write(b",")
                stdout.buffer.write(curr_array)
                
                wait(100)
            # Stop the motor
            steerMotor.stop()
    else:
        stdout.buffer.write(b"bye")
        break