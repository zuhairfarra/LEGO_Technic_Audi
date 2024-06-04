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
RW_Drive = Motor(Port.A)
FW_Drive = Motor(Port.B)
#Steer_Motor = Motor(Port.D)
# Release measuring motor
#Steer_Motor.stop()

# Optional: Register stdin for polling. This allows
# you to wait for incoming data without blocking.
keyboard = poll()
keyboard.register(stdin)

#first_iteration = True
#test_duty_cycles = [10,11,12,13,14]

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
        # Collect stationary data
        while dataOn.time() < 2000:
            time_apx = (dataOn.time()-startTime)/1000
            time_msg = b"{:.2f}".format(time_apx)
            time_array = bytearray(time_msg)

            curr_mA = hub.battery.current()  
            curr_msg = b"{:.0f}".format(curr_mA)
            curr_array = bytearray(curr_msg)

            RWspeed_dps = RW_Drive.speed(100)
            RWspeed_msg = b"{:.0f}".format(RWspeed_dps)
            RWspeed_array = bytearray(RWspeed_msg)

            FWspeed_dps = FW_Drive.speed(100)
            FWspeed_msg = b"{:.0f}".format(FWspeed_dps)
            FWspeed_array = bytearray(FWspeed_msg)

            stdout.buffer.write(time_array)
            stdout.buffer.write(b",")
            stdout.buffer.write(curr_array)
            stdout.buffer.write(b",")
            stdout.buffer.write(FWspeed_array)
            stdout.buffer.write(b",")
            stdout.buffer.write(RWspeed_array)
            
            wait(100)
        # Start driving motors
        RW_Drive.dc(30)
        FW_Drive.dc(30)
        while dataOn.time() < 12000:
            time_apx = (dataOn.time()-startTime)/1000
            time_msg = b"{:.2f}".format(time_apx)
            time_array = bytearray(time_msg)

            curr_mA = hub.battery.current()  
            curr_msg = b"{:.0f}".format(curr_mA)
            curr_array = bytearray(curr_msg)

            RWspeed_dps = RW_Drive.speed(100)
            RWspeed_msg = b"{:.0f}".format(RWspeed_dps)
            RWspeed_array = bytearray(RWspeed_msg)

            FWspeed_dps = FW_Drive.speed(100)
            FWspeed_msg = b"{:.0f}".format(FWspeed_dps)
            FWspeed_array = bytearray(FWspeed_msg)

            stdout.buffer.write(time_array)
            stdout.buffer.write(b",")
            stdout.buffer.write(curr_array)
            stdout.buffer.write(b",")
            stdout.buffer.write(FWspeed_array)
            stdout.buffer.write(b",")
            stdout.buffer.write(RWspeed_array)
            
            wait(100)
        # Stop driving motors
        RW_Drive.stop()
        FW_Drive.stop()
        while dataOn.time() < 14000:
            time_apx = (dataOn.time()-startTime)/1000
            time_msg = b"{:.2f}".format(time_apx)
            time_array = bytearray(time_msg)

            curr_mA = hub.battery.current()  
            curr_msg = b"{:.0f}".format(curr_mA)
            curr_array = bytearray(curr_msg)

            RWspeed_dps = RW_Drive.speed(100)
            RWspeed_msg = b"{:.0f}".format(RWspeed_dps)
            RWspeed_array = bytearray(RWspeed_msg)

            FWspeed_dps = FW_Drive.speed(100)
            FWspeed_msg = b"{:.0f}".format(FWspeed_dps)
            FWspeed_array = bytearray(FWspeed_msg)

            stdout.buffer.write(time_array)
            stdout.buffer.write(b",")
            stdout.buffer.write(curr_array)
            stdout.buffer.write(b",")
            stdout.buffer.write(FWspeed_array)
            stdout.buffer.write(b",")
            stdout.buffer.write(RWspeed_array)
            
            wait(100)
    else:
        stdout.buffer.write(b"bye")
        break