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
Steer_Motor = Motor(Port.D)

# Optional: Register stdin for polling. This allows
# you to wait for incoming data without blocking.
keyboard = poll()
keyboard.register(stdin)

while True:
    if hub.imu.ready() == True:
        # Let the remote program know we are ready for a command.
        stdout.buffer.write(b"rdy")

        # Optional: Check available input.
        while not keyboard.poll(0):
            # Optional: Do something here.
            wait(10)

        # Read three bytes.
        cmd = stdin.buffer.read(1)

        # Center steering mechanism
        if cmd == b"c":
            # The run_until_stalled gives us the angle at which it stalled.
            # We want to know this value for both endpoints.
            left_end = Steer_Motor.run_until_stalled(-200, duty_limit=30)
            right_end = Steer_Motor.run_until_stalled(200, duty_limit=30)

            print(left_end)
            print(right_end)

            # We have just moved to the rightmost endstop. So, we can reset
            # this angle to be half the distance between the two endpoints.
            # That way, the middle corresponds to 0 degrees.
            Steer_Motor.reset_angle((right_end - left_end)/2)

            # From now on we can simply run towards zero to reach the middle.
            Steer_Motor.run_target(200, 0)

            wait(1000)
        # Steer to max angle
        elif cmd == b"s":

            Steer_Motor.run_target(200, 0.3*left_end,Stop.HOLD)

            print(left_end)
            print(Steer_Motor.angle())
            wait(1000)
        # Begin driving forward
        elif cmd == b"g":
            # Reset heading
            hub.imu.reset_heading(0)
            Steer_Motor.hold()
            # Begin logging data
            dataOn = StopWatch()
            startTime = dataOn.time()
            # Collect stationary data
            while dataOn.time() < 2000:
                time_apx = (dataOn.time()-startTime)/1000
                time_msg = b"{:.2f}".format(time_apx)
                time_array = bytearray(time_msg)

                heading = hub.imu.heading()
                heading_msg = b"{:.1f}".format(heading)
                heading_array = bytearray(heading_msg)

                RWspeed_dps = RW_Drive.speed(100)
                RWspeed_msg = b"{:.0f}".format(RWspeed_dps)
                RWspeed_array = bytearray(RWspeed_msg)

                #latAccel = hub.imu.acceleration(Axis.Y)
                #latAccel_msg = b"{:.0f}".format(latAccel)
                #latAccel_array = bytearray(latAccel_msg)

                stdout.buffer.write(time_array)
                stdout.buffer.write(b",")
                stdout.buffer.write(heading_array)
                stdout.buffer.write(b",")
                stdout.buffer.write(RWspeed_array)
                #stdout.buffer.write(b",")
                #stdout.buffer.write(latAccel_array)
                
                wait(100)

            # Start driving motors
            RW_Drive.dc(100)
            FW_Drive.dc(100)
            
            while hub.imu.heading() > -90:

                time_apx = (dataOn.time()-startTime)/1000
                time_msg = b"{:.2f}".format(time_apx)
                time_array = bytearray(time_msg)

                heading = hub.imu.heading()
                heading_msg = b"{:.1f}".format(heading)
                heading_array = bytearray(heading_msg)

                RWspeed_dps = RW_Drive.speed(100)
                RWspeed_msg = b"{:.0f}".format(RWspeed_dps)
                RWspeed_array = bytearray(RWspeed_msg)

                #latAccel = hub.imu.acceleration(Axis.Y)
                #latAccel_msg = b"{:.0f}".format(latAccel)
                #latAccel_array = bytearray(latAccel_msg)

                stdout.buffer.write(time_array)
                stdout.buffer.write(b",")
                stdout.buffer.write(heading_array)
                stdout.buffer.write(b",")
                stdout.buffer.write(RWspeed_array)
                #stdout.buffer.write(b",")
                #stdout.buffer.write(latAccel_array)

                wait(100)
               
            # Stop driving motors
            RW_Drive.stop()
            FW_Drive.stop()
            Steer_Motor.stop()

            stopTime = time_apx
            while dataOn.time() < stopTime*1000 + 2000:
                time_apx = (dataOn.time()-startTime)/1000
                time_msg = b"{:.2f}".format(time_apx)
                time_array = bytearray(time_msg)

                heading = hub.imu.heading()
                heading_msg = b"{:.1f}".format(heading)
                heading_array = bytearray(heading_msg)

                RWspeed_dps = RW_Drive.speed(100)
                RWspeed_msg = b"{:.0f}".format(RWspeed_dps)
                RWspeed_array = bytearray(RWspeed_msg)

                #latAccel = hub.imu.acceleration(Axis.Y)
                #latAccel_msg = b"{:.0f}".format(latAccel)
                #latAccel_array = bytearray(latAccel_msg)

                stdout.buffer.write(time_array)
                stdout.buffer.write(b",")
                stdout.buffer.write(heading_array)
                stdout.buffer.write(b",")
                stdout.buffer.write(RWspeed_array)
                #stdout.buffer.write(b",")
                #stdout.buffer.write(latAccel_array)

                wait(100)
        else:
            stdout.buffer.write(b"bye")
            break