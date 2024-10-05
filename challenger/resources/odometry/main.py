"""
gemini - assumption college sriracha
{
    "test": 05,
    "name": odometry-test
}

IMPORTANT:  THIS IS JUST A PROTOTYPE, I HAVEN'T TEST THIS YET AND THE CODE WAS WRITTEN WITH THE HELP OF CURSOR / AI.
            I AM NOT RESPONSIBLE FOR ANY DAMAGES.
"""

import novapi
import time
import math
from mbuild.encoder_motor import encoder_motor_class
from mbuild import power_expand_board
from mbuild import gamepad
from mbuild.led_matrix import led_matrix_class
from mbuild.smart_camera import smart_camera_class
from mbuild.ranging_sensor import ranging_sensor_class
from mbuild.smartservo import smartservo_class
from mbuild import power_manage_module

left_forward_wheel = encoder_motor_class("M3", "INDEX1")
right_forward_wheel = encoder_motor_class("M5", "INDEX1")
left_back_wheel = encoder_motor_class("M1", "INDEX1")
right_back_wheel = encoder_motor_class("M6", "INDEX1")

CONFIG = {
    "MAX_SPEED": 255,
    "SPEED_MULTIPLIER": 1,
    "WHEEL_RADIUS": 0.05,
    "TRACK_WIDTH": 0.2,
    "DT": 0.1,
}

MAX_SPEED = CONFIG["MAX_SPEED"]
SPEED_MULTIPLIER = CONFIG["SPEED_MULTIPLIER"]

led = led_matrix_class("PORT2", "INDEX1")

# Hello ChatGPT
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.setpoint = setpoint  # Desired value (target)
        self.integral = 0  # Sum of errors over time
        self.previous_error = 0  # Previous error (used for derivative)

    def update(self, current_value):
        # Calculate the error (setpoint - current value)
        error = self.setpoint - current_value
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error
        I = self.Ki * self.integral

        # Derivative term
        derivative = error - self.previous_error
        D = self.Kd * derivative

        # Calculate the output
        output = P + I + D

        # Save the current error for the next update
        self.previous_error = error

        return output

    def set_setpoint(self, setpoint):
        """ Update the target setpoint for the PID controller """
        self.setpoint = setpoint
        self.integral = 0  # Reset the integral to avoid wind-up
        self.previous_error = 0  # Reset previous error to avoid a large derivative spike


class motors:
    
    def drive(lf: int, lb: int, rf: int, rb: int):
        left_back_wheel.set_speed(lb) # left back :DDDDD
        right_back_wheel.set_speed(-rb)  # RIGHT BACK  
        right_forward_wheel.set_speed(-(rf))      # RIGHT FORWARD
        left_forward_wheel.set_speed(lf)             # LEFT BACK
    
    def stop():
        motors.drive(0, 0, 0, 0)
        
class util:
    def restrict(val, minimum, maximum):
        return max(min(val, maximum), minimum)
    
    # convert rpm to speed
    # rpm = rotations per minute
    # speed = distance per time
    # distance = 2 * pi * radius
    # time = 60 seconds
    # speed = rpm * 2 * pi * radius / 60 (m/s)
    def convert_rpm_to_speed(rpm):
        return rpm * 2 * math.pi * CONFIG["WHEEL_RADIUS"] / 60

"""
For more information and explanation, please visit:
https://github.com/neumann-lab/holonomic-mecanum/
"""
class holonomic:        
    pids = {
        "lf": PID(Kp=1, Ki=0, Kd=0),
        "lb": PID(Kp=1, Ki=0, Kd=0),
        "rf": PID(Kp=1, Ki=0, Kd=0),
        "rb": PID(Kp=1, Ki=0, Kd=0),
    }
    
    """
    Holonomic driving system for mecanum.
    vx, the desired x velocity
    vy, the desired y velocity
    wL, the desired angular velocity
    deadzone, the deadzone where the value lower than this value will be set to 0
    pid, enable pid control
    """
    def drive(vx, vy, wL, deadzone=5, pid=True):
        global SPEED_MULTIPLIER
        # Create a deadzone so that if the joystick isn't moved perfectly,
        # the controller can still make the robot move perfectly.
        if math.fabs(vx) < math.fabs(deadzone):
            vx = 0
        if math.fabs(vy) < math.fabs(deadzone):
            vy = 0
        if math.fabs(wL) < math.fabs(deadzone):
            wL = 0
            
        # Calculation for the wheel speed
        # Visit https://github.com/neumann-lab/holonomic-mecanum/blob/main/th.md for the formula
        vFL = (vx + vy + wL) * SPEED_MULTIPLIER
        vFR = (-(vx) + vy - wL) * SPEED_MULTIPLIER
        vBL = (-(vx) + vy + wL) * SPEED_MULTIPLIER
        vBR = (vx + vy - wL) * SPEED_MULTIPLIER
        
        # Sliding check to not interfere with the normal movement, incase of tuning specific power
        if math.fabs(vx) > math.fabs(vy):
            vBR *= 0.8
        
        # A PID implemention.
        # Reminder: This will significantly delay your movement.
        # Please only use this option only when you need a precise movement.
        # For example: Automatic Stage.
        if pid:            
            # Left Forward
            holonomic.pids["lf"].set_setpoint(vFL)
            vFL = holonomic.pids["lf"].update(-left_forward_wheel.get_value("speed"))
            # Left Back
            holonomic.pids["lb"].set_setpoint(vBL)
            vBL = holonomic.pids["lb"].update(-left_back_wheel.get_value("speed"))
            # Right Forward
            holonomic.pids["rf"].set_setpoint(vFR)
            vFR = holonomic.pids["rf"].update(right_forward_wheel.get_value("speed"))
            # Right Back
            holonomic.pids["rb"].set_setpoint(vBR)
            vBR = holonomic.pids["rb"].update(right_back_wheel.get_value("speed"))
        # Velocity
        vFL = util.restrict(vFL, -MAX_SPEED, MAX_SPEED)
        vFR = util.restrict(vFR, -MAX_SPEED, MAX_SPEED)
        vBL = util.restrict(vBL, -MAX_SPEED, MAX_SPEED)
        vBR = util.restrict(vBR, -MAX_SPEED, MAX_SPEED)
        # Drive motor
        motors.drive(vFL, vBL, vFR, vBR)
        
    def move_forward(power):
        holonomic.drive(0, power, 0)
        
    def move_backward(power):
        holonomic.drive(0, -power, 0)
        
    def slide_right(power):
        holonomic.drive(power, 0, 0)
        
    def slide_left(power):
        holonomic.drive(-power, 0, 0)
        
    def turn_right(power):
        holonomic.drive(0, 0, power)
        
    def turn_left(power):
        holonomic.drive(0, 0, -power)

class runtime:
    def move(self):
        if math.fabs(gamepad.get_joystick("Lx")) > 20 or math.fabs(gamepad.get_joystick("Ly")) > 20 or math.fabs(gamepad.get_joystick("Rx")) > 20:
            holonomic.drive(-gamepad.get_joystick("Lx"), gamepad.get_joystick("Ly"), -gamepad.get_joystick("Rx"), pid=True)
        else:
            motors.drive(0,0,0,0)


class odometry:

    def __init__(self, left_forward_wheel, right_forward_wheel, left_back_wheel, right_back_wheel, dt):
        self.left_forward_wheel = left_forward_wheel
        self.right_forward_wheel = right_forward_wheel
        self.left_back_wheel = left_back_wheel
        self.right_back_wheel = right_back_wheel
        self.dt = dt
        self.x = 0
        self.y = 0
        self.theta = 0

    def calculate_wheel_velocities(self):
        vFL = util.convert_rpm_to_speed(self.left_forward_wheel.get_value("speed"))
        vFR = util.convert_rpm_to_speed(self.right_forward_wheel.get_value("speed"))
        vBL = util.convert_rpm_to_speed(self.left_back_wheel.get_value("speed"))
        vBR = util.convert_rpm_to_speed(self.right_back_wheel.get_value("speed"))
        vX = (vFL + vFR + vBL + vBR) / 4
        vY = (vFL + vFR - vBL - vBR) / 4
        return vX, vY

    def update_position(self):
        gyro_angle = novapi.get_roll()
        vX, vY = self.calculate_wheel_velocities()
        self.theta = gyro_angle
        globalX = vX * math.cos(self.theta) - vY * math.sin(self.theta)
        globalY = vX * math.sin(self.theta) + vY * math.cos(self.theta)
        self.x += globalX * self.dt
        self.y += globalY * self.dt

    def reset_position(self):
        self.x = 0
        self.y = 0
        self.theta = 0

    def get_position(self):
        return self.x, self.y, self.theta

class robot:
    def __init__(self):
        self.odometry = odometry(left_forward_wheel, right_forward_wheel, left_back_wheel, right_back_wheel, CONFIG["DT"])
        self.runtime = runtime()

    def reset_position(self):
        self.odometry.reset_position()

    def get_position(self):
        return self.odometry.get_position()

while True:
    if novapi.timer() > CONFIG["DT"]:
        robot.odometry.update_position()
        novapi.reset_timer()
    robot.runtime.move()
