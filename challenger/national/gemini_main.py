"""
gemini - assumption college sriracha
{
    "mode": challenge
}
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

# Wheels
left_forward_wheel = encoder_motor_class("M3", "INDEX1")
right_forward_wheel = encoder_motor_class("M5", "INDEX1")
left_back_wheel = encoder_motor_class("M1", "INDEX1")
right_back_wheel = encoder_motor_class("M6", "INDEX1")

# Constants
MAX_SPEED = 255
SPEED_MULTIPLIER = 2.1
PID_SPEED_MULTIPLIER = 0.6
BL_POWER = 90

# LED
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
        left_forward_wheel.set_speed(lf)             # LEFT FORWARD
        left_back_wheel.set_speed(lb) # left back :DDDDD
        right_forward_wheel.set_speed(-(rf))      # RIGHT FORWARD
        right_back_wheel.set_speed(-rb)  # RIGHT BACK  
   
    def stop():
        motors.drive(0, 0, 0, 0)
        
class util:
    def restrict(val, minimum, maximum):
        return max(min(val, maximum), minimum)

"""
For more information and explanation, please visit:
https://github.com/neumann-lab/holonomic-mecanum/
"""
class holonomic:    

    pids = {
        "lf": PID(Kp=1, Ki=0, Kd=0),
        "lb": PID(Kp=1, Ki=0, Kd=0),
        "rf": PID(Kp=0.9, Ki=0, Kd=0),
        "rb": PID(Kp=0.9, Ki=0, Kd=0),
    }

    # จูนล้อตรงนี้นะคร้าบ
    tune = {
        "fl": 0.9,
        "fr": 0.7,
        "bl": 0.8,
        "br": 0.9,
    }
    
    """
    Holonomic driving system for mecanum.
    vx, the desired x velocity
    vy, the desired y velocity
    wL, the desired angular velocity
    deadzone, the deadzone where the value lower than this value will be set to 0
    pid, enable pid control
    """
    def drive(vx, vy, wL, deadzone=5, pid=False):
        global SPEED_MULTIPLIER, PID_SPEED_MULTIPLIER
        # Create a deadzone so that if the joystick isn't moved perfectly,
        # the controller can still make the robot move perfectly.
        if math.fabs(vx) < math.fabs(deadzone):
            vx = 0
        if math.fabs(vy) < math.fabs(deadzone):
            vy = 0
        if math.fabs(wL) < math.fabs(deadzone):
            wL = 0

        # Ensure the correct speed multiplier
        multiplier = PID_SPEED_MULTIPLIER if pid else SPEED_MULTIPLIER

        # Calculation for the wheel speed
        # Visit https://github.com/neumann-lab/holonomic-mecanum/blob/main/th.md for the formula
        vFL = ((vx * 1.3) + (vy * 1.2) + wL) * multiplier
        vFR = (-(vx * 1.3) + (vy * 1.2) - wL) * multiplier
        vBL = (-(vx * 1.3) + (vy * 1.2) + wL) * multiplier
        vBR = ((vx * 1.3) + (vy * 1.2) - wL) * multiplier
        
        # Sliding check to not interfere with the normal movement, incase of tuning specific power
        if math.fabs(vx) > math.fabs(vy):
            vFL *= holonomic.tune["fl"] # หน้าซ้าย
            vFL *= holonomic.tune["fr"] # หน้าขวา
            vBL *= holonomic.tune["bl"] # หลังซ้าย
            vBR *= holonomic.tune["br"] # หลังขวา
        
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

# Define movement types
FORWARD = 1
BACKWARD = 2

# Define turning types
LEFT = 1
RIGHT = 2

class automatic:
    
    def move_forward_until(distance, basePower=30, kP=0, kI=0, kD=0):
        pid = PID(Kp=kP, Ki=kI, Kd=kD)
        pid.set_setpoint(distance)
        while back_ranging.get_distance() < distance:
            power = pid.update(back_ranging.get_distance())
            led.show(power, wait=False)            
            holonomic.move_forward(basePower + power)

    def slide_right_until(distance, basePower=30, kP=1, kI=0, kD=0):
        pid = PID(Kp=kP, Ki=kI, Kd=kD)
        pid.set_setpoint(distance)
        while left_ranging.get_distance() < distance:
            power = pid.update(left_ranging.get_distance())
            led.show(left_ranging.get_distance(), wait=False)            
            holonomic.slide_right(basePower + power)
        motors.stop()
            
    def move_backward_until(distance, basePower=30, kP=0, kI=0, kD=0):
        pid = PID(Kp=kP, Ki=kI, Kd=kD)
        pid.set_setpoint(distance)
        while back_ranging.get_distance() > distance:
            led.show(back_ranging.get_distance(), wait=False)            
            power = pid.update(back_ranging.get_distance())
            holonomic.move_backward(basePower + power)

    def move_backward_until_left(distance, basePower=30, kP=0, kI=0, kD=0):
        pid = PID(Kp=kP, Ki=kI, Kd=kD)
        pid.set_setpoint(distance)
        while front_ranging.get_distance() < distance:
            power = pid.update(front_ranging.get_distance())
            led.show(front_ranging.get_distance(), wait=False)            
            holonomic.move_backward(basePower + power)

    def set_zero(mode):
        if mode == RIGHT:
            holonomic.turn_left(50)
            time.sleep(0.9)
            holonomic.move_backward(50)
            time.sleep(1.3)
        else:
            holonomic.turn_left(50)
            time.sleep(1.1)
            holonomic.move_forward(50)
            time.sleep(1.3)
        motors.stop()
        
    def grip_block(pull, release, forward, second, pullOut=True):
        if release:
            gripper.set_reverse(True)
            gripper.on()
            time.sleep(1)
            gripper.off()
        # if not second:
        automatic.slide_right_until(170, 30, 0.4, 0, 0.3)
        # เดินหน้านิดหน่อย
        if forward:
            holonomic.move_forward(30)
            time.sleep(0.3)
            motors.stop()
            time.sleep(0.2) # COOLDOWN (ไม่ต้องแก้)
        # # สไสด์์ซ้ายเลี่ยงแม่น้ำ
        if not second:
            holonomic.slide_left(30)
            time.sleep(0.2)
            motors.stop()
        # ดึง lift ลง
        if pull:
            lift.set_reverse(False)
            lift.on()
            if not second:
                time.sleep(0.5)
            else:
                time.sleep(0.2)
            lift.off()
        # เปิด gripper
        gripper.set_reverse(False)
        gripper.on()
        time.sleep(0.3) # COOLDOWN (ไม่ต้องแก้)
        # ดึง lift ขึ้น
        lift.set_reverse(True)
        lift.on()
        time.sleep(0.5)
        lift.off()
        # สไลด์ซ้ายเอาบล็อคออก
        if pullOut:
            holonomic.slide_left(50)
            time.sleep(1)
            motors.stop()

    def right():
        # สไลด์ซ้าย
        holonomic.slide_left(50)
        time.sleep(0.8)
        motors.stop()
        holonomic.move_backward(30)
        time.sleep(0.8)
        # เดินไปหาแม่น้ำ
        automatic.move_forward_until(155, 30, 1, 0, 0.3)
        motors.stop()
        time.sleep(0.5)
        # set zero
        automatic.set_zero(RIGHT)
        motors.stop()
        time.sleep(0.3)
        automatic.slide_right_until(160, 30, 0.4, 0, 0.3)
        motors.stop()
        # เดินไปหาบล็อค
        automatic.move_forward_until(38, 30, 0.7, 0, 0.3)
        motors.stop()
        time.sleep(0.5)
        # คีบบล็อค
        automatic.grip_block(pull=True, release=False, forward=True, second=False)
        # turn left
        holonomic.turn_left(50)
        time.sleep(0.9)
        motors.stop()
        # release block
        gripper.set_reverse(True)
        gripper.on()
        time.sleep(0.4) # COOLDOWN (ไม่ต้องแก้)
        gripper.off()
        # turn right 90
        holonomic.turn_right(50)
        time.sleep(0.9)
        motors.stop()
        # go back
        automatic.move_backward_until(10, 50, 0.6, 0, 0.3)        
        motors.stop()
        automatic.slide_right_until(165, 30, 0.4, 0, 0.3)
        holonomic.move_backward(50)
        time.sleep(0.6)
        motors.stop()
        holonomic.move_forward(50)
        time.sleep(0.1)
        motors.stop()
        automatic.grip_block(pull=True, release=False, forward=False, second=True)

    def left():
        # สไลด์ขวา
        holonomic.slide_right(50)
        time.sleep(0.8)
        motors.stop()
        holonomic.move_backward(30)
        time.sleep(0.8)
        # เดินไปหาแม่น้ำ
        automatic.move_forward_until(155, 30, 1, 0, 0.3)
        motors.stop()
        time.sleep(0.5)
        # set zero
        automatic.set_zero(LEFT)
        time.sleep(0.3)
        automatic.slide_right_until(160, 30, 0.4, 0, 0.3)
        motors.stop()    
        holonomic.move_forward(50)
        time.sleep(0.5)
        motors.stop()
        # เดินไปหาบล็อค
        automatic.move_backward_until_left(45, 30, 0.7, 0, 0.3) # (จูนระยะ ให้เปลี่ยนที่เลขตัวแรก)
        motors.stop()
        time.sleep(0.5)
        # คีบบล็อค
        automatic.grip_block(pull=True, release=False, forward=False, second=False, pullOut=False)
        holonomic.move_forward(30)
        time.sleep(0.5)
        motors.stop()
        holonomic.turn_left(50)
        time.sleep(1.1)
        motors.stop()

# Ranging sensors
left_ranging = ranging_sensor_class("PORT2", "INDEX1")
back_ranging = ranging_sensor_class("PORT2", "INDEX2")
right_ranging = ranging_sensor_class("PORT2", "INDEX3")
front_ranging = ranging_sensor_class("PORT2", "INDEX4")

        
class dc_motor:
    # Default DC port
    dc_port = "DC1"
    # Default direction (not reversed)
    reverse = False
    
    # Initialize DC motor with a specific port
    def __init__(self, port: str) -> None:
        self.dc_port = port
        
    # Method to set the direction of the motor
    def set_reverse(self, rev: bool) -> None:
        self.reverse = rev
        
    # Method to turn on the DC motor
    def on(self) -> None:
        power = -100 if self.reverse else 100
        power_expand_board.set_power(self.dc_port, power)
        
    # Method to turn off the DC motor

    def off(self) -> None:
        power_expand_board.stop(self.dc_port)
        
class brushless_motor:
    # Default brushless motor port
    bl_port = "BL1"
    
    # Initialize brushless motor with a specific port
    def __init__(self, port: str) -> None:
        self.bl_port = port
        
    # Method to turn on the brushless motor
    def on(self) -> None:
        global BL_POWER
        power_expand_board.set_power(self.bl_port, BL_POWER)
        
    # Method to turn off the brushless motor
    def off(self) -> None:
        power_expand_board.stop(self.bl_port)
        
class runtime:
    # Define control mode
    CTRL_MODE = 0
    
    # Robot state
    ENABLED = True

    def change_mode():
        if novapi.timer() > 0.9:
            entrance_feed.off()
            feeder.off()
            if runtime.CTRL_MODE == 0:
                runtime.CTRL_MODE = 1
            else:
                runtime.CTRL_MODE = 0
            novapi.reset_timer()

    def move():
        if math.fabs(gamepad.get_joystick("Lx")) > 20 or math.fabs(gamepad.get_joystick("Ly")) > 20 or math.fabs(gamepad.get_joystick("Rx")) > 20:
            holonomic.drive(-gamepad.get_joystick("Lx"), gamepad.get_joystick("Ly"), -(1.2 * gamepad.get_joystick("Rx")), pid=False)
        else:
            motors.drive(0,0,0,0)

class shoot_mode:

    # Method to control various robot functions based on button inputs
    def control_button():
        if gamepad.is_key_pressed("N1"):
            entrance_feed.set_reverse(False)
            feeder.set_reverse(False)
            # china.set_reverse(True)
            entrance_feed.on()
            feeder.on()
            # china.on()
        elif gamepad.is_key_pressed("L2"):
            entrance_feed.set_reverse(True)
            feeder.set_reverse(True)
            # china.set_reverse(True)
            entrance_feed.on()
            feeder.on()
            # china.on()
        if gamepad.is_key_pressed("Up"):
            shooter_angle.set_reverse(False)
            shooter_angle.on()
        elif gamepad.is_key_pressed("Down"):
            shooter_angle.set_reverse(True)
            shooter_angle.on()
        else:
            shooter_angle.off()
        if gamepad.is_key_pressed("L1"):
            entrance_feed.off()
            feeder.off()
            # china.off()
        if gamepad.is_key_pressed("N2"):
            conveyer.set_reverse(True)
            conveyer.on()
        elif gamepad.is_key_pressed("N3"):
            conveyer.set_reverse(False)
            conveyer.on()
        else:
            conveyer.off()
        if gamepad.is_key_pressed("R1"):
            bl_1.on()
            bl_2.on()
        elif gamepad.is_key_pressed("R2"):
            bl_1.off()
            bl_2.off()
        if gamepad.is_key_pressed("≡"):
            slot_conveyer.set_reverse(True)
            slot_conveyer.on()
        elif gamepad.is_key_pressed("+"):
            slot_conveyer.set_reverse(False)
            slot_conveyer.on()
        else:
            slot_conveyer.off()
            
class gripper_mode:

    status = "NONE"

    # Method to control various robot functions based on button inputs
    def control_button():
        if gamepad.is_key_pressed("N2"):
            lift.set_reverse(True)
            lift.on()
        elif gamepad.is_key_pressed("N3"):
            lift.set_reverse(False)
            lift.on()
        else:
            lift.off()
        if gamepad.is_key_pressed("N4"):
            gripper.set_reverse(False)
            gripper.on()
            gripper_mode.status = "GRIP"
        elif gamepad.is_key_pressed("N1"):
            gripper.set_reverse(True)
            gripper.on()
            gripper_mode.status = "RELEASE"
        else:
            if gripper_mode.status == "RELEASE":
                gripper.off()
        

# Instantiate DC motors
lift = dc_motor("DC2")
gripper = dc_motor("DC1")
china = dc_motor("DC3")
slot_conveyer = dc_motor("DC8")
conveyer = dc_motor("DC5")
shooter_angle = dc_motor("DC4")
entrance_feed = dc_motor("DC6")
feeder = dc_motor("DC7")
bl_1 = brushless_motor("BL1")
bl_2 = brushless_motor("BL2")
# shooter = smartservo_class("M3", "1") # only for angles

while True:
    if power_manage_module.is_auto_mode():
        automatic.right()
        while power_manage_module.is_auto_mode():
            pass
    else:
        # led.show(novapi.get_pitch(), wait=False)
        if gamepad.is_key_pressed("L2") and gamepad.is_key_pressed("R2"):
            runtime.change_mode()
        else:
            runtime.move()
            led.show(front_ranging.get_distance(), wait=False)
            if runtime.CTRL_MODE == 0:
                # led.show("GRP", wait=False)
                gripper_mode.control_button()
            else:
                # led.show("SHT", wait=False)
                shoot_mode.control_button()
