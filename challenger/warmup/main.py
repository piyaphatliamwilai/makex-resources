# Import necessary modules
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

# Define movement types
FORWARD = 1
BACKWARD = 2

# Define turning types
LEFT = 1
RIGHT = 2

# Define the wheels as encoder motors
left_forward_wheel = encoder_motor_class("M3", "INDEX1")
right_forward_wheel = encoder_motor_class("M5", "INDEX1")
left_back_wheel = encoder_motor_class("M6", "INDEX1")
right_back_wheel = encoder_motor_class("M1", "INDEX1")

class drivetrain:
    # Default velocity for the drivetrain
    velocity = 100
    
    # Method to drive the robot forward or backward
    def drive(type):
        if type == FORWARD:
            drivetrain.control_motor(drivetrain.velocity, drivetrain.velocity, drivetrain.velocity, drivetrain.velocity)
        elif type == BACKWARD:
            drivetrain.control_motor(-drivetrain.velocity, -drivetrain.velocity, -drivetrain.velocity, -drivetrain.velocity)
            
    # Method to turn the robot left or right
    def turn(type):
        if type == LEFT:
            drivetrain.control_motor(drivetrain.velocity, drivetrain.velocity, -drivetrain.velocity, -drivetrain.velocity)
        elif type == RIGHT:
            drivetrain.control_motor(-drivetrain.velocity, -drivetrain.velocity, drivetrain.velocity, drivetrain.velocity)
         
    # Method to slide the robot left or right
    def slide(type):
        if type == LEFT: 
            drivetrain.control_motor(drivetrain.velocity * 0.9, -drivetrain.velocity, -drivetrain.velocity, drivetrain.velocity * 0.9 )
        elif type == RIGHT:
            drivetrain.control_motor(-drivetrain.velocity, drivetrain.velocity * 0.9, drivetrain.velocity * 0.9, -drivetrain.velocity)
               
    # Method to set the velocity of the drivetrain
    def set_velocity(speed):
        drivetrain.velocity = speed
        
    # Method to stop the drivetrain
    def stop():
        drivetrain.control_motor(0, 0, 0, 0)
               
    # Method to control the motors of the drivetrain
    def control_motor(left_forward: int, left_back: int, right_forward: int, right_back: int):
        left_forward_wheel.set_power(left_forward)  # LEFT FORWARD
        right_back_wheel.set_power(-(right_back))  # RIGHT BACK  
        right_forward_wheel.set_power(-(right_forward))      # RIGHT FORWARD
        left_back_wheel.set_power(left_back)             # LEFT BACK
        
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
        power_expand_board.set_power(self.bl_port, 100)
        
    # Method to turn off the brushless motor
    def off(self) -> None:
        power_expand_board.stop(self.bl_port)
        
class runtime:
    # Define control mode
    CONTROL_MODE = "shoot"
    
    # Robot state
    ENABLED = True
    
    # Method to control movement based on joystick input
    def control_movement():
        if math.fabs(gamepad.get_joystick("Lx")) > 20 or math.fabs(gamepad.get_joystick("Ly")) > 20 or math.fabs(gamepad.get_joystick("Rx")) > 20:
            if math.fabs(gamepad.get_joystick("Lx")) > 20:
                drivetrain.set_velocity(math.fabs(gamepad.get_joystick("Lx")))
                if gamepad.get_joystick("Lx") > 0:
                    drivetrain.slide(RIGHT)
                else:
                    drivetrain.slide(LEFT)
            elif math.fabs(gamepad.get_joystick("Ly")) > 20:
                drivetrain.set_velocity(math.fabs(gamepad.get_joystick("Ly")))
                if gamepad.get_joystick("Ly") > 0:
                    drivetrain.drive(FORWARD)
                else:
                    drivetrain.drive(BACKWARD)
            elif math.fabs(gamepad.get_joystick("Rx")) > 20:
                drivetrain.set_velocity(math.fabs(gamepad.get_joystick("Rx")) * 0.6)
                if gamepad.get_joystick("Rx") > 0:
                    drivetrain.turn(RIGHT)
                else:
                    drivetrain.turn(LEFT)
        else:
            drivetrain.stop()

    def change_mode():
        if gamepad.is_key_pressed("L2") and gamepad.is_key_pressed("R2"):
            if runtime.CONTROL_MODE == "shoot":
                runtime.CONTROL_MODE = "gripper"
            else:
                runtime.CONTROL_MODE = "shoot"
            time.sleep(0.9)
            
    def turn_on():
        runtime.ENABLED = True
            
    def turn_off_all():
        conveyer.off()
        gripper.off()
        tractor.off()
        entrance_feed.off()
        lift.off()
        feeder.off()
        shooter.off()
        bl_1.off()
        bl_2.off()
        drivetrain.control_motor(0,0,0,0)
        runtime.ENABLED = False

class shoot_mode:

    # Method to control various robot functions based on button inputs
    def control_button():
        if gamepad.is_key_pressed("N1"):
            entrance_feed.set_reverse(False)
            entrance_feed.on()
        if gamepad.is_key_pressed("N4"):
            feeder.set_reverse(True)
            feeder.on()
        if gamepad.is_key_pressed("L1"):
            entrance_feed.off()
            feeder.off()
            conveyer.off()
            gripper.off()
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
        if gamepad.is_key_pressed("R2"):
            bl_1.off()
            bl_2.off()
        if gamepad.is_key_pressed("Up"):
            shooter.set_reverse(True)
            shooter.on()
        elif gamepad.is_key_pressed("Down"):
            shooter.set_reverse(False)
            shooter.on()
        else:
            shooter.off()
        if gamepad.is_key_pressed("L2") and not gamepad.is_key_pressed("R2"):
            feeder.set_reverse(False)
            entrance_feed.set_reverse(False)
            feeder.on()
            entrance_feed.on()

class gripper_mode:

    # Method to control various robot functions based on button inputs
    def control_button():
        if gamepad.is_key_pressed("N1"):
            entrance_feed.set_reverse(False)
            entrance_feed.on()
        if gamepad.is_key_pressed("N4"):
            feeder.set_reverse(True)
            feeder.on()
        if gamepad.is_key_pressed("L1"):
            entrance_feed.off()
            feeder.off()
            conveyer.off()
            gripper.off()
        if gamepad.is_key_pressed("N2"):
            tractor.set_reverse(False)
            tractor.on()
        elif gamepad.is_key_pressed("N3"):
            tractor.set_reverse(True)
            tractor.on()
        else:
            tractor.off()
        if gamepad.is_key_pressed("R1"):
            bl_1.on()
            bl_2.on()
        if gamepad.is_key_pressed("R2"):
            bl_1.off()
            bl_2.off()
        if gamepad.is_key_pressed("Up"):
            gripper.set_reverse(True)
            gripper.on()
        elif gamepad.is_key_pressed("Down"):
            gripper.set_reverse(False)
            gripper.on()
        else:
            gripper.off()
        if gamepad.is_key_pressed("Left"):
            lift.set_reverse(True)
            lift.on()
        elif gamepad.is_key_pressed("Right"):
            lift.set_reverse(False)
            lift.on()
        else:
            lift.off()
        if gamepad.is_key_pressed("L2") and not gamepad.is_key_pressed("R2"):
            feeder.set_reverse(False)
            entrance_feed.set_reverse(False)
            feeder.on()
            entrance_feed.on()

# Instantiate DC motors
conveyer = dc_motor("DC1")
gripper = dc_motor("DC2")
tractor = dc_motor("DC6")
feeder = dc_motor("DC4")
entrance_feed = dc_motor("DC5")
shooter = dc_motor("DC7")
lift = dc_motor("DC8")
bl_1 = brushless_motor("BL1")
bl_2 = brushless_motor("BL2")
# shooter = smartservo_class("M3", "1") # only for angles

while True:
    if power_manage_module.is_auto_mode():
        pass
    else:
        runtime.control_movement()
        if gamepad.is_key_pressed("L2") and gamepad.is_key_pressed("R2"):
            runtime.change_mode()
        else:
            if gamepad.is_key_pressed("≡"):
                runtime.turn_on()
            elif gamepad.is_key_pressed("+"):
                runtime.turn_off_all()
            if runtime.ENABLED == True:
                if runtime.CONTROL_MODE == "shoot":
                    shoot_mode.control_button()
                else:
                    gripper_mode.control_button()
