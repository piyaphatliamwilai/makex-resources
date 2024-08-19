"""

  ▄████ ▓█████  ███▄ ▄███▓ ██▓ ███▄    █  ██▓
 ██▒ ▀█▒▓█   ▀ ▓██▒▀█▀ ██▒▓██▒ ██ ▀█   █ ▓██▒
▒██░▄▄▄░▒███   ▓██    ▓██░▒██▒▓██  ▀█ ██▒▒██▒
░▓█  ██▓▒▓█  ▄ ▒██    ▒██ ░██░▓██▒  ▐▌██▒░██░
░▒▓███▀▒░▒████▒▒██▒   ░██▒░██░▒██░   ▓██░░██░
 ░▒   ▒ ░░ ▒░ ░░ ▒░   ░  ░░▓  ░ ▒░   ▒ ▒ ░▓  
  ░   ░  ░ ░  ░░  ░      ░ ▒ ░░ ░░   ░ ▒░ ▒ ░
░ ░   ░    ░   ░      ░    ▒ ░   ░   ░ ░  ▒ ░
      ░    ░  ░       ░    ░           ░  ░  

"""

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
left_back_wheel = encoder_motor_class("M1", "INDEX1")
right_back_wheel = encoder_motor_class("M6", "INDEX1")

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
        left_forward_wheel.set_power(left_forward * 0.8)  # LEFT FORWARD
        right_back_wheel.set_power(-right_back)  # RIGHT BACK  
        right_forward_wheel.set_power(-(right_forward))      # RIGHT FORWARD
        left_back_wheel.set_power(left_back * 0.8)             # LEFT BACK
        
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
        power_expand_board.set_power(self.bl_port, 85)
        
    # Method to turn off the brushless motor
    def off(self) -> None:
        power_expand_board.stop(self.bl_port)
        
class runtime:
    # Define control mode
    CTRL_MODE = 0
    
    # Robot state
    ENABLED = True
    
    # Method to control movement based on joystick input
    def control_movement():
        if math.fabs(gamepad.get_joystick("Lx")) > 20 or math.fabs(gamepad.get_joystick("Ly")) > 20 or math.fabs(gamepad.get_joystick("Rx")) > 20:
            if math.fabs(gamepad.get_joystick("Lx")) > 20 or math.fabs(gamepad.get_joystick("Ly")) > 20:
                rad = math.atan2(gamepad.get_joystick("Ly"), -(gamepad.get_joystick("Lx")))
                sine = math.sin(rad - 1/4 * math.pi) * 140
                cosine = math.sin(rad + 1/4 * math.pi) * 140
                power_factor = math.fabs(gamepad.get_joystick("Lx")) / 100
                if math.fabs(gamepad.get_joystick("Ly")) > math.fabs(gamepad.get_joystick("Lx")):
                    power_factor = math.fabs(gamepad.get_joystick("Ly")) / 100
                sine *= power_factor
                cosine *= power_factor
                drivetrain.control_motor(cosine, sine, sine, cosine)
            elif math.fabs(gamepad.get_joystick("Rx")) > 20:
                drivetrain.set_velocity(math.fabs(gamepad.get_joystick("Rx")) * 0.6)
                if gamepad.get_joystick("Rx") > 0:
                    drivetrain.turn(RIGHT)
                else:
                    drivetrain.turn(LEFT)
        else:
            drivetrain.stop()

    def freefire():
        entrance_feed.off()
        feeder.off()
        if runtime.CTRL_MODE == 0:
            runtime.CTRL_MODE = 1
        else:
            runtime.CTRL_MODE = 0
        time.sleep(0.9)

class automatic_stage:
    mode = "RIGHT"
    
    def right_auto():
        # Turn left 45 degree
        drivetrain.set_velocity(50)
        drivetrain.turn(LEFT)
        time.sleep(1.0)
        drivetrain.control_motor(0, 0, 0, 0)
        # Move forward
        drivetrain.drive(FORWARD)
        time.sleep(1.3)
        drivetrain.control_motor(0,0,0,0)
        time.sleep(0.1)
        # Turn on feeder
        entrance_feed.on()
        feeder.on()
        # Move to get more discs
        drivetrain.drive(FORWARD)
        time.sleep(0.4)
        drivetrain.control_motor(0,0,0,0)
        # time.sleep(2)
        # Turn off feeder
        entrance_feed.off()
        feeder.off()

    def left_auto():
        pass

    def run():
        if automatic_stage.mode == "LEFT":
            automatic_stage.left_auto()
        else:
            automatic_stage.right_auto()


class shoot_mode:

    # Method to control various robot functions based on button inputs
    def control_button():
        if gamepad.is_key_pressed("N1"):
            entrance_feed.set_reverse(False)
            feeder.set_reverse(False)
            entrance_feed.on()
            feeder.on()
        elif gamepad.is_key_pressed("L2"):
            entrance_feed.set_reverse(True)
            feeder.set_reverse(True)
            entrance_feed.on()
            feeder.on()
        if gamepad.is_key_pressed("Up"):
            shooter_angle.set_reverse(True)
            shooter_angle.on()
        elif gamepad.is_key_pressed("Down"):
            shooter_angle.set_reverse(False)
            shooter_angle.on()
        else:
            shooter_angle.off()
        if gamepad.is_key_pressed("L1"):
            entrance_feed.off()
            feeder.off()
        if gamepad.is_key_pressed("N2"):
            conveyer.set_reverse(False)
            conveyer.on()
        elif gamepad.is_key_pressed("N3"):
            conveyer.set_reverse(True)
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
        if gamepad.is_key_pressed("N1"):
            gripper.set_reverse(True)
            gripper.on()
        elif gamepad.is_key_pressed("N4"):
            gripper.set_reverse(False)
            gripper.on()
        else:
            gripper.off()
        

# Instantiate DC motors
lift = dc_motor("DC2")
gripper = dc_motor("DC1")
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
        automatic_stage.run()
        while power_manage_module.is_auto_mode():
            pass
    else:
        if gamepad.is_key_pressed("L2") and gamepad.is_key_pressed("R2"):
            runtime.freefire()
        else:
            runtime.control_movement()
            if runtime.CTRL_MODE == 0:
                shoot_mode.control_button()
            else:
                gripper_mode.control_button()
