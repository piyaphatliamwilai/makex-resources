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

# Define the wheels as encoder motors
left_forward_wheel = encoder_motor_class("M3", "INDEX1")
right_forward_wheel = encoder_motor_class("M5", "INDEX1")
left_back_wheel = encoder_motor_class("M1", "INDEX1")
right_back_wheel = encoder_motor_class("M6", "INDEX1")

def control_motor(lf, lb, rf, rb):
  left_forward_wheel.set_power(lf)
  right_forward_wheel.set_power(lb)
  left_back_wheel.set_power(rf)
  right_back_wheel.set_power(rb)

# ยังไม่มีหมุน โปรดเพิ่มเองนะครับ :)
def control_movement():
    if math.fabs(gamepad.get_joystick("Lx")) > 20 or math.fabs(gamepad.get_joystick("Ly")) > 20:
      rad = math.atan2(gamepad.get_joystick("Ly"), -(gamepad.get_joystick("Lx")))
      sine = math.sin(rad - 1/4 * math.pi) * 140
      cosine = math.sin(rad + 1/4 * math.pi) * 140
      power_factor = math.fabs(gamepad.get_joystick("Lx")) / 100
      if math.fabs(gamepad.get_joystick("Ly")) > math.fabs(gamepad.get_joystick("Lx")):
          power_factor = math.fabs(gamepad.get_joystick("Ly")) / 100
      sine *= power_factor
      cosine *= power_factor
      control_motor(cosine, sine, sine, cosine)
    else:
      control_motor(0, 0, 0, 0)
