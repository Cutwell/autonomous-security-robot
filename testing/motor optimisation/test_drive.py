"""
Calculate optimal motor balance for motor pairing.

- Find optimal motor balance using hillclimber method for fixed motor/optimising motor pair.
"""

from mpu6050 import MPU6050

import logging
import math
import sys
import time
import numpy as np

import adafruit_mpu6050
import board
import ThunderBorg3 as ThunderBorg  # conversion for python 3

# Function to spin an angle in degrees
def drive_forward_test(TB, max_power):
    """Search for optimal balance between fixed motor1 and optimising motor2.

    Args:
        delta (float): angle to spin in degrees.
        target (float): angle to spin to in degrees.
        TB (ThunderBorg): ThunderBorg object.
        mpu (MPU6050): MPU6050 object.
        max_power (float): maximum power to use.
    """


    drive_left = +1.0
    drive_right = +1.0
    

    motor1_fixed_power = 0.75
    motor2_optimal_power = 0.75

    motor1 = TB.SetMotor2
    motor2 = TB.SetMotor1

    # set motor1 as fixed
    motor1(drive_right * (max_power * motor1_fixed_power))
    
    # set motor2 with optimising value
    motor2(drive_left * (max_power * motor2_optimal_power))

    time.sleep(4)
    
    # Turn the motors off
    TB.MotorsOff()


if __name__ == "__main__":
    # enable debug logging
    logging.basicConfig(format='%(asctime)s - %(message)s', level=logging.DEBUG)

    # Setup the ThunderBorg
    TB = ThunderBorg.ThunderBorg()
    # TB.i2cAddress = 0x15                  # Uncomment and change the value if you have changed the board address
    TB.Init()
    if not TB.foundChip:
        boards = ThunderBorg.ScanForThunderBorg()
        if len(boards) == 0:
            print("No ThunderBorg found, check you are attached :)")
        else:
            print(
                "No ThunderBorg at address %02X, but we did find boards:"
                % (TB.i2cAddress)
            )
            for board in boards:
                print("    %02X (%d)" % (board, board))
            print(
                "If you need to change the I2C address change the setup line so it is correct, e.g."
            )
            print("TB.i2cAddress = 0x%02X" % (boards[0]))
        sys.exit()
    TB.SetCommsFailsafe(False)  # Disable the communications failsafe

    # Power settings
    VOLTAGE_IN = 9.6  # Total battery voltage to the ThunderBorg

    # NOTE: limiter has lower bound to power motors, ~0.4 experimental lower bound
    LIMITER = 0.6  # utilise only <limiter>% of power, to slow down actions

    VOLTAGE_OUT = (
        12.0 * LIMITER
    )  # Maximum motor voltage, we limit it to 95% to allow the RPi to get uninterrupted power

    # Setup the power limits
    if VOLTAGE_OUT > VOLTAGE_IN:
        max_power = 1.0
    else:
        max_power = VOLTAGE_OUT / float(VOLTAGE_IN)

    drive_forward_test(TB, max_power)
