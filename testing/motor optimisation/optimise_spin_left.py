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

def oppositeSigns(x, y):
    return (y >= 0) if (x < 0) else (y < 0)

# Function to spin an angle in degrees
def search(TB, mpu, max_power, logger):
    """Search for optimal balance between fixed motor1 and optimising motor2.

    Args:
        delta (float): angle to spin in degrees.
        target (float): angle to spin to in degrees.
        TB (ThunderBorg): ThunderBorg object.
        mpu (MPU6050): MPU6050 object.
        max_power (float): maximum power to use.
    """

    logger.debug("------")

    drive_left = -1.0
    drive_right = +1.0
    
    mpu.orientation_flag = True

    motor1_fixed_power = 0.8
    motor2_optimal_power = 0.7

    motor1 = TB.SetMotor2
    motor2 = TB.SetMotor1
    
    # poll every <sampling> seconds, fine tune to minimise overshooting target rotation
    sampling = 0.08

    best_total_rotation, best_motor2_optimal_power = None, None

    hillclimber_step = 0.05

    for _ in range(5): # search for 10 steps
        
        # set motor1 as fixed
        motor1(drive_right * (max_power * motor1_fixed_power))
        
        # set motor2 with optimising value
        motor2(drive_left * (max_power * motor2_optimal_power))

        total_rotation = 0
        mpu.orientation = 0

        delta = 90

        while 1:
            abs_z = mpu.abs_z
            sample = abs_z * sampling

            # increment degree rotation by current rotation velocity, divided by sampling time
            total_rotation += sample

            if total_rotation >= delta:
                break  # exit once achieved target rotation
            # if predicted to exceed during sleep, sleep for predicted time to target, then exit
            elif (total_rotation + sample) >= delta:
                # total degrees left in rotation (degress-total_rotation) divided by abs(z)\
                # (positive rotational velocity) gives time to sleep (in seconds) before reaching target
                sleep = (delta - total_rotation) / abs_z

                logging.debug(
                    "Assuming constant rotation of Z:%.2f, sleeping for %.2f seconds to rotate %.2f degrees"
                    % (abs_z, sleep, (delta - total_rotation))
                )
                time.sleep(sleep)

                # NOTE: this will set total rotation to target, which is only correct \
                # assuming rotation halts immediately and rotational velocity remains constant

                # in non-demo system current orientation should be independently \
                # tracked, not adjusted using this approximation

                total_rotation += abs_z * sleep  # update final rotation for tracking
                break

            time.sleep(sampling)

        total_rotation = mpu.orientation

        logger.debug(f"Balance: {motor1_fixed_power} : {motor2_optimal_power}, Score: {total_rotation} degrees")

        # Turn the motors off
        TB.MotorsOff()

        time.sleep(2)

        if best_total_rotation and oppositeSigns(total_rotation, best_total_rotation):
            hillclimber_step /= 2   # half step, slow descent
            hillclimber_step *= -1  # swap descent direction

        if not best_total_rotation: # if no best, assign initial as best
            best_total_rotation = total_rotation
            best_motor2_optimal_power = motor2_optimal_power

        elif abs(total_rotation - 90) < abs(best_total_rotation - 90):  # if new is closer to 90 than best
            best_total_rotation = total_rotation
            best_motor2_optimal_power = motor2_optimal_power

        motor2_optimal_power += hillclimber_step    # adjust by descent step

        logger.info(f"Current optimal: {best_motor2_optimal_power}")
    
    mpu.orientation_flag = False


def setup_logger(logger_name, log_file, level=logging.DEBUG):
    l = logging.getLogger(logger_name)
    formatter = logging.Formatter('%(asctime)s : %(message)s')  # timestamp logs
    fileHandler = logging.FileHandler(log_file, mode='a')   # append all logs of type to this file
    fileHandler.setFormatter(formatter)
    streamHandler = logging.StreamHandler()
    streamHandler.setFormatter(formatter)

    l.setLevel(level)
    l.addHandler(fileHandler)
    l.addHandler(streamHandler)


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

    setup_logger('mpu6050', r'mpu6050.log')
    setup_logger('velocity', r'velocity.log')

    mpu6050_log = logging.getLogger('mpu6050')
    velocity_log = logging.getLogger('velocity')

    mpu = MPU6050(rotation_log=mpu6050_log, velocity_log=velocity_log)
    mpu.setName("MPU6050")
    mpu.start()

    setup_logger('calibration', r'calibration_left.log')

    calibration_log = logging.getLogger('calibration')

    search(TB, mpu, max_power, calibration_log)
