# create a singleton sensor class to house gyroscope instance and provide data as thread
import logging
import math
import time
from threading import Thread

import adafruit_mpu6050
import board


class MPU6050(Thread):
    def __init__(self, rotation_log=None, velocity_log=None):
        Thread.__init__(self)

        # initialise gyroscope board
        i2c = board.I2C()  # uses board.SCL and board.SDA
        self.mpu = adafruit_mpu6050.MPU6050(i2c)

        self.gyro = None
        self.acceleration = None

        self.poll = 0.05  # poll every <self.poll> seconds

        self.abs_z = 0     # preprocessing to save duplicate instructions
        self.z = 0
        self.orientation = 0
        self.orientation_flag = False

        self.rotation_log = rotation_log
        self.velocity_log = velocity_log

    def run(self):
        """ Update loop to poll MPU sensor for gyro and accelerometer data.
        
        """
        
        while True:
            self.gyro = self.mpu.gyro
            self.acceleration = self.mpu.acceleration
            
            if self.orientation_flag:
                self.gyroscopic()

            time.sleep(self.poll)

    def gyroscopic(self):
        """ Update internal orientation, for calculating future rotations.
        
        """

        if self.rotation_log:
            self.rotation_log.debug(self.orientation)
        
        x, y, z = self.mpu.gyro
        self.abs_z = abs(math.degrees(z))
        self.z = math.degrees(z)
        
        self.orientation += -self.z * self.poll

        if self.orientation >= 360:
            self.orientation -= 360
        elif self.orientation < 0:
            self.orientation += 355

if __name__ == "__main__":
    # enable debug logging
    logging.basicConfig(filename="logging_mpu6050", filemode="a", format='%(asctime)s - %(message)s', level=logging.DEBUG)

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

    setup_logger('mpu6050', r'mpu6050.log')
    setup_logger('velocity', r'velocity.log')

    mpu6050_log = logging.getLogger('mpu6050')
    velocity_log = logging.getLogger('velocity')

    # initialise mpu6050 thread
    mpu = MPU6050(rotation_log=mpu6050_log, velocity_log=velocity_log)
    mpu.setName("MPU6050")
    mpu.start()

    mpu.orientation_flag = True

    try:
        while True:
            logging.debug(f"orientation: {mpu.orientation}, z: {mpu.z}")
            #logging.debug(f"accelerometer: {mpu.acceleration}\ngyro: {mpu.gyro}")
    except KeyboardInterrupt:
        pass

    mpu.orientation_flag = False
    mpu.join()
