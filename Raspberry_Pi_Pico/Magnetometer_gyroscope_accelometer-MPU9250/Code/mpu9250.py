"""
MicroPython I2C driver for MPU9250 9-axis motion tracking device
"""

# pylint: disable=import-error
from micropython import const
from mpu6500 import MPU6500
from ak8963 import AK8963
from mpu6500 import ACCEL_FS_SEL_8G, GYRO_FS_SEL_1000DPS
# pylint: enable=import-error

__version__ = "0.3.0"

# Used for enabling and disabling the I2C bypass access
_INT_PIN_CFG = const(0x37)
_I2C_BYPASS_MASK = const(0b00000010)
_I2C_BYPASS_EN = const(0b00000010)
_I2C_BYPASS_DIS = const(0b00000000)

class MPU9250:
    def __init__(self, i2c, accel_fs=ACCEL_FS_SEL_8G, gyro_fs=GYRO_FS_SEL_1000DPS, mpu6500=None, ak8963=None):
        if mpu6500 is None:
            self.mpu6500 = MPU6500(i2c, accel_fs=accel_fs, gyro_fs=gyro_fs)  #  Pass settings to MPU6500
        else:
            self.mpu6500 = mpu6500

        # Enable I2C bypass for AK8963
        char = self.mpu6500._register_char(_INT_PIN_CFG)
        char &= ~_I2C_BYPASS_MASK  # Clear I2C bits
        char |= _I2C_BYPASS_EN
        self.mpu6500._register_char(_INT_PIN_CFG, char)

        if ak8963 is None:
            self.ak8963 = AK8963(i2c)
        else:
            self.ak8963 = ak8963

    @property
    def acceleration(self):
        """
        Acceleration measured by the sensor. By default will return a
        3-tuple of X, Y, Z axis values in m/s^2 as floats. To get values in g
        pass `accel_fs=SF_G` parameter to the MPU6500 constructor.
        """
        return self.mpu6500.acceleration

    @property
    def gyro(self):
        """
        Gyro measured by the sensor. By default will return a 3-tuple of
        X, Y, Z axis values in rad/s as floats. To get values in deg/s pass
        `gyro_sf=SF_DEG_S` parameter to the MPU6500 constructor.
        """
        return self.mpu6500.gyro

    @property
    def temperature(self):
        """
        Die temperature in celcius as a float.
        """
        return self.mpu6500.temperature

    @property
    def magnetic(self):
        """
        X, Y, Z axis micro-Tesla (uT) as floats.
        """
        return self.ak8963.magnetic

    @property
    def whoami(self):
        return self.mpu6500.whoami

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        pass