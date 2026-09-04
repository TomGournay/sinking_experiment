import bluerobotics_navigator as navigator


class NavigatorSensors:

    def __init__(self):
        navigator.init()

    def read_imu(self):

        accel = navigator.read_accel()
        gyro = navigator.read_gyro()
        mag = navigator.read_mag()

        return {
            "xacc": accel.x,
            "yacc": accel.y,
            "zacc": accel.z,

            "xgyro": gyro.x,
            "ygyro": gyro.y,
            "zgyro": gyro.z,

            "xmag": mag.x,
            "ymag": mag.y,
            "zmag": mag.z,
        }
