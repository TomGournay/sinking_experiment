import bluerobotics_navigator as navigator
from smbus2 import SMBus


class NavigatorSensors:
    def __init__(self):
        with SMBus(1) as bus:
            bmp390_id = bus.read_byte_data(0x76, 0x00)
            bmp280_id = bus.read_byte_data(0x76, 0xD0)

        if bmp390_id == 0x60:
            version = navigator.NavigatorVersion.Version2
            print("Navigator V2 détectée — BMP390")
        elif bmp280_id == 0x58:
            version = navigator.NavigatorVersion.Version1
            print("Navigator V1 détectée — BMP280")
        else:
            raise RuntimeError(
                "Baromètre Navigator non reconnu : "
                f"0x00={bmp390_id:#04x}, 0xD0={bmp280_id:#04x}"
            )

        navigator.set_navigator_version(version)
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
