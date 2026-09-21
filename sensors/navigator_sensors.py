import bluerobotics_navigator as navigator
import math
import time

from INSLIB import Navigator as FiltreAttitude, Config
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

        self.filtre = None
        self.precedent_ns = None

    def reset_filtre(self):
        if self.filtre is not None:
            self.filtre.close()

        self.filtre = FiltreAttitude(Config(auto_init=True))
        self.precedent_ns = None

    def read_imu(self):

        if self.filtre is None:
            self.reset_filtre()

        accel = navigator.read_accel()
        gyro = navigator.read_gyro()
        mag = navigator.read_mag()

        instant_ns = time.monotonic_ns()

        donnees = {
            "xacc": accel.x,
            "yacc": accel.y,
            "zacc": accel.z,

            "xgyro": gyro.x,
            "ygyro": gyro.y,
            "zgyro": gyro.z,

            "xmag": mag.x,
            "ymag": mag.y,
            "zmag": mag.z,

            "roll_deg":     None,
            "pitch_deg":    None,
            "yaw_deg":      None,

            "attitude_ok":  False,
            "filter_mode":  "INITIALIZING",
        }

        if self.precedent_ns is None:
            self.precedent_ns = instant_ns
            return donnees

        dt = (instant_ns - self.precedent_ns) / 1e9
        self.precedent_ns = instant_ns

        # Repartir de zéro après une interruption de lecture.
        if dt <= 0 or dt > 0.25:
            self.reset_filtre()
            self.precedent_ns = instant_ns
            return donnees
            
        self.filtre.imu(
            instant_ns // 1000,
            dt,
            acc=(accel.x, accel.y, accel.z),
            gyr=(gyro.x, gyro.y, gyro.z),
        )

        self.filtre.mag(
            (mag.x, mag.y, mag.z),
            (0.0, 0.0, 0.0),
        )

        self.filtre.update()
        resultat = self.filtre.solution()

        donnees["filter_mode"] = resultat.mode

        if resultat.attitude_ok and all(
            math.isfinite(angle)
            for angle in (resultat.roll, resultat.pitch, resultat.yaw)
        ):
            donnees.update({
                "roll_deg": math.degrees(resultat.roll),
                "pitch_deg": math.degrees(resultat.pitch),
                "yaw_deg": math.degrees(resultat.yaw) % 360.0,
                "attitude_ok": True,
            })

        return donnees
