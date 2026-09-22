import bluerobotics_navigator as navigator
import math
import time
import json
import os
import threading


from INSLIB import Navigator as FiltreAttitude, Config
from smbus2 import SMBus
from pathlib import Path
from sensors.frame_transform import rotation_matrix, rotate_vector
from sensors.calibration.gyro import GyroCalibration
from sensors.calibration.accel_mag import AccelMagCalibration

_NAVIGATOR_LOCK = threading.Lock()


def read_navigator(reader, *args):
    """Serialize hardware access across monitoring, recording and calibration."""
    with _NAVIGATOR_LOCK:
        return reader(*args)


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

        config_path = (
                Path(__file__).resolve().parents[1] / "imu_mount.json"
                )
        config = json.loads(config_path.read_text(encoding="utf-8"))

        self.mount_angles = tuple(
            math.radians(float(config[key]))
            for key in ("roll_deg", "pitch_deg", "yaw_deg")
        )

        if not all(math.isfinite(a) for a in self.mount_angles):
            raise ValueError("Les angles de montage doivent être finis")

        self.sensor_to_robot = rotation_matrix(self.mount_angles)

        self.filtre = None
        self.precedent_ns = None

        calibration_path = (
                Path(__file__).resolve().parents[1] / "gyro_calibration.json"
        )

        self.gyro_calibration = GyroCalibration(calibration_path)
        self.accel_mag_calibration = AccelMagCalibration(
            calibration_path.with_name('accel_mag_calibration.json'))

    def read_battery_voltage(self):
        """POWER pin 4 -> ADC3; the Blue Robotics PSM divides voltage by 11."""
        multiplier = float(os.environ.get("BATTERY_VOLTAGE_MULTIPLIER", "11.0"))
        if not math.isfinite(multiplier) or multiplier <= 0:
            raise ValueError("BATTERY_VOLTAGE_MULTIPLIER doit être positif et fini")
        sense = float(read_navigator(navigator.read_adc, navigator.AdcChannel.Ch3))
        if not math.isfinite(sense) or not 0 <= sense <= 3.3:
            raise ValueError("Tension POWER hors plage (0–3,3 V)")
        voltage = sense * multiplier
        if not math.isfinite(voltage):
            raise ValueError("Tension batterie invalide")
        return voltage

    def read_battery_current(self):
        """POWER pin 3 -> ADC2; PSM current = (sense volts - 0.330) * 37.8788."""
        multiplier = float(os.environ.get("BATTERY_CURRENT_MULTIPLIER", "37.8788"))
        offset = float(os.environ.get("BATTERY_CURRENT_OFFSET", "0.330"))
        if not math.isfinite(multiplier) or multiplier <= 0:
            raise ValueError("BATTERY_CURRENT_MULTIPLIER doit être positif et fini")
        if not math.isfinite(offset) or not 0 <= offset <= 3.3:
            raise ValueError("BATTERY_CURRENT_OFFSET doit être entre 0 et 3,3 V")
        sense = float(read_navigator(navigator.read_adc, navigator.AdcChannel.Ch2))
        if not math.isfinite(sense) or not 0 <= sense <= 3.3:
            raise ValueError("Signal courant POWER hors plage (0–3,3 V)")
        current = (sense - offset) * multiplier
        if not math.isfinite(current):
            raise ValueError("Courant batterie invalide")
        return current

    def calibration_summary(self):
        gyro = self.gyro_calibration
        combined = self.accel_mag_calibration
        return {
            "automatic_on_startup": True,
            "saved_available": gyro.path.is_file() or combined.path.is_file(),
            "gyro": {
                "active": bool(gyro.data),
                "calibrated_at": gyro.data.get("calibrated_at"),
                "bias": list(gyro.bias),
                "unit": "rad/s",
            },
            **{
                key: {
                    "active": key in combined.data,
                    "calibrated_at": combined.data.get(key, {}).get(
                        "calibrated_at", combined.data.get("calibrated_at")),
                    "bias": combined.data.get(key, {}).get("bias", [0, 0, 0]),
                    "matrix": combined.data.get(key, {}).get("matrix"),
                    "unit": unit,
                }
                for key, unit in (("acc", "m/s²"), ("mag", "µT"))
            },
        }

    def use_saved_calibration(self):
        """Validate all available files before replacing any active correction."""
        gyro_path = self.gyro_calibration.path
        combined_path = self.accel_mag_calibration.path
        if not gyro_path.is_file() and not combined_path.is_file():
            raise FileNotFoundError("Aucune calibration enregistrée disponible")
        gyro = (GyroCalibration(gyro_path) if gyro_path.is_file()
                else self.gyro_calibration)
        combined = (AccelMagCalibration(combined_path) if combined_path.is_file()
                    else self.accel_mag_calibration)
        self.reset_filtre()
        self.gyro_calibration = gyro
        self.accel_mag_calibration = combined
        return self.calibration_summary()

    def reset_filtre(self):
        if self.filtre is not None:
            self.filtre.close()

        self.filtre = FiltreAttitude(Config(auto_init=True))
        self.precedent_ns = None

    def calibrate_gyro(self):
        result = self.gyro_calibration.calibrate(
            lambda: read_navigator(navigator.read_gyro),
            lambda: read_navigator(navigator.read_accel),
        )
        self.reset_filtre()
        return result

    def calibrate_accel_mag(self, report, finish, cancel, gravity=9.80665, field_ut=0.0,
                            estimate_misalignment=False):
        def vector(v):
            return tuple(float(getattr(v, axis)) for axis in "xyz")

        def read():
            acc = vector(read_navigator(navigator.read_accel))
            stamp = time.monotonic_ns() // 1000
            gyr = vector(read_navigator(navigator.read_gyro))
            mag = vector(read_navigator(navigator.read_mag))
            mag_stamp = time.monotonic_ns() // 1000
            return stamp, acc, gyr, mag_stamp, mag

        return self.accel_mag_calibration.calibrate(
            read, report, finish, cancel, gravity, field_ut, estimate_misalignment)

    def read_imu(self):

        if self.filtre is None:
            self.reset_filtre()

        accel = read_navigator(navigator.read_accel)
        gyro = read_navigator(navigator.read_gyro)
        mag = read_navigator(navigator.read_mag)

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

        acc_robot = rotate_vector(
            self.accel_mag_calibration.correct('acc', (accel.x, accel.y, accel.z)),
            self.sensor_to_robot,
        )

        gyro_corrected = self.gyro_calibration.correct(
                (gyro.x, gyro.y, gyro.z)
        )

        gyro_robot = rotate_vector(
            gyro_corrected,
            self.sensor_to_robot,
        )

        mag_robot = rotate_vector(
            self.accel_mag_calibration.correct('mag', (mag.x, mag.y, mag.z)),
            self.sensor_to_robot,
        )

        # Conserver également les mesures dans le repère du robot.
        for sensor, vector in (
            ("acc", acc_robot),
            ("gyro", gyro_robot),
            ("mag", mag_robot),
        ):
            for axis, value in zip("xyz", vector):
                donnees[f"{axis}{sensor}_robot"] = value

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
            acc=acc_robot,
            gyr=gyro_robot,
        )

        self.filtre.mag(
            mag_robot,
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
