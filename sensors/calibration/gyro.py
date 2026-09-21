"""Calibration immobile du gyroscope, dans le repère brut de la Navigator.

Les seuils sont des contrôles pratiques de mouvement, pas des spécifications
mesurées du capteur. Ils ne détectent pas toute rotation lente et constante.
Ce module n'utilise ni INSLIB ni le baromètre.
"""
import json
import math
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path


class GyroCalibration:
    DURATION_SECONDS = 5.0
    SAMPLE_PERIOD_SECONDS = 0.01
    MIN_SAMPLES = 100
    MAX_GYRO_STD_RAD_S = 0.02
    MAX_ACC_STD_M_S2 = 0.15
    MAX_MEAN_GYRO_NORM_RAD_S = 0.15

    def __init__(self, path):
        self.path = Path(path)
        self.bias = (0.0, 0.0, 0.0)
        if self.path.exists():
            payload = json.loads(self.path.read_text(encoding="utf-8"))
            self.bias = self._vector(payload["gyro_bias_rad_s"])

    @staticmethod
    def _vector(values):
        result = tuple(float(value) for value in values)
        if len(result) != 3 or not all(math.isfinite(value) for value in result):
            raise ValueError("Calibration gyro : trois valeurs finies sont requises")
        return result

    def correct(self, raw_vector):
        """Soustraire le biais, sans modifier les mesures brutes fournies."""
        values = self._vector(raw_vector)
        return tuple(value - bias for value, bias in zip(values, self.bias))

    def calibrate(self, read_gyro, read_accel):
        """Lecture exclusive requise : ne pas enregistrer pendant cet appel.

        Les callbacks renvoient des objets avec les attributs x, y, z.
        Une calibration refusée laisse le fichier et l'ancien biais intacts.
        """
        gyros, accelerations = [], []
        start = time.monotonic()
        while time.monotonic() - start < self.DURATION_SECONDS:
            gyro = read_gyro()
            acc = read_accel()
            gyros.append(self._vector((gyro.x, gyro.y, gyro.z)))
            accelerations.append(self._vector((acc.x, acc.y, acc.z)))
            time.sleep(self.SAMPLE_PERIOD_SECONDS)

        if len(gyros) < self.MIN_SAMPLES:
            raise ValueError("Calibration refusée : trop peu de mesures, réessayer")

        gyro_columns = tuple(zip(*gyros))
        acc_columns = tuple(zip(*accelerations))
        bias = tuple(statistics.mean(axis) for axis in gyro_columns)
        gyro_std = tuple(statistics.pstdev(axis) for axis in gyro_columns)
        acc_std = tuple(statistics.pstdev(axis) for axis in acc_columns)
        acc_mean = tuple(statistics.mean(axis) for axis in acc_columns)
        norm = lambda vector: math.sqrt(sum(value * value for value in vector))

        if (max(gyro_std) > self.MAX_GYRO_STD_RAD_S or
                max(acc_std) > self.MAX_ACC_STD_M_S2 or
                norm(bias) > self.MAX_MEAN_GYRO_NORM_RAD_S or
                not 8.5 <= norm(acc_mean) <= 11.1):
            raise ValueError(
                "Calibration refusée : mesures incompatibles avec un robot "
                "immobile. Poser le robot sans vibration et réessayer."
            )

        result = {
            "gyro_bias_rad_s": bias,
            "gyro_std_rad_s": gyro_std,
            "acc_std_m_s2": acc_std,
            "samples": len(gyros),
            "duration_s": time.monotonic() - start,
            "calibrated_at": datetime.now(timezone.utc).isoformat(),
            "frame": "navigator_raw",
        }
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
        temporary.replace(self.path)
        # Modifier la calibration active seulement après la sauvegarde réussie.
        self.bias = bias
        return result

