"""Guided INSLIB multi-position calibration in the raw Navigator frame."""
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

TOOLS = Path(__file__).resolve().parents[2] / "INSLIB" / "tools"
sys.path.insert(0, str(TOOLS))
import inslib_imu_calib as protocol
import inslib_imu_tk as imu_tk
import inslib_mag_calib as mag_calib


def solve_recording(rec, gravity=protocol.G_MPS2, field_ut=0.0, estimate_misalignment=False, log=None):
    """INSLIB solve/solve_mag flow, with only the gyro fitting disabled.

    Use the upstream detector, threshold sweep, calibration result wrapper,
    quality warnings and time-based magnetic pose pairing without extra gates.
    """
    if len(rec) < 1000:
        raise ValueError("only %d samples recorded, that is not a session" % len(rec))
    t, acc, gyr = rec.arrays()
    result = imu_tk.calibrate(
        acc, gyr, t, g_mag=gravity,
        init_static_sec=protocol.DEFAULT_INIT_SEC, with_gyro=False,
        estimate_misalignment=estimate_misalignment, log=log)
    cal = protocol.Calibration.from_result(
        result, 0.0, 0.0, rec.rate_hz(), rec.temp_min, rec.temp_max,
        gravity, float(t[-1]))
    mag = None
    warnings = list(cal.warnings)
    try:
        mag = protocol.solve_mag(rec, cal, field_ut=field_ut,
                                 field_source="given" if field_ut else "", log=log)
    except Exception as error:
        warnings.append(f"magnetometer not calibrated: {error}")
    if mag is not None:
        warnings.extend(mag.warnings)
        if mag.align is not None:
            warnings.extend(mag.align.warnings)
    return cal, mag, warnings


class AccelMagCalibration:
    def __init__(self, path):
        self.path = Path(path)
        self.data = {}
        if self.path.exists():
            self.data = json.loads(self.path.read_text(encoding="utf-8"))
            for key in ("acc", "mag"):
                if key not in self.data:
                    continue
                matrix = np.asarray(self.data[key]["matrix"], dtype=float)
                bias = np.asarray(self.data[key]["bias"], dtype=float)
                if (matrix.shape != (3, 3) or bias.shape != (3,) or
                        not np.isfinite(matrix).all() or not np.isfinite(bias).all() or
                        np.linalg.det(matrix) <= 0):
                    raise ValueError("Calibration acc/mag invalide")

    def correct(self, sensor, raw):
        if sensor not in self.data:
            return tuple(raw)
        cal = self.data[sensor]
        return tuple(np.asarray(cal["matrix"]) @
                     (np.asarray(raw) - np.asarray(cal["bias"])))

    def calibrate(self, read, report, finish, cancel, gravity=protocol.G_MPS2,
                  field_ut=0.0, estimate_misalignment=False):
        rec = protocol.Recording()
        start = time.monotonic()
        next_scan = 0.0
        while not finish.is_set():
            if cancel.is_set():
                return None
            # Navigator timestamps each sensor read independently. These are
            # host read times, not hardware acquisition timestamps.
            stamp, acc, gyr, mag_stamp, mag = read()
            if not np.isfinite([acc, gyr, mag]).all():
                raise ValueError("Mesure non finie : vérifier les capteurs")
            rec.t_us.append(stamp)
            rec.acc.append(acc)
            rec.gyr.append(gyr)
            rec.mag_t_us.append(mag_stamp)
            rec.mag.append(mag)
            elapsed = time.monotonic() - start
            if elapsed >= next_scan:
                next_scan = elapsed + 1
                if elapsed < protocol.DEFAULT_INIT_SEC:
                    report(phase="rest", progress=int(100 * elapsed / protocol.DEFAULT_INIT_SEC),
                           progress_label="Repos initial", ready=len(rec) >= 1000,
                           instruction=f"Ne pas toucher le robot : repos initial {elapsed:.0f}/20 s.")
                else:
                    intervals, _ = rec.static_poses(
                        protocol.DEFAULT_INIT_SEC, protocol.DEFAULT_POSE_SEC)
                    n = len(intervals)
                    report(phase="collecting", progress=min(100, int(100 * n / 20)),
                           progress_label="Objectif conseillé de 20 poses (indicatif)",
                           poses=n, target_poses=20, ready=len(rec) >= 1000,
                           instruction="Soulever le robot, le tourner vers une nouvelle orientation, "
                                       "le poser et rester immobile environ 4 s. "
                                       "Varier les orientations dans toutes les directions.",
                           detail=f"{n} poses détectées ; minimum INSLIB : {protocol.MIN_POSITIONS}, "
                                  "20 ou plus conseillées. Le pourcentage ne mesure pas la qualité. "
                                  "Arrêter et calculer lorsque les orientations sont bien variées.")
            time.sleep(0.01)
        if cancel.is_set():
            return None
        report(phase="solving", ready=False,
               instruction="Calcul INSLIB en cours. Patienter.")
        cal, mag, warnings = solve_recording(rec, gravity, field_ut, estimate_misalignment)
        timestamp = datetime.now(timezone.utc).isoformat()
        payload = {
            **self.data,
            "acc": {"matrix": cal.acc_matrix, "bias": cal.acc_bias,
                    "calibrated_at": timestamp},
            "frame": "navigator_raw", "matrix_layout": "row_major",
            "calibrated_at": timestamp, "positions": cal.n_positions,
            "acc_residual_m_s2": cal.residual_rms,
            "gravity_m_s2": gravity, "warnings": warnings,
            "estimate_misalignment": estimate_misalignment,
            "mag_updated": mag is not None,
            "acc_stats": cal.acc_stats,
        }
        if mag is not None:
            payload.update(
                mag={"matrix": mag.matrix, "bias": mag.bias, "calibrated_at": timestamp},
                mag_residual_ut=mag.residual_rms_ut, mag_spread=mag.spread,
                field_ut=mag.field_ut, field_source=mag.field_source)
        elif "mag" in payload:
            # Preserve both the previous magnetic correction and its date.
            payload["mag"] = dict(payload["mag"])
            payload["mag"].setdefault("calibrated_at", self.data.get("calibrated_at"))
        detail = (f"{cal.n_positions} poses. Erreur accéléromètre RMS : "
                  f"{cal.acc_stats['raw']['rms']:.5f} → {cal.residual_rms:.5f} m/s².")
        if mag is not None:
            detail += (f" Erreur magnétique RMS : {mag.raw_rms_ut:.3f} → "
                       f"{mag.residual_rms_ut:.3f} µT. Dispersion : {mag.spread:.3f}.")
        else:
            detail += " Magnétomètre non recalibré ; correction précédente conservée si disponible."
        payload["review_detail"] = detail
        if cancel.is_set():
            return None
        return self.save(payload)

    def save(self, payload):
        """Persist before activating, just like the gyro calibration."""
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps(payload, indent=2, allow_nan=False), encoding="utf-8")
        temporary.replace(self.path)
        self.data = payload
        return self.data
