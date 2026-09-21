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


def coverage(vectors):
    """Six signed dominant-axis sectors and direction spread, not elapsed time."""
    v = np.asarray(vectors, dtype=float).reshape(-1, 3)
    if not len(v):
        return 0, 0.0
    norms = np.linalg.norm(v, axis=1)
    v = v[norms > 1e-9] / norms[norms > 1e-9, None]
    if not len(v):
        return 0, 0.0
    axes = np.argmax(np.abs(v), axis=1)
    sectors = {(int(a), bool(row[a] > 0)) for a, row in zip(axes, v)}
    spread = float(3 * np.linalg.eigvalsh(v.T @ v / len(v))[0])
    return len(sectors), max(0.0, spread)


class AccelMagCalibration:
    def __init__(self, path):
        self.path = Path(path)
        self.data = {}
        if self.path.exists():
            self.data = json.loads(self.path.read_text(encoding="utf-8"))
            for key in ("acc", "mag"):
                matrix = np.asarray(self.data[key]["matrix"], dtype=float)
                bias = np.asarray(self.data[key]["bias"], dtype=float)
                if (matrix.shape != (3, 3) or bias.shape != (3,) or
                        not np.isfinite(matrix).all() or not np.isfinite(bias).all() or
                        np.linalg.det(matrix) <= 0):
                    raise ValueError("Calibration acc/mag invalide")

    def correct(self, sensor, raw):
        if not self.data:
            return tuple(raw)
        cal = self.data[sensor]
        return tuple(np.asarray(cal["matrix"]) @
                     (np.asarray(raw) - np.asarray(cal["bias"])))

    def calibrate(self, read, report, finish, cancel):
        rec = protocol.Recording()
        start = time.monotonic()
        next_scan = 0.0
        ready = False
        while not finish.is_set():
            if cancel.is_set():
                return None
            elapsed = time.monotonic() - start
            if elapsed > 900:
                raise ValueError("Session limitée à 15 minutes : recommencer")
            acc, gyr, mag = read()
            if not np.isfinite([acc, gyr, mag]).all():
                raise ValueError("Mesure non finie : vérifier les capteurs")
            stamp = time.monotonic_ns() // 1000
            rec.t_us.append(stamp)
            rec.acc.append(acc)
            rec.gyr.append(gyr)
            rec.mag_t_us.append(stamp)
            rec.mag.append(mag)
            if elapsed >= next_scan:
                next_scan = elapsed + 1
                if elapsed < 20:
                    report(phase="rest", progress=0, ready=False,
                           instruction=f"Poser le robot, ne pas le toucher : repos initial {elapsed:.0f}/20 s.",
                           rest_progress=min(100, int(100 * elapsed / 20)))
                else:
                    intervals, a = rec.static_poses(20, 4)
                    means = [a[s:e+1].mean(axis=0) for s, e in intervals]
                    sectors, spread = coverage(means)
                    n = len(intervals)
                    mag_spread = 0.0
                    try:
                        cloud = np.asarray(rec.mag)
                        fit = mag_calib.ellipsoid_fit(cloud[::max(1, len(cloud) // 1500)])
                        if fit.residual_unit <= 0.05:
                            mag_spread = fit.spread
                    except (ValueError, np.linalg.LinAlgError):
                        pass  # More directions are needed before a meaningful fit.
                    fraction = min(n / 20, sectors / 6, spread / 0.25,
                                   mag_spread / 0.25, 1)
                    ready = n >= 20 and sectors == 6 and spread >= 0.25 and mag_spread >= 0.25
                    report(phase="collecting", progress=int(95 * fraction),
                           poses=n, target_poses=20, sectors=sectors,
                           mag_progress=min(100, int(100 * mag_spread / 0.25)),
                           spread=round(spread, 3), ready=ready,
                           instruction=("Positions couvertes. Terminer pour vérifier et enregistrer."
                                        if ready else
                                        "Tourner lentement le robot autour des trois axes, puis le poser "
                                        "dans une nouvelle orientation et rester immobile environ 4 s. "
                                        "Varier dessus, dessous, côtés et orientations obliques."),
                           detail=f"{n}/20 poses détectées (indicatif), {sectors}/6 secteurs. "
                                  f"Couverture magnétique : {min(100, int(100 * mag_spread / 0.25))} % du seuil requis. "
                                  "Le calcul final confirme la qualité.")
            time.sleep(0.01)
        if cancel.is_set():
            return None
        if not ready:
            raise ValueError("Mouvements incomplets : poursuivre les poses")
        report(phase="solving", progress=95, ready=False,
               instruction="Mouvements terminés. Vérification INSLIB en cours, patienter.")
        t, acc, gyr = rec.arrays()
        result = imu_tk.calibrate(acc, gyr, t, init_static_sec=20, with_gyro=False)
        directions = [acc[s:e+1].mean(axis=0) for s, e in result.intervals]
        sectors, spread = coverage(directions)
        if (result.n_positions < 20 or sectors < 6 or spread < 0.25 or
                result.init_static_frac < 0.9 or result.residual_rms > 0.05):
            raise ValueError("Qualité accéléromètre insuffisante : refaire le repos initial "
                             "puis au moins 20 poses immobiles bien réparties")
        corrected = imu_tk.apply_calib(acc, result.acc_matrix, result.acc_bias)
        pairs = [(corrected[s:e+1].mean(axis=0),
                  np.asarray(rec.mag[s:e+1]).mean(axis=0)) for s, e in result.intervals]
        mag = mag_calib.solve(rec.mag, pairs)
        if (mag.spread < 0.25 or mag.align is None or
                mag.residual_rms_ut > 0.05 * mag.field_ut or
                mag.align.observability < 0.15 or mag.align.scatter_after_deg > 3):
            raise ValueError("Qualité magnétique insuffisante : tourner autour des trois axes "
                             "loin des objets métalliques mobiles, puis recommencer")
        payload = {
            "acc": {"matrix": result.acc_matrix.tolist(), "bias": result.acc_bias.tolist()},
            "mag": {"matrix": mag.matrix, "bias": mag.bias},
            "frame": "navigator_raw", "matrix_layout": "row_major",
            "calibrated_at": datetime.now(timezone.utc).isoformat(),
            "positions": result.n_positions, "acc_residual_m_s2": result.residual_rms,
            "mag_residual_ut": mag.residual_rms_ut, "mag_spread": mag.spread,
            "gravity_m_s2": 9.80665, "field_ut": mag.field_ut,
            "field_source": mag.field_source, "warnings": mag.warnings + mag.align.warnings,
        }
        if cancel.is_set():
            return None
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps(payload, indent=2, allow_nan=False), encoding="utf-8")
        temporary.replace(self.path)
        self.data = payload
        return payload
