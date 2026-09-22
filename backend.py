import asyncio
import json
import re
import time
import threading
import math

from datetime import datetime
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from sensors.navigator_sensors import NavigatorSensors
from sensors.bar30 import Bar30


BASE_DIR = Path(__file__).resolve().parent
DATA_DIR = BASE_DIR / "data"
FRONTEND_DIR = BASE_DIR / "frontend"

DATA_DIR.mkdir(exist_ok=True)
FRONTEND_DIR.mkdir(exist_ok=True)

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# État global simple
navigator = None
bar30 = None

recording = False
tasks = []
t0 = None

current_run = ""
current_note = ""
current_folder = None
imu_file_path = None
bar30_file_path = None
info_file_path = None

latest_imu = None
latest_bar30 = None

imu_count = 0
bar30_count = 0
last_error = ""

calibrating = False
calibration_task = None
calibration_status = {}
calibration_finish = threading.Event()
calibration_cancel = threading.Event()

def get_time():
    return (time.monotonic_ns() - t0) / 1e9


def safe_name(text):
    name = re.sub(r"[^a-zA-Z0-9_-]+", "-", text.strip())
    return name.strip("-_")[:40] or "essai"


def write_info(state):
    info = {
        "run_id": current_run,
        "note": current_note,
        "state": state,
        "started_at": started_at,
        "duration": get_time() if t0 else 0,
        "imu_samples": imu_count,
        "bar30_samples": bar30_count,
        "gyro_bias_rad_s": list(navigator.gyro_calibration.bias),
        "accel_mag_calibration": navigator.accel_mag_calibration.data,
        "files": {
            "imu": imu_file_path.name,
            "bar30": bar30_file_path.name,
        },
    }

    # Écriture atomique pour éviter les fichiers JSON vides
    temporary = info_file_path.with_suffix(".tmp")
    temporary.write_text(
        json.dumps(info, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    temporary.replace(info_file_path)


async def record_imu():
    global latest_imu, imu_count, last_error

    with imu_file_path.open("w", encoding="utf-8", buffering=1) as file:
        while recording:
            try:
                data = await asyncio.to_thread(navigator.read_imu)

                if not recording:
                    break

                output = {
                    "time": get_time(),
#                    "xacc": data["xacc"],
#                    "yacc": data["yacc"],
#                    "zacc": data["zacc"],
#                    "xgyro": data["xgyro"],
#                    "ygyro": data["ygyro"],
#                    "zgyro": data["zgyro"],
#                    "xmag": data["xmag"],
#                    "ymag": data["ymag"],
#                    "zmag": data["zmag"],
#                    "roll_deg" : data["roll_deg"],
#                    "pitch_deg" : data["pitch_deg"],
#                    "yaw_deg" : data["yaw_deg"],
#                    "attitude_ok" : data["attitude_ok"],
                    **data,
                }

                file.write(json.dumps(output) + "\n")
                latest_imu = output
                imu_count += 1

            except Exception as error:
                last_error = f"Navigator : {error}"
                await asyncio.sleep(0.2)

            await asyncio.sleep(0)


async def record_bar30():
    global latest_bar30, bar30_count, last_error

    previous_time = None
    previous_depth = None

    with bar30_file_path.open(
        "w",
        encoding="utf-8",
        buffering=1,
    ) as file:

        while recording:
            try:
                data = await asyncio.to_thread(bar30.read)

                if not recording:
                    break

                current_time = get_time()
                current_depth = float(data["depth"])

                sink_speed = 0.0

                if previous_time is not None:
                    delta_time = current_time - previous_time

                    if delta_time > 0:
                        sink_speed = (
                            current_depth - previous_depth
                        ) / delta_time

                output = {
                    "time": current_time,
                    "mbar": float(data["mbar"]),
                    "pa": float(data["pa"]),
                    "temperature": float(data["temperature"]),
                    "depth": current_depth,
                    "sink_speed": sink_speed,
                }

                file.write(json.dumps(output) + "\n")

                latest_bar30 = output
                bar30_count += 1

                previous_time = current_time
                previous_depth = current_depth

            except Exception as error:
                last_error = f"Bar30 : {error}"
                await asyncio.sleep(0.2)

            await asyncio.sleep(0)

def status():
    return {
        "recording": recording,
        "run_id": current_run,
        "note": current_note,
        "imu_samples": imu_count,
        "bar30_samples": bar30_count,
        "error": last_error,
        "calibrating": calibrating,
        "calibration": calibration_status,
        "sensor_calibrations": navigator.calibration_summary() if navigator else {},
    }


@app.on_event("startup")
async def startup():
    global navigator, bar30

    print("Initialisation Navigator...")
    navigator = await asyncio.to_thread(NavigatorSensors)

    print("Initialisation Bar30...")
    bar30 = await asyncio.to_thread(Bar30, bus=6)

    print("Capteurs prêts.")

    print("Liens de l'interface : 192.168.2.2:8080")


@app.on_event("shutdown")
async def shutdown():
    if recording:
        await stop_recording()
    calibration_cancel.set()
    if calibration_task is not None:
        await asyncio.gather(
                calibration_task,
                return_exceptions=True,
        )

@app.post("/api/start")
async def start_recording(payload: dict):
    global recording, tasks, t0
    global current_run, current_note, current_folder
    global imu_file_path, bar30_file_path, info_file_path
    global latest_imu, latest_bar30
    global imu_count, bar30_count, last_error, started_at

    if recording:
        raise HTTPException(400, "Un enregistrement est déjà en cours")

    if calibrating or tasks:
        raise HTTPException(
            409,
            "Calibration ou arrêt d'enregistrement en cours",
        )

    navigator.reset_filtre()

    current_note = str(payload.get("note", "")).strip()
    note_name = safe_name(current_note)

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    current_run = f"{timestamp}_{note_name}"

    # Un dossier par enregistrement
    current_folder = DATA_DIR / current_run
    current_folder.mkdir(parents=True, exist_ok=False)

    imu_file_path = current_folder / f"{current_run}_imu.jsonl"
    bar30_file_path = current_folder / f"{current_run}_bar30.jsonl"
    info_file_path = current_folder / f"{current_run}_info.json"

    latest_imu = None
    latest_bar30 = None
    imu_count = 0
    bar30_count = 0
    last_error = ""

    started_at = datetime.now().astimezone().isoformat()
    t0 = time.monotonic_ns()
    recording = True

    write_info("recording")

    tasks = [
        asyncio.create_task(record_imu()),
        asyncio.create_task(record_bar30()),
    ]

    print(f"[START] {current_run}")
    return status()


@app.post("/api/stop")
async def stop_recording():
    global recording, tasks

    if not recording:
        return status()

    recording = False

    await asyncio.gather(*tasks, return_exceptions=True)
    tasks = []

    write_info("completed")

    print(f"[STOP] {current_run}")
    return status()


@app.get("/api/status")
def get_status():
    return status()


@app.get("/api/battery")
def get_battery():
    if navigator is None:
        raise HTTPException(503, "Navigator pas encore prête")
    try:
        return {
            "voltage": navigator.read_battery_voltage(),
            "current": navigator.read_battery_current(),
        }
    except Exception as error:
        raise HTTPException(503, f"Batterie : {error}") from error


@app.get("/api/live")
def get_live():
    return {
        "recording": recording,
        "imu": latest_imu,
        "bar30": latest_bar30,
    }


@app.get("/api/runs")
def list_runs():
    runs = []

    for path in DATA_DIR.glob("*/*_info.json"):
        try:
            runs.append(json.loads(path.read_text(encoding="utf-8")))
        except (json.JSONDecodeError, OSError):
            pass

    runs.sort(
        key=lambda run: run.get("started_at", ""),
        reverse=True,
    )

    return {"runs": runs}


def get_run_folder(run_id):
    if not re.fullmatch(r"[a-zA-Z0-9_-]+", run_id):
        raise HTTPException(400, "Nom invalide")

    folder = DATA_DIR / run_id

    if not folder.is_dir():
        raise HTTPException(404, "Enregistrement introuvable")

    return folder


def read_jsonl(path, max_points=5000):
    values = []

    with path.open("r", encoding="utf-8") as file:
        for line in file:
            try:
                values.append(json.loads(line))
            except json.JSONDecodeError:
                pass

    if len(values) <= max_points:
        return values

    step = max(1, len(values) // max_points)
    return values[::step]


@app.get("/api/runs/{run_id}")
def get_run(run_id: str):
    folder = get_run_folder(run_id)

    info_path = next(folder.glob("*_info.json"), None)
    imu_path = next(folder.glob("*_imu.jsonl"), None)
    bar30_path = next(folder.glob("*_bar30.jsonl"), None)

    if not all((info_path, imu_path, bar30_path)):
        raise HTTPException(404, "Fichiers incomplets")

    return {
        "info": json.loads(info_path.read_text(encoding="utf-8")),
        "imu": read_jsonl(imu_path),
        "bar30": read_jsonl(bar30_path),
    }

async def run_gyro_calibration():
    global calibrating

    try:
        result = await asyncio.to_thread(navigator.calibrate_gyro)
        return {"ok": True, **result}

    except ValueError as error:
        raise HTTPException(400, str(error)) from error

    except Exception as error:
        raise HTTPException(
            500,
            f"Échec de la calibration : {error}",
        ) from error

    finally:
        calibrating = False


@app.post("/api/calibrate")
async def calibrate_gyro():
    global calibrating, calibration_task

    if recording or tasks:
        raise HTTPException(
            409,
            "Arrête l'enregistrement avant de calibrer",
        )

    if calibrating:
        raise HTTPException(409, "Calibration déjà en cours")

    calibration_status.clear()
    calibrating = True
    calibration_task = asyncio.create_task(run_gyro_calibration())

    # La calibration termine même si la requête du navigateur est annulée.
    return await asyncio.shield(calibration_task)

@app.post("/api/calibrate/use-saved")
async def use_saved_calibration():
    if recording or tasks or calibrating:
        raise HTTPException(409, "Arrêter l'enregistrement ou la calibration avant de charger")
    if navigator is None:
        raise HTTPException(503, "Navigator pas encore prête")
    try:
        navigator.use_saved_calibration()
    except FileNotFoundError as error:
        raise HTTPException(404, str(error)) from error
    except (ValueError, KeyError, TypeError, OSError) as error:
        raise HTTPException(400, f"Calibration non chargée : {error}") from error
    return status()


def report_calibration(**changes):
    global calibration_status
    calibration_status = {**calibration_status, **changes}


async def run_accel_mag_calibration(gravity=9.80665, field_ut=0.0, estimate_misalignment=False):
    global calibrating
    try:
        result = await asyncio.to_thread(
            navigator.calibrate_accel_mag, report_calibration,
            calibration_finish, calibration_cancel, gravity, field_ut, estimate_misalignment)
        if result is None:
            report_calibration(phase="cancelled", ready=False,
                               instruction="Calibration annulée. Ancienne calibration conservée.")
        else:
            report_calibration(phase="completed", progress=100, progress_label="Enregistré", ready=False,
                               instruction=("Accéléromètre et magnétomètre enregistrés automatiquement dans accel_mag_calibration.json."
                                            if result["mag_updated"] else
                                            "Accéléromètre enregistré automatiquement. Magnétomètre inchangé."),
                               detail=result["review_detail"], warnings=result["warnings"])
    except Exception as error:
        report_calibration(phase="failed", ready=False,
                           instruction=f"Calibration échouée : {error}. Ancienne calibration conservée.")
    finally:
        calibrating = False


@app.post("/api/calibrate/accel-mag")
async def start_accel_mag_calibration(payload: dict = None):
    global calibrating, calibration_task, calibration_status
    if recording or tasks or calibrating:
        raise HTTPException(409, "Arrêter l'enregistrement ou la calibration en cours")
    payload = payload or {}
    try:
        gravity = float(payload.get("gravity", 9.80665))
        field_ut = float(payload.get("field_ut", 0.0))
        estimate_misalignment = payload.get("estimate_misalignment", False)
        if not isinstance(estimate_misalignment, bool):
            raise ValueError("Option de désalignement invalide")
        if not math.isfinite(gravity) or not 9.7 <= gravity <= 9.9 or not math.isfinite(field_ut) or not 0 <= field_ut <= 100:
            raise ValueError("Références invalides")
    except (ValueError, TypeError) as error:
        raise HTTPException(400, "Gravité entre 9,7 et 9,9, champ entre 0 et 100 µT et option de désalignement booléenne requis") from error
    calibration_finish.clear()
    calibration_cancel.clear()
    calibration_status = {"kind": "accel-mag", "phase": "rest", "progress": 0,
                          "ready": False, "instruction": "Poser le robot : repos initial de 20 s."}
    calibrating = True
    calibration_task = asyncio.create_task(run_accel_mag_calibration(gravity, field_ut, estimate_misalignment))
    return status()


@app.post("/api/calibrate/finish")
async def finish_accel_mag_calibration():
    if not calibrating or not calibration_status.get("ready"):
        raise HTTPException(409, "Mouvements incomplets ou aucune session active")
    calibration_finish.set()
    report_calibration(ready=False, phase="solving", instruction="Vérification INSLIB en cours.")
    return status()


@app.post("/api/calibrate/cancel")
async def cancel_accel_mag_calibration():
    if not calibrating or calibration_status.get("kind") != "accel-mag":
        raise HTTPException(409, "Aucune session accéléromètre/magnétomètre active")
    calibration_cancel.set()
    return status()


# Doit rester après les routes /api
app.mount(
    "/",
    StaticFiles(directory=FRONTEND_DIR, html=True),
    name="frontend",
)
