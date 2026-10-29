"""Keep native Navigator aborts outside the web/recording process."""
import multiprocessing
import threading
import time
from types import SimpleNamespace


def _worker(connection):
    # Import and initialize native code only in this disposable process.
    import bluerobotics_navigator as driver
    from smbus2 import SMBus

    try:
        with SMBus(1) as bus:
            bmp390_id = bus.read_byte_data(0x76, 0x00)
            bmp280_id = bus.read_byte_data(0x76, 0xD0)
        if bmp390_id == 0x60:
            version = driver.NavigatorVersion.Version2
        elif bmp280_id == 0x58:
            version = driver.NavigatorVersion.Version1
        else:
            raise RuntimeError("Baromètre Navigator non reconnu")
        driver.set_navigator_version(version)
        driver.init()
        connection.send((True, None))
        while True:
            name, args = connection.recv()
            try:
                if name == "read_adc":
                    value = driver.read_adc(getattr(driver.AdcChannel, args[0]))
                else:
                    value = getattr(driver, name)()
                    value = tuple(float(getattr(value, axis)) for axis in "xyz")
                connection.send((True, value))
            except Exception as error:
                connection.send((False, str(error)))
    except (EOFError, BrokenPipeError):
        pass
    except Exception as error:
        connection.send((False, str(error)))
    finally:
        connection.close()


class NavigatorProcess:
    AdcChannel = SimpleNamespace(Ch2="Ch2", Ch3="Ch3")

    def __init__(self, *, target=_worker, timeout=2.0, startup_timeout=10.0,
                 retry_delay=2.0):
        self._target = target
        self._timeout = timeout
        self._startup_timeout = startup_timeout
        self._retry_delay = retry_delay
        self._retry_at = 0.0
        self._process = None
        self._connection = None
        self._lock = threading.Lock()

    def _stop(self):
        if self._connection is not None:
            self._connection.close()
            self._connection = None
        if self._process is not None and self._process.pid is None:
            self._process.close()
            self._process = None
        if self._process is not None:
            self._process.join(timeout=0.05)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=0.5)
            if self._process.is_alive():
                self._process.kill()
                self._process.join(timeout=0.5)
            if self._process.is_alive():
                # Never start a second owner while the old one is still alive.
                raise OSError("Processus Navigator bloqué, arrêt impossible")
            self._process.close()
            self._process = None

    def close(self):
        with self._lock:
            self._stop()

    def _receive(self, timeout):
        if not self._connection.poll(timeout):
            raise OSError("Délai de réponse Navigator dépassé")
        ok, value = self._connection.recv()
        if not ok:
            raise OSError(value)
        return value

    def _start(self):
        if self._process is not None and self._process.is_alive() and self._connection is not None:
            return
        self._stop()
        context = multiprocessing.get_context("spawn")
        self._connection, child = context.Pipe()
        self._process = context.Process(target=self._target, args=(child,), daemon=True)
        try:
            self._process.start()
        finally:
            child.close()
        self._receive(self._startup_timeout)

    def _call(self, name=None, *args):
        with self._lock:
            if time.monotonic() < self._retry_at:
                raise OSError("Navigator indisponible : nouvelle tentative dans quelques secondes")
            try:
                self._start()
                if name is None:
                    return
                self._connection.send((name, args))
                return self._receive(self._timeout)
            except (OSError, EOFError) as error:
                self._retry_at = time.monotonic() + self._retry_delay
                self._stop()
                raise OSError(
                    f"Lecture Navigator interrompue ({name or 'initialisation'}) : "
                    f"{error or 'arrêt du pilote natif / I2C'}. "
                    "Aucune mesure valide ; reprise automatique différée."
                ) from error

    def init(self):
        self._call()

    def read_adc(self, channel):
        return self._call("read_adc", channel)

    def _vector(self, name):
        return SimpleNamespace(**dict(zip("xyz", self._call(name))))

    def read_accel(self):
        return self._vector("read_accel")

    def read_gyro(self):
        return self._vector("read_gyro")

    def read_mag(self):
        return self._vector("read_mag")
