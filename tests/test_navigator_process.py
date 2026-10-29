import os
import resource
import time
import unittest
from unittest.mock import patch

from sensors.navigator_process import NavigatorProcess


def simulated_worker(connection):
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    connection.send((True, None))
    try:
        while True:
            name, args = connection.recv()
            if name == "read_mag":
                os.abort()  # Same process-level failure as Rust panic_cannot_unwind.
            elif name == "read_gyro":
                time.sleep(30)
            else:
                connection.send((True, (1.0, 2.0, 3.0)))
    except EOFError:
        pass
    finally:
        connection.close()


def startup_abort(connection):
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    os.abort()


class NavigatorProcessTests(unittest.TestCase):
    def client(self, **kwargs):
        client = NavigatorProcess(target=simulated_worker, startup_timeout=5,
                                  **kwargs)
        self.addCleanup(client.close)
        return client

    def test_native_abort_is_caught_and_next_read_restarts_worker(self):
        client = self.client(retry_delay=0)
        self.assertEqual(client.read_accel().x, 1.0)
        with self.assertRaisesRegex(OSError, "read_mag"):
            client.read_mag()
        self.assertIsNone(client._process)
        self.assertEqual(client.read_accel().z, 3.0)

    def test_restart_backoff(self):
        client = self.client(retry_delay=60)
        with self.assertRaises(OSError):
            client.read_mag()
        retry_at = client._retry_at
        for _ in range(3):
            with self.assertRaisesRegex(OSError, "indisponible"):
                client.read_accel()
        self.assertEqual(client._retry_at, retry_at)
        self.assertIsNone(client._process)
        with patch("sensors.navigator_process.time.monotonic", return_value=retry_at + 1):
            self.assertEqual(client.read_accel().x, 1.0)

    def test_hung_read_is_terminated(self):
        client = self.client(timeout=0.1, retry_delay=0)
        client.init()
        start = time.monotonic()
        with self.assertRaisesRegex(OSError, "Délai"):
            client.read_gyro()
        self.assertLess(time.monotonic() - start, 3)
        self.assertEqual(client.read_accel().y, 2.0)

    def test_abort_during_initialization(self):
        client = NavigatorProcess(target=startup_abort, startup_timeout=5)
        self.addCleanup(client.close)
        with self.assertRaisesRegex(OSError, "initialisation"):
            client.init()
        self.assertIsNone(client._process)

    def test_close_reaps_worker(self):
        client = self.client()
        client.init()
        client.close()
        self.assertIsNone(client._process)
        self.assertIsNone(client._connection)
