import importlib
import importlib.util
from pathlib import Path
import os
import sys
import types
import unittest
from unittest.mock import Mock, patch

def sensor_class():
    driver = types.ModuleType("bluerobotics_navigator")
    ins = types.ModuleType("INSLIB")
    ins.Navigator = Mock()
    ins.Config = Mock()
    bus = types.ModuleType("smbus2")
    bus.SMBus = Mock()
    path = Path(__file__).resolve().parents[1] / "sensors/navigator_sensors.py"
    spec = importlib.util.spec_from_file_location("saved_calibration_sensor_test", path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {"bluerobotics_navigator": driver,
                                 "INSLIB": ins, "smbus2": bus}):
        spec.loader.exec_module(module)
    return module.NavigatorSensors





class BatteryTests(unittest.TestCase):
    def setUp(self):
        cls = sensor_class()
        self.sensor = cls.__new__(cls)
        self.driver = cls.read_battery_voltage.__globals__["navigator"]
        self.driver.AdcChannel = types.SimpleNamespace(Ch3=3, Ch2=2)
        self.driver.read_adc = Mock(return_value=1.5)

    def test_power_channel_and_psm_conversion(self):
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(self.sensor.read_battery_voltage(), 16.5)
        self.driver.read_adc.assert_called_once_with(3)

    def test_custom_multiplier_and_zero(self):
        with patch.dict(os.environ, {"BATTERY_VOLTAGE_MULTIPLIER": "10"}):
            self.assertEqual(self.sensor.read_battery_voltage(), 15.0)
            self.driver.read_adc.return_value = 0
            self.assertEqual(self.sensor.read_battery_voltage(), 0.0)

    def test_invalid_adc_and_configuration(self):
        for value in (float("nan"), float("inf"), -0.1, 3.4):
            self.driver.read_adc.return_value = value
            with self.subTest(adc=value), self.assertRaises(ValueError):
                self.sensor.read_battery_voltage()
        for value in ("nan", "inf", "0", "-1", "invalid"):
            with patch.dict(os.environ, {"BATTERY_VOLTAGE_MULTIPLIER": value}):
                with self.subTest(multiplier=value), self.assertRaises(ValueError):
                    self.sensor.read_battery_voltage()

    def test_current_conversion_and_zero_offset(self):
        with patch.dict(os.environ, {}, clear=True):
            self.driver.read_adc.return_value = 0.330
            self.assertEqual(self.sensor.read_battery_current(), 0.0)
            self.driver.read_adc.assert_called_with(2)
            self.driver.read_adc.return_value = 0.594
            self.assertAlmostEqual(self.sensor.read_battery_current(), 10.0, places=4)
            self.driver.read_adc.return_value = 0.320
            self.assertLess(self.sensor.read_battery_current(), 0)

    def test_current_custom_calibration_and_invalid_inputs(self):
        with patch.dict(os.environ, {"BATTERY_CURRENT_MULTIPLIER": "20",
                                     "BATTERY_CURRENT_OFFSET": "0.5"}):
            self.assertEqual(self.sensor.read_battery_current(), 20)
        for value in (float("nan"), float("inf"), -0.1, 3.4):
            self.driver.read_adc.return_value = value
            with self.subTest(adc=value), self.assertRaises(ValueError):
                self.sensor.read_battery_current()
        for key, values in (
            ("BATTERY_CURRENT_MULTIPLIER", ("nan", "inf", "0", "-1", "bad")),
            ("BATTERY_CURRENT_OFFSET", ("nan", "inf", "-1", "3.4", "bad")),
        ):
            for value in values:
                with patch.dict(os.environ, {key: value}):
                    with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                        self.sensor.read_battery_current()

    def test_api_at_rest_recording_calibration_and_failure(self):
        with patch.dict(sys.modules, {
            "sensors.navigator_sensors": types.SimpleNamespace(NavigatorSensors=Mock()),
            "sensors.bar30": types.SimpleNamespace(Bar30=Mock()),
        }):
            backend = importlib.import_module("backend")
        sensor = types.SimpleNamespace(read_battery_voltage=Mock(return_value=16.5),
                                       read_battery_current=Mock(return_value=2.5))
        with patch.object(backend, "navigator", sensor):
            for recording, calibrating in ((False, False), (True, False), (False, True)):
                with patch.object(backend, "recording", recording), patch.object(backend, "calibrating", calibrating):
                    self.assertEqual(backend.get_battery(), {"voltage": 16.5, "current": 2.5})
            sensor.read_battery_current.side_effect = OSError("Current ADC unavailable")
            with self.assertRaises(backend.HTTPException) as failure:
                backend.get_battery()
            self.assertEqual(failure.exception.status_code, 503)
            sensor.read_battery_current.side_effect = None
            sensor.read_battery_voltage.side_effect = OSError("ADC unavailable")
            with self.assertRaises(backend.HTTPException) as failure:
                backend.get_battery()
            self.assertEqual(failure.exception.status_code, 503)
        with patch.object(backend, "navigator", None):
            with self.assertRaises(backend.HTTPException) as failure:
                backend.get_battery()
            self.assertEqual(failure.exception.status_code, 503)


if __name__ == "__main__":
    unittest.main()
