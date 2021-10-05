import machine
import esp32
import time
from micropython import const
import micropython

import _thread

FAN_TEMP_THRESHOLD = 80
MIN_FAN_RPM = 500
MAX_FAN_RPM = 5000
MIN_TEMP = 100
MAX_TEMP = 160
TACH_PULSES = 2

FAN_PWM = const(26)
FAN_TACH = const(36)

RPM_OFFSET = 280

micropython.alloc_emergency_exception_buf(100)


fan_pwm_pin = machine.Pin(FAN_PWM, machine.Pin.OUT)
fan_tach_pin = machine.Pin(FAN_TACH, machine.Pin.IN)


def int_remap(value, new_min, new_max, old_min, old_max):
    old_range = int(round(old_max - old_min))
    new_range = int(round(new_max - new_min))
    return int(round((((value - old_min) * new_range) / old_range) + new_min))


class FanController(object):

    def __init__(self, pwm_pin, speed_pin):
        self.pwm_pin = machine.PWM(pwm_pin, freq=25000, duty=0, duty_resolution=20)
        self.speed_pin = speed_pin
        self._last_speed_time = time.ticks_us()
        self._new_speed_time = 0
        self._speed_time_diff = 0.0
        self._new_speed_time_diff = 0.0
        self._rpm = 0
        self._rpm_changed = False
        self._target_rpm = 0
        self.speed_pin.irq(self.speed_callback, trigger=self.speed_pin.IRQ_FALLING)
        self._last_temp = 0
        self._max_duty = self.pwm_pin.maximum_duty()
        self._offset = 0
        self._temp_rpm = 0
        self._smoothing = []
        self._offset_counter = 0

        def _do():
            self.rpm = MAX_FAN_RPM
            count = 0
            while count < 5:
                time.sleep_ms(500)
                rpm = self.rpm

                if rpm >= MAX_FAN_RPM * 0.85:
                    count += 1

            count = 0
            self.rpm = MIN_FAN_RPM
            while count < 5:
                time.sleep_ms(500)
                rpm = self.rpm

                if 0 < rpm <= MIN_FAN_RPM * 1.15:
                    count += 1

        _thread.start_new_thread(_do, ())

    @property
    def is_initilized(self):
        return self._offset != 0

    def update(self):
        temp = self.temperature
        if self._last_temp != temp:
            self._last_temp = temp
            if temp < MIN_TEMP:
                self.rpm = 0
            else:
                self.rpm = int_remap(temp, MIN_FAN_RPM, MAX_FAN_RPM, MIN_TEMP, MAX_TEMP)

    def speed_callback(self, _):
        self._new_speed_time = time.ticks_us()
        self._new_speed_time_diff = time.ticks_diff(self._new_speed_time, self._last_speed_time) * 1e-6

        if (
            (1 / ((MIN_FAN_RPM * TACH_PULSES) / 60) < self._new_speed_time_diff) or
            (1 / ((MAX_FAN_RPM * TACH_PULSES) / 60) > self._new_speed_time_diff)
        ):
            self._last_speed_time = self._new_speed_time
            self._speed_time_diff = 0.0
            return

        if not self._rpm_changed and self._speed_time_diff != 0:
            if not (70 < (self._new_speed_time_diff / self._speed_time_diff) * 100 < 130):
                return

        self._speed_time_diff = self._new_speed_time_diff
        self._last_speed_time = self._new_speed_time

        self._temp_rpm = int(((1 / self._speed_time_diff) / TACH_PULSES) * 60)
        self._smoothing.append(self._temp_rpm)
        if len(self._smoothing) == 41:
            self._smoothing.pop(0)

        if self._rpm_changed:
            if len(self._smoothing) == 40:
                self._smoothing.remove(min(self._smoothing))
                self._smoothing.remove(max(self._smoothing))
                if self._target_rpm * 0.85 < sum(self._smoothing) // len(self._smoothing) < self._target_rpm * 1.15:
                    self._rpm_changed = False
                    self._offset = self._target_rpm - sum(self._smoothing) // len(self._smoothing)
        elif len(self._smoothing) == 40:
            self._offset_counter += 1
            if self._offset_counter == 40:
                self._offset = self._target_rpm - sum(self._smoothing) // len(self._smoothing)
                self._offset_counter = 0

    @property
    def rpm(self):
        diff = time.ticks_diff(time.ticks_us(), int(self._last_speed_time)) * 1e-6

        if diff > (1 / ((MIN_FAN_RPM * TACH_PULSES) / 60)) * 1.5:
            self._speed_time_diff = 0.0
            return 0

        if self._smoothing:
            return sum(self._smoothing) // len(self._smoothing) + self._offset

        return 0

    @rpm.setter
    def rpm(self, value):
        if value < MIN_FAN_RPM:
            self._target_rpm = 0
            self.pwm_pin.duty(0)
            return

        if value > MAX_FAN_RPM:
            value = MAX_FAN_RPM

        self._rpm_changed = True
        self._target_rpm = value

        value = int_remap(value, 0, self._max_duty, 0, MAX_FAN_RPM)
        self.pwm_pin.duty(value)

    def stop(self):
        self.pwm_pin.duty(0)
        machine.disable_irq()
        self.speed_pin.deinit()

    @property
    def temperature(self):
        return esp32.raw_temperature()


fan = FanController(fan_pwm_pin, fan_tach_pin)
