import array
import micropython
import utime
from machine import Pin
from micropython import const

class InvalidChecksum(Exception):
    pass

class InvalidPulseCount(Exception):
    pass

MAX_UNCHANGED = const(100)
MIN_INTERVAL_US = const(200000)
HIGH_LEVEL = const(50)
EXPECTED_PULSES = const(84)

class DHT11:
    def __init__(self, pin):
        self._pin = Pin(pin, Pin.OUT, Pin.PULL_DOWN)
        self._last_measure = utime.ticks_us()
        self._temperature = -1
        self._humidity = -1

    def measure(self):
        current_ticks = utime.ticks_us()
        if utime.ticks_diff(current_ticks, self._last_measure) < MIN_INTERVAL_US and (
            self._temperature > -1 or self._humidity > -1
        ):
            # Too soon since the last measurement
            return

        self._send_init_signal()
        try:
            pulses = self._capture_pulses()
            buffer = self._convert_pulses_to_buffer(pulses)
            self._verify_checksum(buffer)
            self._humidity = buffer[0] + buffer[1] / 10
            self._temperature = buffer[2] + buffer[3] / 10
        except (InvalidPulseCount, InvalidChecksum) as e:
            print(f"Error during measurement: {e}")
        finally:
            self._last_measure = utime.ticks_us()

    @property
    def humidity(self):
        self.measure()
        return self._humidity

    @property
    def temperature(self):
        self.measure()
        return self._temperature

    def _send_init_signal(self):
        self._pin.init(Pin.OUT, Pin.PULL_DOWN)
        self._pin.value(1)
        utime.sleep_ms(50)
        self._pin.value(0)
        utime.sleep_ms(18)

    @micropython.native
    def _capture_pulses(self):
        self._pin.init(Pin.IN, Pin.PULL_UP)
        val = 1
        idx = 0
        transitions = bytearray(EXPECTED_PULSES)
        unchanged = 0
        timestamp = utime.ticks_us()

        while unchanged < MAX_UNCHANGED:
            if val != self._pin.value():
                if idx >= EXPECTED_PULSES:
                    raise InvalidPulseCount(
                        f"Got more than {EXPECTED_PULSES} pulses"
                    )
                now = utime.ticks_us()
                transitions[idx] = now - timestamp
                timestamp = now
                idx += 1
                val = 1 - val
                unchanged = 0
            else:
                unchanged += 1

        self._pin.init(Pin.OUT, Pin.PULL_DOWN)
        if idx != EXPECTED_PULSES:
            raise InvalidPulseCount(
                f"Expected {EXPECTED_PULSES} but got {idx} pulses"
            )
        return transitions[4:]

    def _convert_pulses_to_buffer(self, pulses):
        binary = 0
        for idx in range(0, len(pulses), 2):
            binary = binary << 1 | int(pulses[idx] > HIGH_LEVEL)

        buffer = array.array("B")
        for shift in range(4, -1, -1):
            buffer.append(binary >> shift * 8 & 0xFF)
        return buffer

    def _verify_checksum(self, buffer):
        checksum = sum(buffer[:4])
        if checksum & 0xFF != buffer[4]:
            print(f"Checksum mismatch: {checksum & 0xFF} != {buffer[4]}")
            raise InvalidChecksum()