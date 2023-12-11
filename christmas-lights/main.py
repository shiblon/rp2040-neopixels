import random
import sys
from time import sleep, ticks_ms
from neopixel import NeoPixel
from machine import Pin, Timer
import machine
machine.freq(240000000)


num_pixels = 200
data_pin = 18


class Twinkle(object):
    steady_base_steps = 20
    steady_random_steps = 10
    peak_intensities = (0.7, 0.75, 0.8, 0.85, 0.95, 1.0, 0.95, 0.85, 0.8, 0.7)
    base_intensity = 0.55
    BASE_COLOR = (0.4, 1.0, 0.0)
    min_grb = 16
    max_grb = 191
    # NOTE: min 16, max 191, base (0.4, 1.0, 0.0) gives a nice rose gold.
    # NOTE: the reason it isn't just yellow is that there is a minimum amount of blue.

    STATE_BASE = 0
    STATE_PEAK = 1
    NUM_STATES = 2

    def __init__(self, np, pixel_idx):
        self.np = np
        self.idx = pixel_idx
        self.state = self.STATE_BASE
        self.state_step = random.randint(0, self.steady_base_steps // 2)
        self._set_base_end()

    def _set_base_end(self):
        self.state_end = self.steady_base_steps + random.randint(-self.steady_random_steps,
                                                                 self.steady_random_steps+1)

    def _set_peak_end(self):
        self.state_end = len(self.peak_intensities) - 1

    def _update_state(self):
        self.state_step += 1
        if self.state_step >= self.state_end:
            self.state = (self.state + 1) % self.NUM_STATES
            self.state_step = 0

            if self.state == self.STATE_BASE:
                self._set_base_end()
            elif self.state == self.STATE_PEAK:
                self._set_peak_end()

    def _grb(self, intensity):
        return tuple(self.min_grb + int((self.max_grb-self.min_grb) * intensity * c)
                     for c in self.BASE_COLOR)

    def __next__(self):
        self._update_state()
        if self.state == self.STATE_BASE:
            return self._grb(self.base_intensity)
        elif self.state == self.STATE_PEAK:
            return self._grb(self.peak_intensities[self.state_step])
        else:
            print("Weird state: %r" % self.state)


def twinkle(np, delay_ms=40):
    all_states = [Twinkle(np, i) for i in range(num_pixels)]
    # Note: maybe a state wrapper than knows when something actually changes? To optimize?
    # Note: maybe use a timer later on?
    # global_timer = Timer(period=1000, mode=Timer.PERIODIC, callback=advance_states)
    while True:
        for i, s in enumerate(all_states):
            np[i] = next(s)
        sleep(delay_ms/1000)
        np.write()


def crazy(np, delay_ms=250):
    colors = [(0x00, 0x00, 0x00),
              (0x80, 0x80, 0x80),
              (0xc0, 0xc0, 0xc0),
              (0xff, 0xff, 0xff),
              (0x00, 0x80, 0x00),
              (0x00, 0xff, 0x00),
              (0x80, 0x80, 0x00),
              (0xff, 0xff, 0x00),
              (0x80, 0x00, 0x00),
              (0xff, 0x00, 0x00),
              (0x80, 0x00, 0x80),
              (0xff, 0x00, 0xff),
              (0x00, 0x00, 0x80),
              (0x00, 0x00, 0xff),
              (0x00, 0x80, 0x80),
              (0x00, 0xff, 0xff), ]

    while True:
        for i in range(num_pixels):
            #np[i] = colors[(start + i) % num_pixels]
            np[i] = random.choice(colors)
        np.write()
        sleep(delay_ms/1000)


def spiral(np, delay_ms=10):
    colors = [
        (0xaf, 0xff, 0xff),
        (0x9f, 0xef, 0x00),
        (0x8f, 0xdf, 0x00),
        (0x7f, 0xcf, 0x00),
        (0x6f, 0xbf, 0x00),
        (0x5f, 0xaf, 0x00),
        (0x4f, 0x9f, 0x00),
        (0x3f, 0x8f, 0x00),
        (0x2f, 0x7f, 0x00),
        (0x00, 0x00, 0x00),
        (0x00, 0x00, 0x00),
        (0x00, 0x00, 0x00),
        (0x00, 0x00, 0x00),
        (0x00, 0x00, 0x00),
        (0x00, 0x00, 0x00),
        (0x00, 0x00, 0x00),
        (0x00, 0x00, 0x00),
        (0x00, 0x00, 0x00),
        (0x00, 0x00, 0x00),
        (0x00, 0x00, 0x00),
        (0x00, 0x00, 0x00),
        (0x00, 0x00, 0x00),
        (0x00, 0x00, 0x00),
        (0x00, 0x00, 0x00),
    ]

    while True:
        for start in range(len(colors)):
            for i in range(num_pixels):
                np[i] = colors[(start + i) % len(colors)]
            np.write()
            sleep(delay_ms/1000)


def debounced_button_handler(handler):
    last_ms = ticks_ms()

    def _deb_handler(pin):
        nonlocal last_ms
        curr_ms = ticks_ms()
        if curr_ms - last_ms < 200:
            return False
        try:
            handler(pin)
            return True
        finally:
            last_ms = curr_ms
    return _deb_handler


def main():
    switch = Pin(26, Pin.IN, Pin.PULL_UP)
    switch.irq(handler=debounced_button_handler(lambda p: print("pressed", p)),
               trigger=Pin.IRQ_FALLING)

    patterns = [
        twinkle,
        crazy,
        spiral,
    ]

    patt = 0

    np = NeoPixel(Pin(data_pin), num_pixels)
    # TODO:
    # - update strategies to be coroutines
    # - call them from a Timer
    # - user interrupt handler to switch between strategies
    patterns[patt](np)


try:
    main()
finally:
    print("shutting down")
