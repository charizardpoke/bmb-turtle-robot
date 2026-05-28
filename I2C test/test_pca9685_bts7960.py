# test_pca9685_bts7960.py
# Run: sudo python3 test_pca9685_bts7960.py

import sys, time, select, termios, tty
from smbus import SMBus  # python3-smbus

I2C_BUS = 1
ADDR = 0x40   # PCA9685

# Adjust to your wiring
LEFT_LPWM  = 0
LEFT_RPWM  = 1
RIGHT_LPWM = 2
RIGHT_RPWM = 3

MODE1      = 0x00
MODE2      = 0x01
PRESCALE   = 0xFE
LED0_ON_L  = 0x06

def w8(bus, reg, val): bus.write_byte_data(ADDR, reg, val & 0xFF)
def r8(bus, reg): return bus.read_byte_data(ADDR, reg)

def set_pwm_freq(bus, freq_hz=1000):
    prescale = int(round(25000000.0 / (4096 * freq_hz)) - 1)
    oldmode = r8(bus, MODE1)
    w8(bus, MODE1, oldmode | 0x10)   # sleep
    w8(bus, PRESCALE, prescale)
    w8(bus, MODE2, 0x04)             # OUTDRV push-pull
    w8(bus, MODE1, 0x20)             # wake + AI
    time.sleep(0.005)

def set_pwm_raw(bus, ch, on, off):
    base = LED0_ON_L + 4*ch
    w8(bus, base+0,  on  & 0xFF)
    w8(bus, base+1, (on  >> 8) & 0x0F)
    w8(bus, base+2,  off & 0xFF)
    w8(bus, base+3, (off >> 8) & 0x0F)

def set_duty(bus, ch, duty):  # duty in [0..100]
    duty = max(0, min(100, duty))
    base = LED0_ON_L + 4*ch
    if duty == 0:
        w8(bus, base+1, 0x00)  # clear FULL_ON
        w8(bus, base+3, 0x10)  # FULL_OFF
    elif duty == 100:
        w8(bus, base+3, 0x00)  # clear FULL_OFF
        w8(bus, base+1, 0x10)  # FULL_ON
    else:
        off = int(duty/100.0 * 4095)
        set_pwm_raw(bus, ch, 0, off)

def drive_wheel(bus, lpwm_ch, rpwm_ch, speed):
    speed = max(-1.0, min(1.0, speed))
    duty = int(abs(speed) * 80)  # cap at 80% for safety
    if speed > 0:
        set_duty(bus, lpwm_ch, 0)
        set_duty(bus, rpwm_ch, duty)
    elif speed < 0:
        set_duty(bus, rpwm_ch, 0)
        set_duty(bus, lpwm_ch, duty)
    else:
        set_duty(bus, lpwm_ch, 0)
        set_duty(bus, rpwm_ch, 0)

def stop_all(bus):
    for ch in (LEFT_LPWM, LEFT_RPWM, RIGHT_LPWM, RIGHT_RPWM):
        set_duty(bus, ch, 0)

def wait_for_any_key():
    # If not a TTY (e.g., piped), fall back to Enter.
    if not sys.stdin.isatty():
        input("Press Enter to stop...\n")
        return
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)  # no Enter needed
        print("Running. Press any key to stop.")
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if r:
                sys.stdin.read(1)
                return
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def main():
    bus = SMBus(I2C_BUS)
    try:
        set_pwm_freq(bus, 1000)  # ~1 kHz
        stop_all(bus)

        # Run forward fast until keypress
        drive_wheel(bus, LEFT_LPWM, LEFT_RPWM,  1.0)
        drive_wheel(bus, RIGHT_LPWM, RIGHT_RPWM, 1.0)

        # No need to spam I2C; hold command and just wait
        wait_for_any_key()

    finally:
        try:
            stop_all(bus)
        finally:
            bus.close()

if __name__ == "__main__":
    main()

