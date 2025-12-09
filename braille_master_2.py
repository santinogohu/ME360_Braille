import serial
import time

# =========================
# USER CONFIG
# =========================

PORT = "COM3"          # change if needed
BAUD = 250000

FEEDRATE_MOVE = 1500   # mm/min for travel moves

SOLENOID_PULSE_MS = 3000     # ms solenoid pulse (time energized)
DWELL_BEFORE_FIRE_MS = 50    # ms to wait after arriving at dot, before firing
DWELL_AFTER_FIRE_MS = 200    # ms to wait after solenoid OFF (retraction time)

# LOGICAL GEOMETRY (U = horizontal, V = vertical)
CELL_PITCH_U = 5.0     # horizontal distance between braille cells
CELL_PITCH_V = 10.0    # vertical distance between braille rows

DOT_OFFSETS = {
    # offsets in (U, V) from cell origin
    1: (0.0, 0.0),
    2: (0.0, 2.5),
    3: (0.0, 5.0),
    4: (2.5, 0.0),
    5: (2.5, 2.5),
    6: (2.5, 5.0),
}

# Motion softening so your rig doesn’t skip itself apart
SAFE_MOTION_TUNING = [
    "M203 X50 Y50 Z50",     # max speeds (mm/s)
    "M201 X200 Y200 Z200",  # max accelerations
    "M204 P200 T200",       # print/travel accel
    "M205 X3 Y3 Z3",        # jerk
]

# =========================
# BRAILLE MAP (Grade 1 letters a–z)
# bit 0 = dot1, bit 1 = dot2, ... bit5 = dot6
# layout:
# 1 4
# 2 5
# 3 6
# =========================

BRAILLE_MAP = {
    "a": 0b000001,
    "b": 0b000011,
    "c": 0b000101,
    "d": 0b000111,
    "e": 0b000110,
    "f": 0b001101,
    "g": 0b001111,
    "h": 0b001110,
    "i": 0b001001,
    "j": 0b001011,
    "k": 0b010001,
    "l": 0b010011,
    "m": 0b010101,
    "n": 0b010111,
    "o": 0b010110,
    "p": 0b011101,
    "q": 0b011111,
    "r": 0b011110,
    "s": 0b011001,
    "t": 0b011011,
    "u": 0b110001,
    "v": 0b110011,
    "w": 0b101011,
    "x": 0b110101,
    "y": 0b110111,
    "z": 0b110110,
}

# =========================
# SERIAL / GCODE HELPERS
# =========================

def open_serial():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)  # board reset time
    return ser


def send(ser, cmd):
    print(f">> {cmd}")
    ser.write((cmd + "\n").encode())
    ser.flush()
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print(f"<< {line}")
        if "ok" in line.lower():
            break


def move_to_uv(ser, u, v, feedrate=FEEDRATE_MOVE):
    """
    Logical move to (U,V).

    Hardware mapping:
      - U axis -> X and Y must match (X = Y = U)
      - V axis -> Z (or change to E if that’s your vertical motor)
    """
    send(ser, f"G1 X{u:.3f} Y{u:.3f} Z{v:.3f} F{feedrate}")


def fire_solenoid(ser, ms=SOLENOID_PULSE_MS):
    # Solenoid ON
    send(ser, "M42 P65 S255")
    time.sleep(ms / 1000.0)
    # Solenoid OFF (start of retraction)
    send(ser, "M42 P65 S0")


# =========================
# BRAILLE LOGIC
# =========================

def char_to_pattern(ch):
    if ch == " ":
        return 0
    return BRAILLE_MAP.get(ch.lower(), 0)


def iter_dots(pattern):
    for dot in range(1, 7):
        if pattern & (1 << (dot - 1)):
            yield dot


def cell_origin_uv(cell_col, cell_row):
    u0 = cell_col * CELL_PITCH_U
    v0 = cell_row * CELL_PITCH_V
    return u0, v0


def print_braille_line(ser, text, row_index=0):
    """
    Print one line of braille using:
      U: along your dual-motor axis (X & Y together)
      V: perpendicular axis (Z)

    Sequence per dot:
      1) Move to dot
      2) Wait DWELL_BEFORE_FIRE_MS
      3) Fire solenoid (SOLENOID_PULSE_MS)
      4) Wait DWELL_AFTER_FIRE_MS (retraction time)
    """
    for cell_col, ch in enumerate(text):
        pattern = char_to_pattern(ch)
        if pattern == 0:
            continue  # space / unsupported char, skip dots but keep column

        cell_u, cell_v = cell_origin_uv(cell_col, row_index)

        for dot in iter_dots(pattern):
            du, dv = DOT_OFFSETS[dot]
            u = cell_u + du
            v = cell_v + dv

            # Move into position
            move_to_uv(ser, u, v)

            # Let mechanics settle before firing
            time.sleep(DWELL_BEFORE_FIRE_MS / 1000.0)

            # Fire solenoid
            fire_solenoid(ser)

            # Give the solenoid time to retract before moving again
            time.sleep(DWELL_AFTER_FIRE_MS / 1000.0)


# =========================
# MAIN
# =========================

def main():
    ser = open_serial()

    # Soften motion so both motors don’t fight/skip
    for cmd in SAFE_MOTION_TUNING:
        send(ser, cmd)

    # Absolute positioning
    send(ser, "G90")

    # Treat current physical position as (U,V) = (0,0)
    # i.e. X=Y=0, Z=0
    send(ser, "G92 X0 Y0 Z0")

    text = "hello"
    print(f"Printing braille for {text!r} with X/Y locked together")
    print_braille_line(ser, text, row_index=0)

    print("Done.")
    ser.close()


if __name__ == "__main__":
    main()
