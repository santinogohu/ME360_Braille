import serial
import time

# ============================================================
# USER CONFIG – MACHINE / ELECTRONICS
# ============================================================

PORT = "COM3"          # Serial port of your controller (e.g. "COM3", "/dev/ttyUSB0")
BAUD = 250000          # Baud rate; must match your firmware

FEEDRATE_MOVE = 1500   # Travel feedrate in mm/min for G1 moves

SOLENOID_PIN = 65      # Firmware pin number used with M42 to drive the solenoid

# Solenoid timing (tune to your hardware)
# NORMAL logic: S0 = OFF, S255 = ON
SOLENOID_ON_MS = 200       # How long to energize the solenoid (down stroke)
SOLENOID_RETRACT_MS = 200  # Time to allow retract (up) before moving away

# EXTRA SAFETY DELAYS
DELAY_BETWEEN_DOTS_MS = 500    # extra delay after each dot (inside a letter)
DELAY_BETWEEN_CELLS_MS = 500   # extra delay after each letter/cell

# Motion tuning so the rig doesn’t skip
SAFE_MOTION_TUNING = [
    "M203 X50 Y50 Z50",     # Max speeds (mm/s)
    "M201 X200 Y200 Z200",  # Max accelerations
    "M204 P200 T200",       # Print/travel accelerations
    "M205 X3 Y3 Z3",        # Jerk limits
]

# ============================================================
# USER CONFIG – BOARD GEOMETRY (HOLE GRID, IN MILLIMETERS)
# ============================================================
# Each braille cell is a 2 x 3 grid of holes.

BOARD_CELLS_U = 20  # number of cell columns
BOARD_CELLS_V = 22  # number of cell rows

# Measured center-to-center spacing between neighboring holes
HOLE_PITCH_U_MM = 2.4   # horizontal hole spacing in mm
HOLE_PITCH_V_MM = 2.4   # vertical hole spacing in mm

# Hole counts inside a cell (2 x 3):
HOLES_BETWEEN_DOTS_U = 1  # holes between left/right dots within a cell
HOLES_BETWEEN_DOTS_V = 1  # holes between top/middle/bottom dots within a cell

# Derived cell pitch in mm (your tuned values)
CELL_PITCH_U = 7
CELL_PITCH_V = 11

# Dot offsets from cell origin (origin at dot 1 position)
DOT_OFFSETS = {
    # dot: (U offset, V offset) in mm from cell origin
    1: (0 * HOLE_PITCH_U_MM,
        0 * HOLE_PITCH_V_MM),
    2: (0 * HOLE_PITCH_U_MM,
        1 * HOLE_PITCH_V_MM * HOLES_BETWEEN_DOTS_V),
    3: (0 * HOLE_PITCH_U_MM,
        2 * HOLE_PITCH_V_MM * HOLES_BETWEEN_DOTS_V),

    4: (1 * HOLE_PITCH_U_MM * HOLES_BETWEEN_DOTS_U,
        0 * HOLE_PITCH_V_MM),
    5: (1 * HOLE_PITCH_U_MM * HOLES_BETWEEN_DOTS_U,
        1 * HOLE_PITCH_V_MM * HOLES_BETWEEN_DOTS_V),
    6: (1 * HOLE_PITCH_U_MM * HOLES_BETWEEN_DOTS_U,
        2 * HOLE_PITCH_V_MM * HOLES_BETWEEN_DOTS_V),
}

# ============================================================
# USER CONFIG – TEXT / LAYOUT
# ============================================================

TEXT_TO_PRINT = "We Are Charlie Kirk we carry the flame"
START_ROW_INDEX = 0     # Braille row index (0 = first row; must be < BOARD_CELLS_V)

# ============================================================
# BRAILLE MAP (Grade 1 letters, numbers, space)
# ============================================================

BRAILLE_MAP = {
    "a": 0b000001,
    "b": 0b000011,
    "c": 0b001001,
    "d": 0b011001,
    "e": 0b010001,
    "f": 0b001011,
    "g": 0b011011,
    "h": 0b010011,
    "i": 0b001010,
    "j": 0b011010,
    "k": 0b000101,
    "l": 0b000111,
    "m": 0b001101,
    "n": 0b011101,
    "o": 0b010101,
    "p": 0b001111,
    "q": 0b011111,
    "r": 0b010111,
    "s": 0b001110,
    "t": 0b011110,
    "u": 0b100101,
    "v": 0b100111,
    "w": 0b111010,
    "x": 0b101101,
    "y": 0b111101,
    "z": 0b110101,
    "1": 0b000001,
    "2": 0b000011,
    "3": 0b000101,
    "4": 0b000111,
    "5": 0b000110,
    "6": 0b001101,
    "7": 0b001111,
    "8": 0b001110,
    "9": 0b001001,
    "0": 0b001011,
    " ": 0b000000,
}

# ============================================================
# SERIAL / GCODE HELPERS
# ============================================================

def open_serial():
    """Open the serial port and wait for the controller to reset."""
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)  # allow board reset time
    return ser

def send(ser, cmd: str):
    """Send a single G/M-code line and wait for an 'ok'."""
    print(f">> {cmd}")
    ser.write((cmd + "\n").encode())
    ser.flush()
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print(f"<< {line}")
        if "ok" in line.lower():
            break

def move_to_uv(ser, u: float, v: float, feedrate: float = FEEDRATE_MOVE):
    """
    Logical move to (U,V) in mm, mapped to machine axes:
      X = U
      Y = U
      Z = V
    """
    send(ser, f"G1 X{u:.3f} Y{u:.3f} Z{v:.3f} F{feedrate}")
    # We rely on M400 in fire_solenoid to ensure motion is finished before punching

def fire_solenoid(ser):
    """
    Single dot:
      - Wait until motion is finished (M400)
      - ON (down) for SOLENOID_ON_MS
      - OFF (up), wait SOLENOID_RETRACT_MS
    NORMAL logic: S255 = ON, S0 = OFF.
    """
    # Make sure all moves are finished before punching
    send(ser, "M400")

    # ON: energize solenoid (down stroke)
    send(ser, f"M42 P{SOLENOID_PIN} S255")
    time.sleep(SOLENOID_ON_MS / 1000.0)

    # OFF: de-energize solenoid (rest/up)
    send(ser, f"M42 P{SOLENOID_PIN} S0")
    time.sleep(SOLENOID_RETRACT_MS / 1000.0)

# ============================================================
# BRAILLE LOGIC
# ============================================================

def char_to_pattern(ch: str) -> int:
    """Convert a character to a 6-bit braille dot pattern. Unknown/space -> 0."""
    if ch == " ":
        return 0
    return BRAILLE_MAP.get(ch.lower(), 0)

def iter_dots(pattern: int):
    """Yield dot numbers (1–6) that are set in the pattern."""
    for dot in range(1, 7):
        if pattern & (1 << (dot - 1)):
            yield dot

def cell_origin_uv(cell_col: int, cell_row: int):
    """Return (U,V) origin in mm for the given cell indices."""
    u0 = cell_col * CELL_PITCH_U
    v0 = cell_row * CELL_PITCH_V
    return u0, v0

def print_braille_text(ser, text: str, start_row_index: int = 0):
    """
    Print braille for an arbitrary-length string, automatically wrapping:
      - when we hit BOARD_CELLS_U columns, move to next row
      - stop if we run out of rows (>= BOARD_CELLS_V)
    """
    row = start_row_index
    col = 0

    if row < 0 or row >= BOARD_CELLS_V:
        print(f"Start row {row} is outside board (0..{BOARD_CELLS_V - 1})")
        return

    for ch in text:
        # If we've filled this row, move to the next
        if col >= BOARD_CELLS_U:
            row += 1
            col = 0
            if row >= BOARD_CELLS_V:
                print("No more rows on board; stopping.")
                break

        pattern = char_to_pattern(ch)

        # Blank cell for spaces / unknown chars, but still advance one cell
        if pattern == 0:
            col += 1
            time.sleep(DELAY_BETWEEN_CELLS_MS / 1000.0)
            continue

        cell_u, cell_v = cell_origin_uv(col, row)

        # Dots within a single letter/cell
        for dot in iter_dots(pattern):
            du, dv = DOT_OFFSETS[dot]
            u = cell_u + du
            v = cell_v + dv

            move_to_uv(ser, u, v)
            fire_solenoid(ser)

            # extra safety delay between dots in same letter
            time.sleep(DELAY_BETWEEN_DOTS_MS / 1000.0)

        # done with this cell/letter
        col += 1
        time.sleep(DELAY_BETWEEN_CELLS_MS / 1000.0)

# ============================================================
# MAIN
# ============================================================

def main():
    ser = open_serial()

    # Make sure solenoid starts OFF
    send(ser, f"M42 P{SOLENOID_PIN} S0")

    # Conservative motion settings
    for cmd in SAFE_MOTION_TUNING:
        send(ser, cmd)

    # Absolute positioning, millimeters
    send(ser, "G90")
    # Optionally ensure mm units:
    # send(ser, "G21")

    # Zero logical origin at current physical tool position
    send(ser, "G92 X0 Y0 Z0")

    print(f"Printing braille for {TEXT_TO_PRINT!r} starting at row {START_ROW_INDEX}")
    print_braille_text(ser, TEXT_TO_PRINT, start_row_index=START_ROW_INDEX)

    # Return to original position
    move_to_uv(ser, 0.0, 0.0)
    send(ser, "M400")  # ensure final move is complete

    # Ensure solenoid is OFF at the end
    send(ser, f"M42 P{SOLENOID_PIN} S0")

    print("Done.")
    ser.close()

if __name__ == "__main__":
    main()
