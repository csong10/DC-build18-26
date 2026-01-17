from bluepy.btle import Scanner
import time

# ===================== CONFIG ===================== #

# Put your Locate Beacon UUID here, with NO dashes, all lowercase.
# Example: if the app shows:
#   E2C56DB5-DFFB-48D2-B060-D0F5A71096E0
# you write:
#   "e2c56db5dffb48d2b060d0f5a71096e0"
TARGET_UUID = "dc672026bd1866667777674206742067".lower()

# How many quick scans to average together for smoothing
SAMPLES_PER_READING = 2

# Path loss exponent (2.0 = open space, 3–4 = indoors/cluttered)
PATH_LOSS_N = 2.0

# Direction scan settings
NUM_HEADING_STEPS = 8          # how many directions per sweep
SCAN_ARC_DEGREES = 360         # 360 for full circle, 180 for front half
TURN_TIME_PER_90_DEG = 0.4     # seconds of rotate_in_place ≈ 90° (tune this)

# ===================== MOTOR / ROTATION STUB ===================== #
# Replace this with real motor control if you have it wired.

def rotate_in_place(direction: str, degrees: float):
    """
    Rotate the robot in place.
    direction: 'left' or 'right'
    degrees: how much to rotate (approximate).
    Currently just sleeps; replace with motor code when ready.
    """
    print(f"[ROTATE] {direction} ~{degrees:.1f}°")
    # TODO: your motor driver code goes here:
    #   - set left/right wheel speeds opposite directions
    #   - compute duration from degrees
    # For now, approximate timing:
    #   TURN_TIME_PER_90_DEG seconds ≈ 90 degrees
    if degrees <= 0:
        return
    duration = (abs(degrees) / 90.0) * TURN_TIME_PER_90_DEG
    time.sleep(duration)
    # stop motors here when you have them


# ===================== iBEACON PARSING ===================== #

def parse_ibeacon(scan_entry):
    """
    Extract iBeacon (uuid, major, minor, tx_power) from Manufacturer data.
    Returns None if not an iBeacon.
    """
    for (adtype, desc, value) in scan_entry.getScanData():
        if desc == "Manufacturer":
            raw = value.lower()
            # iBeacon prefix for Apple: 0x4c00 0x02 0x15 -> "4c000215"
            if raw.startswith("4c000215") and len(raw) >= 50:
                uuid = raw[8:40]      # 16 bytes (32 hex chars)
                major = int(raw[40:44], 16)
                minor = int(raw[44:48], 16)
                tx_power = int(raw[48:50], 16)
                # tx_power is signed int8
                if tx_power > 127:
                    tx_power -= 256
                return uuid, major, minor, tx_power
    return None

def estimate_distance(rssi: float, tx_power: int, n: float = PATH_LOSS_N):
    """
    Rough distance estimate using log-distance path loss model:
        d = 10 ^ ((TxPower - RSSI) / (10 * n))

    This is VERY approximate and will be noisy.
    """
    # Avoid divide-by-zero / weird values
    if rssi == 0:
        return None
    ratio_db = tx_power - rssi
    distance = 10 ** (ratio_db / (10 * n))
    return distance

# ===================== BASIC RSSI/DISTANCE READING ===================== #

def get_beacon_rssi_and_distance(scanner: Scanner):
    rssi_values = []
    tx_power_values = []

    for _ in range(SAMPLES_PER_READING):
        devices = scanner.scan(0.1)  # 0.1s scan window
        for dev in devices:
            data = parse_ibeacon(dev)
            if data:
                uuid, major, minor, tx_power = data
                if uuid == TARGET_UUID:
                    rssi_values.append(dev.rssi)
                    tx_power_values.append(tx_power)
        time.sleep(0.05)

    if not rssi_values:
        return None, None, None

    avg_rssi = sum(rssi_values) / len(rssi_values)
    avg_tx_power = int(sum(tx_power_values) / len(tx_power_values))

    distance = estimate_distance(avg_rssi, avg_tx_power)

    return avg_rssi, avg_tx_power, distance

# ===================== HEADING / ANGLE ESTIMATION ===================== #

def find_beacon_heading(scanner: Scanner):
    """
    Rotate the robot in NUM_HEADING_STEPS steps over SCAN_ARC_DEGREES,
    measure RSSI at each step, and return the angle (relative to start)
    that has the strongest signal.
    """
    step_angle = SCAN_ARC_DEGREES / NUM_HEADING_STEPS
    angle_rssi = []

    print(f"[HEADING] Starting heading scan over {SCAN_ARC_DEGREES}° "
          f"in {NUM_HEADING_STEPS} steps (~{step_angle:.1f}° each).")

    # Start at "0°" (whatever direction the robot is currently facing)
    current_angle = 0.0

    for i in range(NUM_HEADING_STEPS):
        # At each orientation, measure RSSI a few times
        rssi_values = []
        tx_values = []

        for _ in range(max(3, SAMPLES_PER_READING // 2)):  # a few samples per angle
            avg_rssi, avg_tx_power, _ = get_beacon_rssi_and_distance(scanner)
            if avg_rssi is not None:
                rssi_values.append(avg_rssi)
                tx_values.append(avg_tx_power)
            time.sleep(0.05)

        if rssi_values:
            angle_avg_rssi = sum(rssi_values) / len(rssi_values)
        else:
            angle_avg_rssi = -999  # treat as very weak / not found

        angle_rssi.append((current_angle, angle_avg_rssi))
        print(f"[HEADING] angle={current_angle:.0f}°, RSSI={angle_avg_rssi:.1f} dBm")

        # Rotate to next step unless this is the last one
        if i < NUM_HEADING_STEPS - 1:
            rotate_in_place("right", step_angle)
            current_angle += step_angle

    # Find angle with strongest RSSI (highest, least negative)
    best_angle, best_rssi = max(angle_rssi, key=lambda x: x[1])
    print(f"[HEADING] Best angle ≈ {best_angle:.0f}° (RSSI {best_rssi:.1f} dBm)")

    # OPTIONAL: rotate back to that angle relative to where we ended
    # Right now we end at SCAN_ARC_DEGREES; to face best_angle again:
    # difference = SCAN_ARC_DEGREES - best_angle
    # rotate_in_place("left", difference)

    return best_angle, best_rssi

# ===================== MAIN LOOP ===================== #

def main():
    print("[INFO] Starting BLE scanner...")
    scanner = Scanner()

    while True:
        # 1) Get a basic distance reading
        avg_rssi, avg_tx_power, distance = get_beacon_rssi_and_distance(scanner)

        if avg_rssi is None:
            print("[INFO] Beacon not found. Is the app broadcasting and screen on?")
        else:
            # Format distance nicely
            if distance is None:
                dist_str = "unknown"
            else:
                dist_str = f"{distance:.2f} m"

            print(
                f"[BEACON] RSSI: {avg_rssi:.1f} dBm | "
                f"TxPower: {avg_tx_power} dBm | "
                f"Estimated distance: {dist_str}"
            )

            # 2) Run a heading scan to find direction of maximum RSSI
            best_angle, best_rssi = find_beacon_heading(scanner)
            print(
                f"[RESULT] Estimated best heading: {best_angle:.0f}° "
                f"(relative to starting orientation), RSSI={best_rssi:.1f} dBm"
            )

        # Wait a bit before the next full cycle
        print("-" * 50)
        time.sleep(1.0)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
