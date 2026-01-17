from bluepy.btle import Scanner
import time

# ===================== CONFIG ===================== #

# Put your Locate Beacon UUID here, with NO dashes, all lowercase.
# Example: if the app shows:
#   E2C56DB5-DFFB-48D2-B060-D0F5A71096E0
# you write:
#   "e2c56db5dffb48d2b060d0f5a71096e0"
TARGET_UUID = "DC672026BD1866667777674206742067".lower()

# How many quick scans to average together for smoothing
SAMPLES_PER_READING = 5

# Path loss exponent (2.0 = open space, 3–4 = indoors/cluttered)
PATH_LOSS_N = 2.0

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

# ===================== MAIN SCANNING LOGIC ===================== #

def get_beacon_rssi_and_distance(scanner: Scanner):
    rssi_values = []
    tx_power_values = []

    for _ in range(SAMPLES_PER_READING):
        devices = scanner.scan(0.4)  # 0.4s scan window
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

def main():
    print("[INFO] Starting BLE scanner...")
    scanner = Scanner()

    while True:
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

        time.sleep(0.5)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
