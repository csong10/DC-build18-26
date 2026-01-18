# Raspberry Pi Installation Guide for RC Car

This guide walks you through everything you need to install and configure on your Raspberry Pi to run the threaded RC car system.

---

## Step 1: Update Your System

```bash
sudo apt-get update
sudo apt-get upgrade -y
```

---

## Step 2: Install System Packages

### Install OpenCV and Camera Support
```bash
sudo apt-get install -y python3-opencv python3-picamera2
```

### Install Python Development Tools
```bash
sudo apt-get install -y python3-pip python3-dev build-essential
```

### Install Bluetooth Libraries
```bash
sudo apt-get install -y libbluetooth-dev libglib2.0-dev
```

---

## Step 3: Install Python Packages

### Install Required Python Libraries
```bash
pip3 install bluepy --break-system-packages
pip3 install pyserial --break-system-packages
pip3 install numpy --break-system-packages
```

**Note:** The `--break-system-packages` flag is needed on newer Raspberry Pi OS versions.

---

## Step 4: Download Haar Cascade File

This file is needed for face detection:

```bash
cd ~
wget https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_default.xml
```

**Important:** Put this file in the same directory as your `rc_car_threaded.py` script!

---

## Step 5: Enable Camera

### Option A: Using raspi-config (Recommended)
```bash
sudo raspi-config
```
- Navigate to: **Interface Options** → **Camera** → **Enable**
- Select **Finish** and reboot

### Option B: Manual Configuration
Edit `/boot/config.txt`:
```bash
sudo nano /boot/config.txt
```
Add this line:
```
start_x=1
```
Save (Ctrl+X, Y, Enter) and reboot:
```bash
sudo reboot
```

---

## Step 6: Enable UART for STM32 Communication

### Edit config.txt
```bash
sudo nano /boot/config.txt
```

Add this line at the end:
```
enable_uart=1
```

Save and exit (Ctrl+X, Y, Enter)

### Disable Serial Console

Edit cmdline.txt:
```bash
sudo nano /boot/cmdline.txt
```

**Remove** this part if it exists:
```
console=serial0,115200
```

The line should look something like this (all on one line):
```
console=tty1 root=PARTUUID=xxxxxxxx-xx rootfstype=ext4 fsck.repair=yes rootwait
```

Save and exit (Ctrl+X, Y, Enter)

### Reboot
```bash
sudo reboot
```

### Verify UART is Working
After reboot, check if UART is available:
```bash
ls -l /dev/ttyAMA0
```
You should see something like:
```
crw-rw---- 1 root dialout 204, 64 Jan 17 10:30 /dev/ttyAMA0
```

---

## Step 7: Add Your User to Bluetooth Group

This allows running Bluetooth without sudo (optional but recommended):
```bash
sudo usermod -a -G bluetooth $USER
```

Log out and log back in for this to take effect.

---

## Step 8: Wire the Hardware

### Raspberry Pi to STM32 UART Connection
```
┌─────────────────┐         ┌─────────────────┐
│  Raspberry Pi   │         │     STM32       │
├─────────────────┤         ├─────────────────┤
│ GPIO 14 (TX)    │────────▶│ RX              │
│ GPIO 15 (RX)    │◀────────│ TX              │
│ GND             │────────▶│ GND             │
└─────────────────┘         └─────────────────┘
```

**Pin Locations on Raspberry Pi (Physical Pins):**
- Pin 8: GPIO 14 (TX)
- Pin 10: GPIO 15 (RX)
- Pin 6, 9, 14, 20, 25, 30, 34, or 39: GND (any will work)

### Camera Connection
- Connect the Raspberry Pi Camera to the camera port (ribbon cable)
- Make sure it clicks into place securely

---

## Step 9: Set Up Your Project Files

### Create a project directory:
```bash
mkdir ~/rc_car_project
cd ~/rc_car_project
```

### Copy your files:
1. Copy `rc_car_threaded.py` to this directory
2. Copy `haarcascade_frontalface_default.xml` to this directory
3. Copy `test_components.py` to this directory (optional, for testing)

### Verify files are in place:
```bash
ls -l
```
You should see:
```
rc_car_threaded.py
haarcascade_frontalface_default.xml
test_components.py
```

---

## Step 10: Configure Your Beacon UUID

Edit the script to match your Bluetooth beacon:
```bash
nano rc_car_threaded.py
```

Find this line near the top:
```python
TARGET_UUID = "DC672026BD1866667777674206742067".lower()
```

Replace it with YOUR beacon's UUID (remove all dashes, make lowercase).

For example, if your beacon UUID is:
```
E2C56DB5-DFFB-48D2-B060-D0F5A71096E0
```

Change the line to:
```python
TARGET_UUID = "e2c56db5dffb48d2b060d0f5a71096e0".lower()
```

Save and exit (Ctrl+X, Y, Enter)

---

## Step 11: Test Individual Components

Before running the full system, test each component:

```bash
sudo python3 test_components.py
```

Select each test one by one:
1. Test Threading Basics (should always work)
2. Test Bluetooth Scanner (make sure beacon is on!)
3. Test Camera & Face Detection (look at the camera)
4. Test UART Communication (STM32 must be connected and powered)

**All tests should pass before proceeding!**

---

## Step 12: Run the RC Car System

Once all tests pass:

```bash
sudo python3 rc_car_threaded.py
```

**Note:** `sudo` is required for Bluetooth scanning.

You should see:
```
[INIT] Initializing components...
[INIT] UART initialized on /dev/ttyAMA0 at 115200 baud
[INIT] Bluetooth scanner initialized
[INIT] Camera initialized with face detection
[INIT] All components initialized

============================================================
RC CAR SYSTEM RUNNING
============================================================
Priority: 1. Avoid Obstacles  2. Follow Beacon  3. Search
Press Ctrl+C to stop
============================================================
```

---

## Troubleshooting Common Issues

### Bluetooth "Operation not permitted"
**Solution:** Run with `sudo`:
```bash
sudo python3 rc_car_threaded.py
```

### "bluepy" installation fails
**Solution:** Install dependencies first:
```bash
sudo apt-get install -y libbluetooth-dev libglib2.0-dev
pip3 install bluepy --break-system-packages
```

### Camera not detected
**Solution:** 
1. Check camera is enabled: `sudo raspi-config` → Interface Options → Camera
2. Check camera connection (ribbon cable)
3. Reboot: `sudo reboot`
4. Test: `libcamera-hello` (should show camera preview)

### Haar cascade XML not found
**Solution:** Make sure the XML file is in the same directory as the Python script:
```bash
ls haarcascade_frontalface_default.xml
```
If missing, download again:
```bash
wget https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_default.xml
```

### UART not working / STM32 not responding
**Solution:**
1. Verify UART is enabled: `ls -l /dev/ttyAMA0`
2. Check wiring (TX→RX, RX→TX, GND→GND)
3. Verify baud rate matches STM32 (default: 115200)
4. Test with minicom: `sudo minicom -D /dev/ttyAMA0 -b 115200`

### "Permission denied" on /dev/ttyAMA0
**Solution:** Add your user to dialout group:
```bash
sudo usermod -a -G dialout $USER
```
Then log out and log back in.

---

## Quick Reference Commands

### Start the RC Car:
```bash
cd ~/rc_car_project
sudo python3 rc_car_threaded.py
```

### Stop the RC Car:
Press `Ctrl+C`

### Test Components:
```bash
sudo python3 test_components.py
```

### Check System Status:
```bash
# Check UART
ls -l /dev/ttyAMA0

# Check Camera
libcamera-hello

# Check Bluetooth
sudo hciconfig
```

---

## Optional: Auto-Start on Boot

If you want the RC car to start automatically when the Pi boots:

### Create a systemd service:
```bash
sudo nano /etc/systemd/system/rc-car.service
```

Add this content:
```ini
[Unit]
Description=RC Car Threaded Control
After=multi-user.target

[Service]
Type=simple
User=root
WorkingDirectory=/home/pi/rc_car_project
ExecStart=/usr/bin/python3 /home/pi/rc_car_project/rc_car_threaded.py
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

### Enable and start the service:
```bash
sudo systemctl daemon-reload
sudo systemctl enable rc-car.service
sudo systemctl start rc-car.service
```

### Check status:
```bash
sudo systemctl status rc-car.service
```

### View logs:
```bash
sudo journalctl -u rc-car.service -f
```

### Disable auto-start:
```bash
sudo systemctl disable rc-car.service
sudo systemctl stop rc-car.service
```

---

## Summary Checklist

- [ ] System updated (`sudo apt-get update && sudo apt-get upgrade`)
- [ ] OpenCV installed (`python3-opencv python3-picamera2`)
- [ ] Python packages installed (`bluepy pyserial numpy`)
- [ ] Haar cascade XML downloaded
- [ ] Camera enabled via `raspi-config`
- [ ] UART enabled in `/boot/config.txt`
- [ ] Serial console disabled in `/boot/cmdline.txt`
- [ ] Raspberry Pi rebooted
- [ ] STM32 wired correctly (TX→RX, RX→TX, GND→GND)
- [ ] Beacon UUID configured in script
- [ ] All component tests passed
- [ ] STM32 programmed to receive UART commands

Once all items are checked, you're ready to run the system!

---

**Need Help?** Refer to the main README.md for detailed explanations and customization options.
