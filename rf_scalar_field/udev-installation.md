# USB Device Persistent Naming Setup (XBee Modem)

This guide shows how to assign **persistent, human-readable names** to your USB devices (e.g., GPS and XBee) on Linux using udev rules. This ensures that your devices always appear as `/dev/gps` and `/dev/xbee`, regardless of the order they are plugged in.

---

## 1. Identify the USB Device IDs

### Step 1.1: Plug in the device
Connect the USB device (GPS or XBee) to your computer.

### Step 1.2: List USB devices
```bash
lsusb
```
In the example output below
```bash
Bus 004 Device 003: ID 0403:6001 Future Technology Devices International, Ltd FT232 Serial (UART) IC
```
The `0403` corresponds to the idVendor and the `6001` corresponds to the idProduct.

### Step 1.3 Determine Serial Number
Run the following again
```bash
# Replace vendor:product with your device
lsusb -v -d 0403:6001 | grep -i serial
```
to find the serial number. An example output would be
```
iSerial                 3 A28G60W7
```
## 2. Update the UDev Rules File
Under the file under the `/udev` directory, modify the file
```
# XBee device (FT232, serial-specific)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="A28G60W7", SYMLINK+="xbee", MODE="0666"

```

### 3. Copy file in correct location and update dev rules
Copy that file to the `/etc/udev/rules.d` directory, and perform the following commands
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
```
