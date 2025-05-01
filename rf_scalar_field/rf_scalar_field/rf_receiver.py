from digi.xbee.devices import XBeeDevice, XBeeMessage

PORT = "/dev/ttyUSB1" 
BAUD_RATE = 115200
device = XBeeDevice(PORT, BAUD_RATE)
latest_message = None

def handle_rx_message(xbee_message):
    global latest_message
    latest_message = xbee_message

def main():

    prev_message = None

    try:
        device.open()
        print("Device opened. Waiting for messages...")

        # Register the callback
        device.add_data_received_callback(handle_rx_message)

        while True:
            # Wait for a message to be received
            if latest_message and latest_message != prev_message:
                print("\nReceived:", latest_message.data.decode())
                print("From:", latest_message.remote_device.get_64bit_addr())
                print("Timestamp:", latest_message.timestamp)

                try:
                    if device.is_open():
                        rssi_bytes = device.get_parameter("DB")
                        if rssi_bytes:
                            rssi = int.from_bytes(rssi_bytes)
                            print(f"RSSI of last received packet: -{rssi} dBm")
                        else:
                            print("DB command returned no value.")
                    else:
                        print("Device closed before DB could be read.")
                except Exception as e:
                    print("Error retrieving RSSI:", e)

                prev_message = latest_message

    except KeyboardInterrupt:
        print("Interrupted by user. Exiting.")

    finally:
        if device.is_open():
            device.close()
            print("Device closed.")

if __name__ == "__main__":
    main()
