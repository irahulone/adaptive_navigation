from digi.xbee.devices import XBeeDevice
import threading
import time

PORT = "/dev/ttyUSB1" 
BAUD_RATE = 9600

device = XBeeDevice(PORT, BAUD_RATE)

# Shared variable to flag a message was received
message_event = threading.Event()
latest_message = None

def handle_rx_message(xbee_message):
    global latest_message
    latest_message = xbee_message
    message_event.set()  # Signal to main thread

def main():
    try:
        device.open()
        print("Device opened. Waiting for messages...")

        # Register the callback
        device.add_data_received_callback(handle_rx_message)

        # Wait for a message to be received
        message_event.wait(timeout=1.0)

        if message_event.is_set():
            message_event.clear()  # Reset flag

            if latest_message:
                print("\nReceived:", latest_message.data.decode())
                print("From:", latest_message.remote_device.get_64bit_addr())

                try:
                    if device.is_open():
                        rssi_bytes = device.get_parameter("DB")
                        if rssi_bytes:
                            rssi = int.from_bytes(rssi_bytes, byteorder="big")
                            print(f"RSSI of last received packet: -{rssi} dBm")
                        else:
                            print("DB command returned no value.")
                    else:
                        print("Device closed before DB could be read.")
                except Exception as e:
                    print("Error retrieving RSSI:", e)

    except KeyboardInterrupt:
        print("Interrupted by user. Exiting.")

    finally:
        if device.is_open():
            device.close()
            print("Device closed.")

if __name__ == "__main__":
    main()
