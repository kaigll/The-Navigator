import asyncio
import os
from bleak import BleakClient
import requests
import http.server
import socketserver
import threading

address = "c8:27:3a:cd:71:63"
MODEL_NBR_UUID = "1337"
FIRMWARE_UUID = (
    "abcd"  # Replace with the actual UUID for your firmware update characteristic
)

# Define the relative path to the firmware file
current_dir = os.path.dirname(__file__)
firmware_relative_path = os.path.join(
    current_dir,
    "..",
    "arduino-code",
    ".pio",
    "build",
    "nano33ble",
    "firmware.bin",
)
FIRMWARE_PATH = os.path.abspath(firmware_relative_path)
FIRMWARE_URL = "http://localhost:8000/firmware.bin"

print("Firmware Path:", FIRMWARE_PATH)  # Print the constructed path


# Serve the firmware file
def serve_firmware():
    os.chdir(
        os.path.dirname(FIRMWARE_PATH)
    )  # Change directory to where the firmware is located
    handler = http.server.SimpleHTTPRequestHandler
    httpd = socketserver.TCPServer(("", 8000), handler)
    print("Serving at port", 8000)
    httpd.serve_forever()


# Function to download firmware
async def download_firmware():
    response = requests.get(FIRMWARE_URL)
    if response.status_code == 200:
        with open("firmware.bin", "wb") as f:
            f.write(response.content)
        return True
    return False


# Function to update firmware
async def update_firmware(client):
    with open("firmware.bin", "rb") as f:
        firmware_data = f.read()
        await client.write_gatt_char(FIRMWARE_UUID, firmware_data)


# Main function
async def main(address):
    # Start the firmware server in a new thread
    server_thread = threading.Thread(target=serve_firmware)
    server_thread.daemon = True
    server_thread.start()

    async with BleakClient(address) as client:
        model_number = await client.read_gatt_char(MODEL_NBR_UUID)
        print("Model Number: {0}".format("".join(map(chr, model_number))))

        # Integrate FOTA functionality
        if await download_firmware():
            print("Firmware downloaded successfully.")
            await update_firmware(client)
            print("Firmware update completed successfully.")
        else:
            print("Failed to download firmware.")


asyncio.run(main(address))
