import asyncio
from bleak import BleakClient

address = "c8:27:3a:cd:71:63"
MODEL_NBR_UUID = "1337"

async def main(address):
    async with BleakClient(address) as client:
        model_number = await client.read_gatt_char(MODEL_NBR_UUID)
        print("Model Number: {0}".format("".join(map(chr, model_number))))

asyncio.run(main(address))