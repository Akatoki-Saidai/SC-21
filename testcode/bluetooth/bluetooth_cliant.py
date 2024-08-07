import asyncio
from bleak import BleakClient

ADDRESS = "B8:27:EB:85:BC:AD"  # Raspberry PiのBluetoothアドレス
UUID = "5A2B2149-66DA-B159-EF44-35E31B1BC15F"  # サービスUUID
CHARACTERISTIC = "943B36FB-863B-2597-9B83-B68F6558C906"  # キャラクタリスティックUUID

async def notification_handler(sender, data):
    print(f"received: {data}")

async def run():
    async with BleakClient(ADDRESS) as client:
        await client.start_notify(CHARACTERISTIC, notification_handler)
        await asyncio.sleep(10.0)

asyncio.run(run())
