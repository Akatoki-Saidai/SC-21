import asyncio
from bleak import BleakClient

ADDRESS = "B8:27:EB:85:BC:AD"  # Raspberry PiのBluetoothアドレス
UUID = "5A2B2149-66DA-B159-EF44-35E31B1BC15F"  # サービスUUID
CHARACTERISTIC = "943B36FB-863B-2597-9B83-B68F6558C906"  # キャラクタリスティックUUID


async def notification_handler(sender, data):
    print(f"received: {data}")


async def run():
    async with BleakClient(ADDRESS) as client:
        # 接続を待つ
        while not await client.is_connected():
            print("Connecting...")
            await asyncio.sleep(0.5)  # 0.5秒間隔で接続を確認

        if client.is_connected():
            print("Connected!")
            await client.start_notify(CHARACTERISTIC, notification_handler)
            await asyncio.sleep(10.0)  # 10秒間通知を待機


asyncio.run(run())
