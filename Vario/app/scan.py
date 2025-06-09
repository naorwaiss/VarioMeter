# import asyncio
# from bleak import BleakClient

# address = "F0:F5:BD:2C:F6:DA"  # Replace with your device
# CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"  # Replace with correct characteristic UUID

# async def main():
#     async with BleakClient(address) as client:
#         print("Connected:", await client.is_connected())
#         value = await client.read_gatt_char(CHAR_UUID)
#         print("Value:", value)

# asyncio.run(main())



import asyncio
from bleak import BleakClient, BleakError

address = "F0:F5:BD:2C:F6:DA"  # Your device MAC
CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"  # Your characteristic UUID

async def read_loop():
    while True:
        try:
            print(f"Trying to connect to {address}...")
            async with BleakClient(address) as client:
                if await client.is_connected():
                    print("✅ Connected!")
                else:
                    print("❌ Failed to connect")
                    continue

                while await client.is_connected():
                    try:
                        value = await client.read_gatt_char(CHAR_UUID)
                        print("📦 Value:", value)
                    except Exception as e:
                        print("⚠️ Read error:", e)
                        break

        except BleakError as e:
            print("🔌 Connection error:", e)

asyncio.run(read_loop())
