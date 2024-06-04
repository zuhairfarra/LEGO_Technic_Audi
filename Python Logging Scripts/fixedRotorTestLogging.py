# SPDX-License-Identifier: MIT
# Copyright (c) 2020 Henrik Blidh
# Copyright (c) 2022-2023 The Pybricks Authors

"""
Program for computer-to-hub communication during driving forward event.

Requires Pybricks firmware >= 3.3.0.
"""

import asyncio
from contextlib import suppress
from bleak import BleakScanner, BleakClient
import matplotlib.pyplot as plt

PYBRICKS_COMMAND_EVENT_CHAR_UUID = "c5f50002-8280-46da-89f4-6d8051e4aeef"

# Replace this with the name of your hub if you changed
# it when installing the Pybricks firmware.
HUB_NAME = "Audi_Technic_SF"

data_array = list()

async def main():
    main_task = asyncio.current_task()

    def handle_disconnect(_):
        print("Hub was disconnected.")

        # If the hub disconnects before this program is done,
        # cancel this program so it doesn't get stuck waiting
        # forever.
        if not main_task.done():
            main_task.cancel()

    ready_event = asyncio.Event()

    def handle_rx(_, data: bytearray):
        global data_array

        if data[0] == 0x01:  # "write stdout" event (0x01)
            payload = data[1:]

            if payload == b"rdy":
                ready_event.set()
            elif payload == b"bye":
                print("Hub disconnecting...")
            else:
                print("Received:", payload)
                data_array.append(collect_data(data_array,payload))

    # Do a Bluetooth scan to find the hub.
    device = await BleakScanner.find_device_by_name(HUB_NAME)

    if device is None:
        print(f"could not find hub with name: {HUB_NAME}")
        return

    # Connect to the hub.
    async with BleakClient(device, handle_disconnect) as client:

        # Subscribe to notifications from the hub.
        await client.start_notify(PYBRICKS_COMMAND_EVENT_CHAR_UUID, handle_rx)

        # Shorthand for sending some data to the hub.
        async def send(data):
            print("In send")
            # Wait for hub to say that it is ready to receive data.
            await ready_event.wait()
            print("Ready over")
            # Prepare for the next ready event.
            ready_event.clear()
            # Send the data to the hub.
            await client.write_gatt_char(
                PYBRICKS_COMMAND_EVENT_CHAR_UUID,
                b"\x06" + data,  # prepend "write stdin" command (0x06)
                response=True
            )
            print("Write over")

        def collect_data(dataArray,dataPayload:bytearray):
            payloadString = dataPayload.decode('utf-8')
            payloadList = payloadString.split(",")
            print(payloadList)
            mytime = float(payloadList[0])
            mycurrent = float(payloadList[1])
            return [mytime,mycurrent]

        # Tell user to start program on the hub.
        print("Start the program on the hub now with the button.")

        # Send a few messages to the hub.
        for i in range(1):
            await send(b"s")
            print("Send s")
            await asyncio.sleep(28)
            print(".", end="", flush=True)

        # Send a message to indicate stop.
        await send(b"x")
        print(data_array)
        # Save the list to a text file
        with open('fixedrotordata.txt', 'w') as file:
            for row in data_array:
                file.write('\t'.join(map(str, row)) + '\n')

        print("done.\n")
    # Hub disconnects here when async with block exits.

# Run the main async program.
if __name__ == "__main__":
    with suppress(asyncio.CancelledError):
        asyncio.run(main())

