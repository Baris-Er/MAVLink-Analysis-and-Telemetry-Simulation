from pymavlink import mavutil 
import asyncio

px4_pymav_port = 14550


async def read_messages(connection):
    while True:
        msg = connection.recv_match(blocking=False)
        if msg:
            print(msg)
        await asyncio.sleep(0.001)

async def main():
    connection_string = 'udpin:localhost:%d' %px4_pymav_port
    pymav_analysis_port = mavutil.mavlink_connection(connection_string)
    #pymav_analysis_port.wait_heartbeat()       #not necessarily needed but good to use 
    await asyncio.gather(read_messages(pymav_analysis_port))
asyncio.run(main())