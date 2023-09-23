import math
import moteus
import asyncio
import time
import sys

qr = moteus.QueryResolution()
qr._extra = {
    moteus.Register.CONTROL_POSITION: moteus.F32,
    moteus.Register.CONTROL_VELOCITY: moteus.F32,
    moteus.Register.CONTROL_TORQUE: moteus.F32,
    moteus.Register.POSITION_ERROR: moteus.F32,
    moteus.Register.VELOCITY_ERROR: moteus.F32,
    moteus.Register.TORQUE_ERROR: moteus.F32,
}

motor = moteus.Controller(query_resolution=qr, id=1)

async def send_position_command(controller, position_degrees):
    await controller.set_position(position=position_degrees, query=True)

send_position_command(motor, math.pi/6.0)

print(motor)
