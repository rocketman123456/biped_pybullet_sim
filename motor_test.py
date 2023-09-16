import math
import moteus
import asyncio
import time
import sys
sys.path.append('./walking')


DEBUG = False


class Bipedal:
    def __init__(self):
        self.zero_position = []
        self.current_position = []
        self.motors = []
        qr = moteus.QueryResolution()
        qr._extra = {
            moteus.Register.CONTROL_POSITION: moteus.F32,
            moteus.Register.CONTROL_VELOCITY: moteus.F32,
            moteus.Register.CONTROL_TORQUE: moteus.F32,
            moteus.Register.POSITION_ERROR: moteus.F32,
            moteus.Register.VELOCITY_ERROR: moteus.F32,
            moteus.Register.TORQUE_ERROR: moteus.F32,
        }

        for i in range(5, 10):
            motor = moteus.Controller(query_resolution=qr, id=i+1)
            self.motors.append(motor)
            pass

        for i in range(0, 5):
            motor = moteus.Controller(query_resolution=qr, id=i+1)
            self.motors.append(motor)
            pass

    async def get_motor_position(self, motor):
        state = await motor.set_position(position=math.nan, query=True)
        return state.values[moteus.Register.POSITION]

    async def get_motor_velocity(self, motor):
        state = await motor.set_position(position=math.nan, query=True)
        return state.values[moteus.Register.VELOCITY]

    async def get_motor_torque(self, motor):
        state = await motor.set_position(position=math.nan, query=True)
        return state.values[moteus.Register.TORQUE]

    async def send_position_command(self, controller, position_degrees):
        await controller.set_position(position=position_degrees, query=True)
        # print(f"Position command sent to motor {controller.address} to rotate to {position_degrees} degrees.")

    async def set_bipedal_joint(self, position):
        motor_position = []
        motor_position.append(position[4])
        motor_position.append(position[2] + position[3])
        motor_position.append(-position[2])
        motor_position.append(-position[1])
        motor_position.append(-position[0])
        motor_position.append(position[9])
        motor_position.append(position[7] + position[8])
        motor_position.append(-position[7])
        motor_position.append(-position[6])
        motor_position.append(-position[5])
        # print(motor_position)
        await self.send_position_command(self.motors[0], (motor_position[0]/(2*math.pi)*20 + self.zero_position[0]))
        await self.send_position_command(self.motors[1], (motor_position[1]/(2*math.pi) + self.zero_position[1]))
        await self.send_position_command(self.motors[2], (motor_position[2]/(2*math.pi) + self.zero_position[2]))
        await self.send_position_command(self.motors[3], (motor_position[3]/(2*math.pi) + self.zero_position[3]))
        await self.send_position_command(self.motors[4], (motor_position[4]/(2*math.pi) + self.zero_position[4]))
        await self.send_position_command(self.motors[5], (motor_position[5]/(2*math.pi)*20 + self.zero_position[5]))
        await self.send_position_command(self.motors[6], (motor_position[6]/(2*math.pi) + self.zero_position[6]))
        await self.send_position_command(self.motors[7], (motor_position[7]/(2*math.pi) + self.zero_position[7]))
        await self.send_position_command(self.motors[8], (motor_position[8]/(2*math.pi) + self.zero_position[8]))
        await self.send_position_command(self.motors[9], (motor_position[9]/(2*math.pi) + self.zero_position[9]))

    async def main(self):

        # 获取初始电机位置
        for motor in self.motors:
            zero_pos = await self.get_motor_position(motor)
            self.zero_position.append(zero_pos)
        if DEBUG:
            print("\n->zero_pos is: ", zero_pos)

        # 设置关节位置
        for i in range(1000):
            position = [0.00, 0.0, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 + math.pi/4 * i * 0.001]
            await self.set_bipedal_joint(position)
            await asyncio.sleep(0.005)


if __name__ == '__main__':
    bipedal_instance = Bipedal()
    asyncio.run(bipedal_instance.main())
    print(bipedal_instance.zero_position)
