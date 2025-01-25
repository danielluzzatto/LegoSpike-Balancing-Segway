from app import linegraph
import runloop, motor_pair, motor
from hub import port, motion_sensor
import time
import math

diam = 0.056

def get_gyro_rates():
    omega_x, omega_y, omega_z = motion_sensor.angular_velocity(False)
    return omega_x / 10, omega_y / 10, omega_z / 10

def get_accel_angles():
    accel_x, accel_y, accel_z = motion_sensor.acceleration(False)
    roll_angle = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)) * 180 / math.pi
    pitch_angle = math.atan2(-accel_z, accel_x) * 180 / math.pi
    return pitch_angle + 90, roll_angle

async def main():
    time.sleep(1)
    K = [1.0000,-1.4658,5.6345,1.0935]
    for i in range(len(K)):
        K[i] = pow(10,4)*K[i]
    print(K)
    fs = 50
    dt = 1 / fs
    motor_pair.unpair(motor_pair.PAIR_1)
    motor_pair.pair(motor_pair.PAIR_1, port.C, port.D)
    x = [0, 0, 0, 0] # State vector [position, linear velocity, angle, angular velocity]
    pitch_angle_accel, roll_angle_accel = get_accel_angles()
    angle = roll_angle_accel
    a = 0.5
    u = 0
    current_time = 0
    pos_init = motor.relative_position(2) * diam * dt / 2
    linegraph.clear_all()

    while True:
        gyro_rate_x, gyro_rate_y, gyro_rate_z = get_gyro_rates()
        pitch_angle, roll_angle = get_accel_angles()
        # Complementary filter
        angle = a * (angle + gyro_rate_x * dt) + (1 - a) * roll_angle

        lin_vel = motor.velocity(2) * diam * math.pi / 36 # Convert velocity to meters/sec
        pos = motor.relative_position(2) * diam * math.pi / 360 - pos_init

        x = [pos, lin_vel, angle, gyro_rate_x]
        u = -sum([K[i] * x[i] for i in range(4)]) #control law
        movement_speed = int(u)
        motor_pair.move(motor_pair.PAIR_1, 0, velocity=movement_speed)
        current_time +=dt
        linegraph.plot(9, current_time, angle)
        linegraph.plot(8, current_time, pos)
        linegraph.show(False)

        await runloop.sleep_ms(int(dt * 1000))

runloop.run(main())
