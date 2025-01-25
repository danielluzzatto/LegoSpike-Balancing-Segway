import runloop, motor_pair
from hub import port, motion_sensor
import time
import math

def get_gyro_rates():
    omega_x, omega_y, omega_z = motion_sensor.angular_velocity(False)
    return omega_x/10, omega_y/10, omega_z/10

def get_accel_angles():
    accel_x, accel_y, accel_z = motion_sensor.acceleration(False)
    roll_angle = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)) * 180 / math.pi# around z-axis
    pitch_angle = math.atan2(-accel_z, accel_x) * 180 / math.pi# around y-axis
    return pitch_angle+90, roll_angle

async def main():
    time.sleep(1)

    target_roll = 0
    power = 1  
    k_p = 35 
    k_i =2  
    k_d =40 

    error = 0
    integral = 0
    derivative = 0
    previous_error = 0

    tau = 0.5 
    fs = 70
    dt = 1 / fs
    a = tau / (tau + dt)

    pitch_angle_accel, roll_angle_accel = get_accel_angles()
    angle = roll_angle_accel

    motor_pair.unpair(motor_pair.PAIR_1)
    motor_pair.pair(motor_pair.PAIR_1, port.C, port.D)

    while True:
        gyro_rate_x, gyro_rate_y, gyro_rate_z = get_gyro_rates()
        pitch_angle_accel, roll_angle_accel = get_accel_angles()

        angle = a * (angle + gyro_rate_x * dt) + (1 - a) * roll_angle_accel
        error = -(target_roll - angle) #[deg]
        integral += error * dt #[deg*s]
        derivative = (error - previous_error) / dt #[deg/s]
        result_power = k_p * error + k_i * integral + k_d * derivative
        movement_speed = int(result_power * power)

        print(angle,movement_speed)
        motor_pair.move(motor_pair.PAIR_1, 0, velocity=movement_speed) # max speed = +-1110 deg/s
        previous_error = error
        await runloop.sleep_ms(int(dt * 1000))

runloop.run(main())

