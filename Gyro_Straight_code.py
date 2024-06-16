"""
This won't work in any place other than spike prime itself
"""

from hub import light_matrix ,motion_sensor, port, light
import runloop, motor_pair, motor, color
import time 

    degrees_per_cm = 360/27.6
    def initiate():
        motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
        motion_sensor.set_yaw_face(motion_sensor.TOP)
        motion_sensor.reset_yaw(0)
        motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)
    def straight(distance=100):
        motion_sensor.reset_yaw(0)
        motor.reset_relative_position(port.A, 0)
        motor.reset_relative_position(port.E, 0)
        distance_deg = abs(distance*degrees_per_cm)
        time.sleep_ms(1000)
        dist_target_deg = distance_deg
        speed = 500.0
        if distance < 0:
            speed = -speed
        kp = 3.5
        while distance_deg>0:
            yaw = motion_sensor.tilt_angles()[0]
            speed_correction= float(yaw) * kp
            if distance > 0:
                distance_deg = int(dist_target_deg- int((-motor.relative_position(port.A)+motor.relative_position(port.E))/2))
            else:
                distance_deg = int(dist_target_deg- int((motor.relative_position(port.A)-motor.relative_position(port.E))/2))
            motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, distance_deg, int(speed+speed_correction), int(speed-speed_correction), acceleration=10000, deceleration=10000)
        time.sleep(.05)

    initiate()
    straight(200)
    straight(-200)