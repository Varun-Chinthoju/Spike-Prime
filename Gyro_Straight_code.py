"""
This won't work in any place other than spike prime itself
"""

from hub import light_matrix ,motion_sensor, port, light
import runloop, motor_pair, motor, color, time
from app import linegraph, music, sound

time_prev = time.ticks_us() - 1
# time_t = 
integral = 0
e_prev = 0

Kp = 4.75
Ki = 0.70
Kd = 5.0

linegraph.clear_all()


def PID(e):
    print("err: ", e/10)
    global integral, time_prev, e_prev
    time_t = time.ticks_us()
    print("time delta: ", (time_t - time_prev)/1000000.0)
    # PID calculation
    P = Kp*(e/10)
    # print(time.ticks_ms())
    integral = integral + Ki*e*(time_t - time_prev)/1000000.0
    print("integral: ", integral)
    
    D = 1000.0 * Kd*(e - e_prev)/(time_t - time_prev)
    # D = 0
    print("d:", D)
    # calculate manipulated variable - MV
    MV = P + integral + D
    linegraph.plot(color.BLUE, time.ticks_us(), D)
    linegraph.plot(color.RED, time.ticks_us(), P)
    linegraph.plot(color.GREEN, time.ticks_us(), MV)
    linegraph.plot(color.BLACK, time.ticks_us(), e/10)
    print("MV: ", MV)

    # update stored data for next iteration
    e_prev = e
    time_prev = time_t
    return MV

async def main():
    degrees_per_cm = 360/27.6
    def initiate():
        motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
        motion_sensor.set_yaw_face(motion_sensor.TOP)
        motion_sensor.reset_yaw(0)
        motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)
        motor.reset_relative_position(port.A, 0)
        motor.reset_relative_position(port.E, 0)

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
        #kp = 2
        while distance_deg>0:
            err = motion_sensor.tilt_angles()[0]
            correction = PID(float(err))
            if distance > 0:
                distance_deg = int(dist_target_deg- int((-motor.relative_position(port.A)+motor.relative_position(port.E))/2))
            else:
                distance_deg = int(dist_target_deg- int((motor.relative_position(port.A)-motor.relative_position(port.E))/2))
            motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, distance_deg, int(speed+correction), int(speed-correction), acceleration=1000, deceleration=2000)
        #time.sleep(.05)

    initiate()
    straight(200)
    straight(-200)

    
runloop.run(main())
