from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy


GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):

    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel,
                 max_steer_angle, decel_limit, accel_limit, wheel_radius, brake_deadband, total_mass):
        #throttle PID parameters
        kp = 50
        ki = 10
        kd = 10

        # these parameters are also affected from the frequency of the node
        tau = 0.08
        ts = 0.02

        self.steer_ratio = steer_ratio
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.brake_deadband = brake_deadband
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        self.throttle_pid = PID(kp, ki, kd, self.decel_limit, self.accel_limit)
        self.lp_filter = LowPassFilter(tau, ts)
        self.wheel_radius = wheel_radius
        self.total_mass = total_mass
        self.timestamp = rospy.get_time()
        self.yaw_controller = YawController(wheel_base,
                                            self.steer_ratio,
                                            self.min_speed,
                                            self.max_lat_accel,
                                            self.max_steer_angle)
    def reset(self):
        self.throttle_pid.reset()
    def control(self, target_velocity, target_angular, current_velocity,current_timestamp):

        #
        time_diff =  current_timestamp - self.timestamp

        if time_diff > 1e-3:
            velocity_diff = target_velocity - current_velocity
            time_diff = current_timestamp - self.timestamp
            target_acceleration = velocity_diff/time_diff

            # we need to accelerate ?
            if target_acceleration > 0:

                # check if acceleration exceeds the limit
                if target_acceleration > self.accel_limit:
                    #clip the extra acceleration, if above the limit
                    target_acceleration = min(self.accel_limit, target_acceleration)
                    velocity_diff = target_acceleration*time_diff

                # calculate throttle
                throttle_pid = self.throttle_pid.step(velocity_diff, time_diff)


                #print("PID Throttle:",throttle_pid)
                # clip at 1.0
                throttle = min(1.0, throttle_pid)
                throttle = self.lp_filter.filt(throttle_pid)
                #throttle = 1.0

                breaking_force = 0.0

            elif target_acceleration <0:

                # get absolute value

                # check if deceleration exceeds the limit
                if target_acceleration< self.decel_limit:

                    # clip the extra deceleration
                    target_acceleration = max(self.decel_limit,target_acceleration)

                # calculate the breaking
                # deceleration force = total mass * deceleration
                # torque = deceleration force * wheel radius
                breaking_force = self.total_mass * abs(target_acceleration) * self.wheel_radius

                print("Breaking:",breaking_force)

                if breaking_force < self.brake_deadband:
                    breaking_force = 0.0

                throttle = 0.0

            elif velocity_diff ==0:
                throttle = 0.0
                breaking_force = 0.0

            # steering
            steering = self.yaw_controller.get_steering(target_velocity, target_angular,current_velocity)

            # set new timestamp
            self.timestamp = current_timestamp

        else:
            print("dt too small")
            throttle = 0.0
            breaking_force = 0.0
            steering = 0.0

        # Return throttle, brake, steer
        return throttle, breaking_force, steering
