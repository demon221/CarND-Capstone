from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        
	#Throttle controller
	th_kp = 0.1
	th_ki = 0.0
	th_kd = 0.0

        self.th_controller = PID(th_kp, th_ki, th_kd)

	#Steering controller
	steer_kp = 0.5
	steer_ki = 0.0
	steer_kd = 0.0

        self.steer_controller = PID(steer_kp, steer_ki, steer_kd)

    def control(self, velocity_err, steering_err, sampling_time):
        #throttle = self.th_controller.step(velocity_err, sampling_time)
	steering_angle = self.steer_controller.step(steering_err, sampling_time)
	#steering_angle = 0.0
	throttle = 0.0	

	if (throttle < 0.0):
		brake = throttle
		throttle = 0.0
	else:
		brake = 0.0
        # Return throttle, brake, steer
        return throttle, brake, steering_angle
