import time

class PID:
    def __init__(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.clear()
    def clear(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.int_error = 0.0
        self.output = 0.0
        self.last_time = time.time()
    def update(self, feedback_value):
        error = self.SetPoint - feedback_value
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error
        self.PTerm = self.Kp * error #p
        self.ITerm += error * delta_time #i
        self.DTerm = 0.0
        if delta_time > 0:
            self.DTerm = delta_error / delta_time #d
        self.last_time = self.current_time
        self.last_error = error
        self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
        #print self.SetPoint,feedback_value,self.PTerm,self.Ki * self.ITerm,self.Kd * self.DTerm,"----",delta_time


