
class PID:
    def __init__(self):
        print("Creating PID obj")
        self.Kp = 0.1
        self.Ki = 0.005 * 3
        self.Kd = 0.05 * 0.33

        self.tau = 0.15

        self.limMin = -1
        self.limMax = 1

        self.T = 0.033

        self.proportional = 0
        self.integrator = 0
        self.differentiator = 0

        self.prev_measurement = 0
        self.prev_error = 0


        self.out = 0
        self.prev_outs =[0]*5
        pass

    def step(self, setpoint, measurment) -> float:
        error = setpoint - measurment

        self.proportional = self.Kp * error

        if self.Ki == 0:
            self.integrator = 0
        else:
            self.integrator = self.integrator + 0.5 * self.Ki * self.T * (error + self.prev_error)

        #self.differentiator = -(2.0 * self.Kd * (measurment - self.prev_measurement))
        if self.Kd == 0:
            self.differentiator = 0
        else:
            #self.differentiator = (2 * self.Kd/(2*self.tau+self.T)*(error - self.prev_error)+(2*self.tau-self.T)/(2*self.tau+self.T)*self.differentiator)
            self.differentiator = -(2*self.Kd * (measurment - self.prev_measurement))+(2*self.tau-self.T)/(2*self.tau+self.T)*self.differentiator
            pass

        self.out = self.proportional + self.integrator + self.differentiator

        '''print("set: {s:.3f} | mes: {m:.3f}".format(s=setpoint, m=measurment))
        print("Error: {e:.3f}".format(e=error))
        print("P:", self.proportional)
        print("I:",self.integrator)
        print("D:",self.differentiator)'''

        if self.out > self.limMax:
            self.out = self.limMax
        elif self.out < self.limMin:
            self.out = self.limMin

        self.integrator = self.out - (self.proportional + self.differentiator)

        self.prev_measurement = measurment
        self.prev_error = error

        self.prev_outs[1:len(self.prev_outs)] = self.prev_outs[0:len(self.prev_outs)-1]
        self.prev_outs[0] = self.out

        self.out = sum(self.prev_outs)/len(self.prev_outs)

        return self.out

    pass
