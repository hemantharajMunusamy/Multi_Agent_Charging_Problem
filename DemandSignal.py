import numpy as np
import math
from data_model.data import DemandType


class DemandSignal(object):
    def __init__(self, demand_type: DemandType, min: int, max: int, time_period: int, step: int):
        self.demand_type: DemandType = demand_type
        self.maximum = max
        self.minimum = min
        self.time_period = time_period
        self.step = step
        self.frequency = 1 / time_period

    def get_demand_value(self, t):
        if self.demand_type == DemandType.CONST:
            return self.get_demand_for_const(t)
        elif self.demand_type == DemandType.SIN:
            return self.get_demand_for_sin(t)
        elif self.demand_type == DemandType.RAMP:
            return self.get_demand_for_ramp(t)
        elif self.demand_type == DemandType.SQUARE:
            return self.get_demand_for_square(t)
        elif self.demand_type == DemandType.TRIANGULAR:
            return self.get_demand_for_triangular(t)
        elif self.demand_type == DemandType.TRIANGULAR_STEP:
            return self.get_demand_for_triangular_step(t)
        elif self.demand_type == DemandType.RANDOM:
            return self.get_demand_for_random(t)

    def get_demand_for_const(self, t):
        return (self.maximum + self.minimum) // 2

    def get_demand_for_sin(self, t):
        diff = self.maximum - self.minimum
        return ((diff//2) * np.sin(2*np.pi*self.frequency*t)) + (self.maximum - (diff//2))

    def get_demand_for_ramp(self, t):
        cycle_time = math.fmod(t, self.time_period)
        diff = self.maximum - self.minimum
        return (diff * (cycle_time/self.time_period))+ self.minimum

    def get_demand_for_square(self, t):
        cycle_time = math.fmod(t ,self.time_period)
        if cycle_time >= (self.time_period // 2):
            return self.minimum
        else:
            return self.maximum

    def get_demand_for_triangular(self,t):
        cycle_time = math.fmod(t, self.time_period)
        diff = self.maximum - self.minimum
        if cycle_time <= (self.time_period // 2):
            return (diff * ((2*cycle_time)/self.time_period)) + self.minimum
        else:
            cycle_time -= self.time_period//2
            return self.maximum - (diff * (((2*cycle_time)/self.time_period)))

    def get_demand_for_triangular_step(self,t:int):
        cycle_time = math.fmod(t, self.time_period)
        diff = self.maximum - self.minimum
        if cycle_time <= (self.time_period // 2):
            val = self.minimum
            for i in range(int(cycle_time)//self.step):
                if (i%2) == 0:
                    val += 1
            return min(round(val),self.maximum)
        else:
            cycle_time -= self.time_period//2
            val = self.maximum
            for i in range(int(cycle_time) // self.step):
                if (i%2) == 0:
                    val -= 1
            return max(round(val),self.minimum)

    def get_demand_for_random(self, t:int):
        np.random.seed(0)
        for _ in range(int(t)):
            np.random.randint(low=self.minimum, high=self.maximum)
        return np.random.randint(low=self.minimum, high=self.maximum)
