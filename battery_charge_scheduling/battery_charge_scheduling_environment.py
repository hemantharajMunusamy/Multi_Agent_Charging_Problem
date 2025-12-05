import random
import numpy as np
from .utils import round_off_add_to_one


class BatteryChargeSchedulingEnvironment:
    """_summary_
    """
    def __init__(self, charge_model, discharge_model, go_charge_model, task_prob, reward_prob):
        self.horizon = 12

        self.init_t = None
        self.init_battery = None
        self.init_task = None
        self.init_charging = None
        
        #state
        self.t = None
        self.battery = None
        self.task_present = None
        self.charging = None

        #action
        self.actions = {'gather_reward':0, 'go_charge':1, 'stay_charging':2}
        self.action_idx = {0:'gather_reward', 1:'go_charge', 2:'stay_charging'}

        # Multi-objective weights (scalarization)
        # self.w_task = 1.0
        # self.w_battery = -.5
        self.w_task = 1.0
        self.w_battery = -1.0

        #Transistion probability
        self.charge_model = charge_model
        self.discharge_model = discharge_model
        self.go_charge_model = go_charge_model
        
        self.task_prob = task_prob
        self.reward_probability = reward_prob
        print("Task probability")
        print(self.task_prob)

    def get_parameter(self):
        return (self.horizon, 101, 2, 2)
    
    def set_reset_parameter(self, init_time, init_battery, init_task, init_charging):
        self.init_t = init_time
        self.init_battery = init_battery
        self.init_task = init_task
        self.init_charging = init_charging

    def reset(self):
        self.t = self.init_t
        self.battery = self.init_battery
        self.task_present = self.init_task
        self.charging = self.init_charging

    def get_state(self):
        return (self.t, self.battery, self.task_present, self.charging)

    def set_state(self, t, battery, task_present, charging):
        self.t = t
        self.battery = battery
        self.task_present = task_present
        self.charging = charging

    def get_random_action(self):
        if self.charging:
            return random.choice(["gather_reward", "stay_charging"])
        else:
            return random.choice(list(self.actions.keys()))
            
    def get_action_idx(self,action):
        return self.actions[action]

    def get_action_from_idx(self, idx:int):
        return self.action_idx[idx]
        
    def update_time(self, t):
        return t + 1

    def update_discharging(self, b):
        return np.random.choice(list(self.discharge_model[b].keys()), p= round_off_add_to_one(list(self.discharge_model[b].values())))

    def update_charging(self, b):
        return np.random.choice(list(self.charge_model[b].keys()), p= round_off_add_to_one(list(self.charge_model[b].values())))

    def update_go_charging(self, b):
        return np.random.choice(list(self.go_charge_model[b].keys()), p= round_off_add_to_one(list(self.go_charge_model[b].values())))

    def update_task(self, t):
        return 1 if random.random() < self.task_prob[t][0] else 0
        
    def step(self, action):
        rew = 0
        battery_cost = 0
        is_charging = 0
        is_task_present = 0
        done = False

        next_battery = None
        
        if action == 'gather_reward':
            next_battery = self.update_discharging(self.battery)
            is_charging = 0
            is_task_present = self.update_task(self.t)
            rew = 1 if is_task_present else 0
        elif action == 'go_charge':
            next_battery = self.update_go_charging(self.battery)
            is_charging = 1
        elif action == 'stay_charging':
            next_battery = self.update_charging(self.battery)
            is_charging = 1

        next_t = self.update_time(self.t)
        
        if next_battery < 40:
            battery_cost = 1
            
        scalar_reward = self.w_task * rew + self.w_battery * battery_cost

        if (next_t + 1) >= self.horizon:
            done = True
            
        return (next_t, next_battery, is_task_present, is_charging), scalar_reward, done