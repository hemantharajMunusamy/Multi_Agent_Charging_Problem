import numpy as np
import os
from utils import parse_adversary, read_prism_output, round_off_add_to_one

FILE_DIRECTORY_NAME = os.path.dirname(os.path.realpath(__file__))

class MDP:

    def __init__(self, pareto_point, charge_model, discharge_model, gocharge_model):
        self.path = FILE_DIRECTORY_NAME+'/models/'
        self.path_data = self.path + '/data/'
        self.req_pareto_point = pareto_point
        self.labels = []
        self.states = []
        self.policy = []
        self.charge_model = charge_model
        self.discharge_model = discharge_model
        self.gocharge_model = gocharge_model
    
    def initiate(self, result_file_path):
        f_no = read_prism_output(result_file_path, self.req_pareto_point)
        if f_no is not None:
            print('Reading from model_rhc'+str(f_no)+'.adv')
            self.labels, self.states, self.policy = parse_adversary(path=self.path,
                                                                    filenames=['model_rhc'+f_no+'.adv', 'model_rhc.sta', 'model_rhc.lab'])
        else:
            raise ValueError('Adversary Not Found !!!')
        
    def get_initial_state(self):
        for element in self.labels:
            if int(element[1]) == 0:
                return element[0]
            
    def get_state(self, current_state):
        d = self.states[current_state]
        return d[0], d[1], d[2], d[3], d[4], d[5], d[6]
    
    def get_possible_next_states(self,state):
        next_states = [states[0] for states in self.policy[state]]
        trans_prob = [states[1] for states in self.policy[state]]
        actions = [states[2] for states in self.policy[state]]
        return next_states, trans_prob, actions

    def get_next_state(self,current_state, reward):
            actions = []
            obtained_reward = 0
            action_set = ['gather_reward', 'go_charge', 'stay_charging']
            charging = None
            battery = None
            req_a = None
            while not any(a in action_set for a in actions): #Run until actions have any element in action_set
                nx_s, trans_prob, actions = self.get_possible_next_states(current_state)
                # print nx_s, trans_prob, actions
                if all(a == 'observe' for a in actions):
                    if  len(nx_s) == 1:  ### only 1 state possible. 
                        current_state = nx_s[0]
                    else:
                        for s in nx_s:
                            t, tp, o, e, b, ch, cl = self.get_state(s)
                            if reward != 0 and tp == '1':
                                current_state = s
                            elif reward == 0 and tp == '0':
                                current_state = s

                elif all(a == 'evaluate' for a in actions):
                    min_cl = np.inf
                    for s in nx_s:
                        t, tp, o, e, b, ch, cl = self.get_state(s)
                        if abs(self.clusters[int(cl)] - reward) < min_cl:
                            current_state = s

                else:
                    ct, ctp, co, ce, cb, cch, ccl = self.get_state(current_state)
                    charging = cch
                    battery = cb
                    if all(a == 'stay_charging' for a in actions):
                        current_state = self.act(next_states=nx_s, cb=cb, battery_dynamics=self.charge_model)
                        req_a = actions[nx_s.index(current_state)]
                        obtained_reward = 0

                    elif all(a == 'go_charge' for a in actions):
                        current_state = self.act(next_states=nx_s, cb=cb, battery_dynamics=self.gocharge_model)
                        req_a = actions[nx_s.index(current_state)]
                        obtained_reward = 0
                
                    elif all(a == 'gather_reward' for a in actions):
                        current_state = self.act(next_states=nx_s, cb=cb, battery_dynamics=self.discharge_model)
                        req_a = actions[nx_s.index(current_state)]
                        obtained_reward = reward

                    actions.append(req_a)
            
            return current_state, req_a, obtained_reward, charging, battery
    
    def act(self, next_states, cb, battery_dynamics):
        freq = []
        next_b = []
        for ns in next_states:
            t, tp, o, e, b, ch, cl = self.get_state(ns)                  
            freq.append(battery_dynamics[int(cb)][int(b)])
            next_b.append(int(b))
        prob = round_off_add_to_one(freq)
        next_state = np.random.choice(next_states, p=np.array(prob))
        return next_state
