""" Implementation of Receding Horizon Control 
    Which uses task probability, battery dynamics probability then construct MDP model to maximize the cost.
"""
from io import TextIOWrapper
import subprocess
import os
import copy
from .utils import calculate_battery_model_from_transition, get_probabilistic_reward_model, round_off_add_to_one
import numpy as np
from .mdp import MDP

#PRISM_PATH = '/home/milan/prism/prism/bin'
PRISM_PATH = '/home/hemantharaj/Downloads/prism-4.9-linux64-x86/bin'
FILE_DIRECTORY_NAME = os.path.dirname(os.path.realpath(__file__))

class RecedingHorizonControl:
    """_summary_"""

    def __init__(self, pareto_point, horizon):
        self.task_prob, self.prob, self.clusters = get_probabilistic_reward_model()
        self.charge_model, self.discharge_model = calculate_battery_model_from_transition()
        self.gocharge_model = self.get_gocharge_model(self.charge_model)

        self.sample_reward = []
        self.actual_reward = None
        self.exp_reward = []
        self.horizon = horizon ## in terms of intervals
        self.no_simulations = 1

        self.req_pareto_point = pareto_point

        self.main_path = FILE_DIRECTORY_NAME
        self.path_mod = self.main_path + '/models/'
        self.path_data = self.main_path + '/data/'

        self.actions = ['gather_reward', 'go_charge', 'stay_charging']

        self.model = MDP(self.req_pareto_point, self.charge_model, self.discharge_model, self.gocharge_model, self.clusters)

        self.init_battery = None
        self.init_charging = None
        self.init_clid = None
        self.obtained_rewards = []
        self.battery = []
        self.charging = []
        self.time =[]
        self.pareto_point = []

    def initiate(self, init_battery:int, init_charging:int, init_clid:int, actual_reward):
        """Initiate the Receding Horizon Control with initial robot battery, charging state and task present

        Args:
            init_battery (_type_): _description_
            init_charging (_type_): _description_
            init_clid (_type_): _description_
        """
        self.init_battery = int(init_battery)
        self.init_charging = int(init_charging)
        self.init_clid = int(init_clid)
        self.actual_reward = int(actual_reward)


    def get_plan(self, fname):
        if 'pre1' == self.req_pareto_point or 'pre2' == self.req_pareto_point:
            plan_path = self.path_data + self.req_pareto_point+ fname
        else:
            plan_path = self.path_data + 'p'+ str(self.req_pareto_point)+ fname
        print('Writing plan to ', plan_path, ' ...')
        with open(plan_path, 'w') as f:
            f.write('time battery charging action obtained_reward match_reward actual_reward exp_reward pareto\n')
            for t, b, ch, a, obr, mr, ar, er, pp in zip(self.time, self.battery, self.charging, self.actions, self.obtained_rewards, self.sample_reward, self.actual_reward, self.exp_reward, self.pareto_point):
                f.write('{0} {1} {2} {3} {4} {5} {6} {7} {8}\n'.format(t, b, ch, a, obr, mr, ar, er, pp))


    def get_gocharge_model(self, charge_model:dict):
        """_summary_

        Args:
            charge_model (dict): _description_

        Returns:
            _type_: _description_
        """
        gocharge_model = dict ()
        for b, bdict in charge_model.items():
            g_bdict = dict()
            if b == 99 or b == 100:
                g_bdict = copy.deepcopy(bdict)
            else:
                for bn, count in bdict.items():
                    gbn = round(0.99*bn)
                    if gbn > b:
                        if gbn in g_bdict:
                            g_bdict[gbn] += count
                        else:
                            g_bdict.update({gbn : count})
                    else:
                        if bn in g_bdict:
                            g_bdict[bn] += count
                        else:
                            g_bdict.update({bn : count})
            gocharge_model.update({b:g_bdict})
        return gocharge_model
    
    def get_horizon_prob(self, t):
        prob_c = {}
        prob_t = np.zeros((self.horizon, 2))
        for k in range(self.horizon):  
            prob_c[k] = self.prob[(t+k) % len(self.prob)]
            prob_t[k] = self.task_prob[(t+k) % len(self.task_prob)]
        return prob_c, prob_t

    def obtain_prism_model(self,t):
        prob_c, prob_t = self.get_horizon_prob(t)
        self.write_prism_file('model_rhc.prism', prob_t, prob_c, self.init_battery, self.init_charging, self.init_clid)

        result_file_path = self.path_data + 'result_rhc'
        #######################SPECIFY LOCATION ######################
        # running prism and saving output from prism
        with open(result_file_path, 'w') as f:
            process = subprocess.call(
                './prism '+ self.path_mod + 'model_rhc.prism '+ 
                self.path_mod +'batterycost_model_prop.props -paretoepsilon 0.1 -v -exportadv '+ 
                self.path_mod+ 'model_rhc.adv -exportprodstates ' + self.path_mod +'model_rhc.sta -exporttarget '+
                self.path_mod+'model_rhc.lab',cwd=PRISM_PATH, shell=True, stdout=f
            )

        self.model.initiate(result_file_path)

    def get_next_state(self, t):
        current_state = self.model.get_initial_state()
        print("Current state ", current_state)
        print("Current State ", self.get_state(current_state))
        next_state, req_a, obtained_reward, charging, battery = self.model.get_next_state(current_state=current_state, 
                                                                                          reward=self.actual_reward)
        print("Next state ", next_state, req_a)
        print("Next state ", self.get_state(next_state))
        return next_state
    
    def get_state(self, s):
        return self.model.get_state(s)
        

    def time_model(self, task_prob:list, init_clid, f: TextIOWrapper):
        """_summary_

        Args:
            f (TextIOWrapper): _description_
        """
        f.write('module time_model\n')
        f.write(f't:[0..{self.horizon}] init 0;\n')
        f.write(f'task_present:[0..1] init {0};\n')
        f.write(f'o:[0..1] init {0};\n')
        f.write(f'e:[0..1] init {0};\n')
        for i in range(self.horizon):
            f.write(f"[observe] (t={i}) & (o=0) & (e=0) -> {task_prob[i][0]}:(task_present'=1) & (o'=1) + {task_prob[i][1]}:(task_present'=0) & (o'=1);\n")
        f.write(f"[evaluate] (t<{self.horizon}) & (o=1) & (task_present=1) & (e=0) -> (e'=1);\n")
        for action in self.actions:
            f.write(f"[{action}] (t<{self.horizon}) & (o=1) & (task_present=1) & (e=1) -> (t'=t+1) & (o'=0) & (e'=0) ;\n")
        for action in self.actions[1:]:
            f.write(f"[{action}] (t<{self.horizon}) & (o=1) & (task_present=0) -> (t'=t+1) & (o'=0);\n")
        f.write(f"[dead] (t<{self.horizon}) -> (t'=t+1) & (o'=0);\n")
        f.write('endmodule\n\n')

    def battery_dynamics(self, init_b:int, f: TextIOWrapper, min_b = 25):
        """
            Probability battery dynamics

                module battery_model
                    battery:[0..100] init {init_b};

                    [gather_reward] (battery={b}) & (t>-1) -> "{prob_bnext}:(battery'={bnext})"
                    [go_charge] (battery={0}) & (t>-1) -> "{prob_bnext}:(battery'={bnext})"
                    [stay_charging] (battery={b}) & (t>-1) -> "{prob_bnext}:(battery'={bnext})"

                    [evaluate] (battery>0) -> (battery'=battery);
                    [observe] (battery>0) -> (battery'=battery);
                    [dead] (battery=0) & (t>-1) -> (battery' = battery);
                endmodule
        """
        f.write('module battery_model\n')
        f.write(f'battery:[0..100] init {init_b};\n')
        for b in self.discharge_model:
            if b > min_b:
                bnext_dict:dict = self.discharge_model[b]
                bnext_prob = round_off_add_to_one(list(bnext_dict.values()))
                f.write(f"[gather_reward] (battery={b}) & (t>-1) -> ")
                b_next = "+".join(
                    [f"{round(prob,2)}:(battery'={int(bnext)})" for bnext, prob in zip(bnext_dict.keys(), bnext_prob)]
                )
                f.write(b_next)
                f.write(';\n')

        for b,bnext_dict in self.gocharge_model.items():
            bnext_prob = round_off_add_to_one(list(bnext_dict.values()))
            f.write(f"[go_charge] (battery={b}) & (t>-1) -> ")
            b_next = "+".join(
                    [f"{prob}:(battery'={int(bnext)})" for bnext, prob in zip(bnext_dict.keys(), bnext_prob)]
            )
            f.write(b_next)
            f.write(';\n')

        for b in self.charge_model:
            bnext_dict = self.charge_model[b]
            bnext_prob = round_off_add_to_one(list(bnext_dict.values()))
            f.write(f"[stay_charging] (battery={b}) & (t>-1) -> ")
            b_next = "+".join(
                    [f"{prob}:(battery'={int(bnext)})" for bnext, prob in zip(bnext_dict.keys(), bnext_prob)]
            )
            f.write(b_next)
            f.write(';\n')

        f.write("[evaluate] (battery>0) -> (battery'=battery);\n")
        f.write("[observe] (battery>0) -> (battery'=battery);\n")
        f.write("[dead] (battery=0) & (t>-1) -> (battery' = battery);\n")
        f.write('endmodule\n\n')

    def model_charging_state(self, init_ch, f: TextIOWrapper):
        """_summary_

        Args:
            init_ch (_type_): _description_
            f (TextIOWrapper): _description_
        """
        f.write('module charging_state\n')
        f.write(f'charging:[0..1] init {init_ch};\n')
        f.write("[gather_reward] (charging=0) | (charging=1) -> (charging'=0);\n")
        f.write("[stay_charging] (charging=1) -> (charging'=1);\n")
        f.write("[go_charge] (charging=0) -> (charging'=1);\n")
        f.write("[dead] (charging=0) -> (charging'=0);\n")
        f.write('endmodule\n\n')

    def model_cluseter_evolution(self, init_clid, prob_cl:list, f: TextIOWrapper):
        """_summary_

        Args:
            f (TextIOWrapper): _description_
        """
        f.write('module cluster_evolution\n\n')
        f.write(f'cl:[0..{len(self.clusters)}] init {init_clid if init_clid is not None else 0};\n')
        for t in range(self.horizon):
            if not all([p == 0 for p in prob_cl[t]]):
                f.write(f'[evaluate] (task_present=1) & (t={t}) -> ')
                cluster_probability = "+".join(
                    [f"{prob_cl[t][cl]}:(cl'={cl})" for cl in range(len(self.clusters))]
                )
                f.write(cluster_probability)
                f.write(';\n')
        f.write('endmodule\n\n')


    def cluster_reward(self, f: TextIOWrapper):
        """ Construct Task Cluster Reward

            rewards "rew"
                gather_reward] (cl={task_cluster}):{task_cluster_value};
            endrewards
        """
        f.write('rewards "rew"\n')
        for e,cl in enumerate(self.clusters):
            f.write(f'[gather_reward] (cl={e}):{cl};\n')
        f.write('\nendrewards\n\n')

    def battery_cost_reward(self,f: TextIOWrapper, threshold = 40, cost = 1):
        """Construct Battery Cost
            Whenever the battery less than 40 add a unit cost as the reward.
            
            rewards "batterycost"
                [gather_reward] (battery<40):1;
                [stay_charging] (battery<40):1;
                [go_charge] (battery<40):1;
            endrewards
        """
        f.write('rewards "batterycost"\n')
        for action in self.actions:
            f.write(f'[{action}] (battery < {threshold}):{cost};\n')
        f.write('\nendrewards\n\n')

    def write_prism_file(self, filename, task_prob:list, prob_cl:list, init_b:int, init_ch:int, init_clid:int):
        """_summary_

        Args:
            filename (_type_): _description_
            prob_c (list): List of cluster probability
            init_b (int): initial battery percent of the robot
            init_ch (int): initial chargning state of the robot
            init_clid (int): initial task performing which is having the cluster id clid
        """
        path = self.main_path +'/models/' + filename 
        with open(path, 'w', encoding='utf-8') as f:
            f.write('mdp\n\n')
            self.time_model(task_prob=task_prob, init_clid=init_clid, f=f)
            self.battery_dynamics(init_b=init_b, f=f)           
            self.model_charging_state(init_ch=init_ch, f=f)
            self.model_cluseter_evolution(init_clid=init_clid, prob_cl=prob_cl, f=f)
            self.cluster_reward(f)
            self.battery_cost_reward(f)
