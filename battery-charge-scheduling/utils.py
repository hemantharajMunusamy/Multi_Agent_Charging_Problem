""" Utility function to support the battery charge scheduler"""
import yaml
import pandas as pd
import numpy as np
import ast

CHARGING_FNAME = "./models/battery_charge_model.yaml"
DISCHARGING_FNAME = "./models/battery_discharge_model.yaml"
TASK_FNAME = "./models/task_prob.csv"
REWARD_FNAME = "./models/reward_prob.csv"

def read_battery_model(charging_fname:str, discharging_fname:str):
    """ Read the battery model from the given file name"""
    try:
        with open (charging_fname, encoding='utf-8') as f_charge:
            charge_model = yaml.load(f_charge, Loader=yaml.SafeLoader)

        with open (discharging_fname, encoding='utf-8') as f_discharge:
            discharge_model = yaml.load(f_discharge, Loader=yaml.SafeLoader)

        return charge_model, discharge_model
    
    except FileNotFoundError:
        raise ValueError("Files aren't found.")
    
def read_task_probability(task_fname:str) -> pd.DataFrame:
    """
    Read the task probability from the given task_fname
    Args:
        task_fname (str): _description_

    Raises:
        ValueError: _description_

    Returns:
        pd.DataFrame: Output Dataframe have the following field
                        - day_interval as a index column
                        - task_present_prob as a probability of task present in that time interval
                        - task_not_present_prob as a probability of task not present in that time interval
    """
    try:
        return ( pd.read_csv(task_fname)
                    .set_index("day_interval")
                    .to_numpy()
        )
    except FileNotFoundError:
        raise ValueError
    
def read_and_process_reward_probability(reward_fname:str) -> pd.DataFrame:
    """
    Read the task probability from the given task_fname
    Args:
        task_fname (str): _description_

    Raises:
        ValueError: _description_

    Returns:
        pd.DataFrame: Output Dataframe have the following field
                        - day_interval as a index column
                        - task_present_prob as a probability of task present in that time interval
                        - task_not_present_prob as a probability of task not present in that time interval
    """
    try:
        
        reward_df = pd.read_csv(reward_fname, sep=' ').set_index("day_interval")
        
        #Find all the unique clusters value
        clusters = [0,1]

        reward_prob =  {}
        reward_prob[1] = reward_df["rew_cluster_1_and_prob"].map(ast.literal_eval)
        reward_prob[0] = reward_df['rew_cluster_2_and_prob'].map(ast.literal_eval)

        rew_prob:dict = {} #{interval: {cluster1:prob1, cluster2:prob2}}

        for index, value in zip(reward_prob[1].index.to_list(), reward_prob[1].to_list()):
            if index not in rew_prob:
                rew_prob[index] = {}
            rew_prob[index].update({value[0]:value[1]})

        for index, value in zip(reward_prob[0].index.to_list(), reward_prob[0].to_list()):
            if index not in rew_prob:
                rew_prob[index] = {}
            rew_prob[index].update({value[0]:value[1]})

        return rew_prob, clusters
    except FileNotFoundError:
        raise ValueError


def calculate_battery_model_from_transition():
    """
        Given: b_curr: {b_next1 : b_next1_count, b_next2 : b_next2_count, ..., b_nextn : b_next2_count}
        Using battery transition statistics to find a transition probabilities
    """
    return read_battery_model(CHARGING_FNAME, DISCHARGING_FNAME)


def get_probabilistic_reward_model():
    """ Get the probabilistic reward model from a stored file"""
    task_prob =  read_task_probability(TASK_FNAME)
    reward_prob , clusters = read_and_process_reward_probability(REWARD_FNAME)
    return task_prob, reward_prob, clusters


def parse_adversary(path:str, filenames:list):
    labels:list = []
    states:dict = dict()
    policy:dict = dict()
    for name in filenames:
        if name[-4:] == '.lab':
            with open(path+name, 'r', encoding='utf-8') as label_file:
                labels =[]
                for line in label_file.readlines():
                    if 'init' not in line:
                        labels.append(line[:-1].split(': '))
        elif name[-4:] == '.sta':
            with open(path+name, 'r', encoding='utf-8') as state_file:
                states = dict()
                for line in state_file.readlines():
                    if 't' not in line and 'battery' not in line:
                        state = line[:-2].split(':(')
                        s = state[1].split(',')
                        states.update({state[0] : [el.strip() for el in s]})
        elif name[-4:] == '.adv':
            with open(path+name, 'r', encoding='utf-8') as adv_file:
                policy = dict()
                for line in adv_file:
                    array = line[:-1].split(' ')
                    if len(array) > 2:
                        if array[0] not in policy:
                            policy.update({array[0] : [array[1:]]})
                        else:
                            policy[array[0]].append(array[1:])
        #self.initial_state = self.get_initial_state()
    return labels, states, policy

def read_prism_output(result_file_path:str, req_pareto_point):
    """reading output from prism to find policy file for bcth

    Returns:
        _type_: _description_
    """
    pre1_point = None
    pre2_point = None
    pareto_point = []
    with open(result_file_path, 'r', encoding='utf-8') as f:
        line_list = f.readlines()
        f_no_list = []
        pareto_points = []
        init = 0
        for e, line in enumerate(line_list):
            if 'pre1.adv' in line:
                pre1_point = abs(float(line_list[e+1].split(',')[0].split('(')[1].strip()))

            if 'pre2.adv' in line:
                pre2_point = abs(float(line_list[e+1].split(',')[0].split('(')[1].strip())) 

            if ': New point is (' in line:
                el = line.split(' ')
                print(el)
                if init == 0:
                    init_no = int(el[0][:-1])
                cost40 = abs(float(el[4][1:-1]))
                pareto_points.append(cost40)
                #f_no_list.append(str(int(el[0][:-1])-2))
                f_no_list.append(str(int(el[0][:-1])-1))
                init +=1
    if 'pre1' == req_pareto_point or 'pre2' == req_pareto_point:
        f_no = req_pareto_point
        if req_pareto_point == 'pre1':
            pareto_point.append(pre1_point)
        elif req_pareto_point == 'pre2':
            pareto_point.append(pre2_point)
    else:
        if f_no_list:
            if req_pareto_point > 3 and req_pareto_point < 6:
                approx_p_point = min(pareto_points) + ((max(pareto_points)-min(pareto_points))/3)*(float((req_pareto_point%3))/3)
            elif req_pareto_point == 6:
                sorted_pareto_points = sorted(pareto_points)
                if len(sorted_pareto_points) > 1:
                    approx_p_point = sorted_pareto_points[1]
                else:
                    approx_p_point =  sorted_pareto_points[0]
            else:
                approx_p_point = min(pareto_points) + ((max(pareto_points)-min(pareto_points)))*(float(req_pareto_point)/3) ## 3 -> no. of pareto points being considered
            p_point = min(pareto_points, key=lambda x: abs(x-approx_p_point))
            pareto_point.append(p_point)
            f_ind = pareto_points.index(p_point)
            f_no = f_no_list[f_ind]
        else:
            f_no = None
        
    print(pareto_points)
    print(pareto_point)
    print(f_no)
    return f_no


def round_off_add_to_one(data:list, decimal_place=2):
        """ Step 1: Round each probability to {decimal_place} decimals
            Step 2: Compute the difference from 1.
        Args:
            data (list): _description_
        """
        normalize_data = np.array(data)/np.sum(data)
        round_data = np.round(normalize_data, decimal_place)
        diff = round((1 - np.sum(round_data)) * 10**(decimal_place))
        
        sorted_rounded = np.argsort(round_data)
        small_diff = np.round(np.power(0.1, decimal_place), decimal_place)
        for i in range(abs(diff)):
            idx =  sorted_rounded[i % len(round_data)]
            if diff > 0:
                round_data[idx] += small_diff
            else:
                round_data[idx] -= small_diff
        return round_data
