from utils import calculate_battery_model_from_transition

class RecedingHorizonControl:

    def __init__(self, init_battery, init_charging, test_days, pareto_point):
        ur = probabilistic_rewards.uncertain_rewards(test_days)
        self.task_prob, self.prob, self.clusters = ur.get_probabilistic_reward_model()
        self.charge_model, self.discharge_model, self.gocharge_model = calculate_battery_model_from_transition()
        self.cl_id = []
        self.sample_reward = []
        self.actual_reward = []
        self.exp_reward = []
        self.no_int = ur.no_int 
        self.no_days = len(ur.test_days)
        self.horizon = 48 ## in terms of intervals 
        self.no_simulations = 1
        for z in range(self.no_int*self.no_days):
            self.exp_reward.append(sum(self.prob[z%self.no_int]*self.clusters))
            print self.prob[z%self.no_int], self.clusters, sum(self.prob[z%self.no_int]*self.clusters)
        self.req_pareto_point = pareto_point
    
        sg = generate_samples.sample_generator(True, test_days) 
        self.cl_id = []
        for clid in sg.cl_ids:
            if np.isnan(clid):
                cl = None
            else:
                cl = int(clid)
            self.cl_id.append(cl)
        self.sample_reward = sg.rewards
        self.actual_reward = sg.act_rewards

        self.totalreward = np.zeros((self.no_days))
        self.init_battery = init_battery
        self.init_charging = init_charging
        self.actions = []
        self.obtained_rewards = []
        self.battery = []
        self.charging = []
        self.time =[]
        self.pareto_point = []

        self.simulate()