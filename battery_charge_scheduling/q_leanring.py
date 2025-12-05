import random
import numpy as np

class QLearning:
    def __init__(self, env):
        self.env = env
        
        (num_timesteps, battery_levels, task_states, charging_states) = self.env.get_parameter()
        
        self.q = np.zeros((num_timesteps, battery_levels, task_states, charging_states, len(self.env.actions)))
    
        self.alpha = 0.05            # learning rate
        self.gamma = 0.9            # discount factor
        self.epsilon_min = 0.001
        self.epsilon_decay = 0.995
        self.epsilon = 0.1          # exploration

        self.num_episodes = 5000
        self.num_timesteps = 100

        self.max_reward = []
        
    def training(self):
        for ep in range(self.num_episodes):
            self.env.reset()
            state = self.env.get_state()  # initial state
            total_reward = 0
            self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)
            for ts in range(self.num_timesteps):
                t, battery, task, charging = self.env.get_state()
                
                action = self.env.get_action_from_idx(np.argmax(self.q[t, battery, task, charging]))
                
                # ε-greedy action selection
                if random.random() < self.epsilon:
                    action = self.env.get_random_action()

                #print(t, battery, task, charging, action)
                
                # Step environment
                next_state, reward, done = self.env.step(action)
                nt, nbattery, ntask, nch = next_state
                
                #print(next_state)
                total_reward += reward

                action_idx = self.env.get_action_idx(action)
                
                # Q-learning update
                td_error = (reward + self.gamma * np.max(self.q[nt, nbattery, ntask, nch]) -
                    self.q[t, battery, task, charging, action_idx])

                
                self.q[t, battery, task, charging, action_idx] += self.alpha * td_error
                
                if done:
                    #print(ts, total_reward ,"done")
                    break
                    
                self.env.set_state(nt, nbattery, ntask, nch)

            self.max_reward.append(total_reward)
        print("Averge reward ", np.mean(self.max_reward[-100:]))

    def get_best_action(self,s_t, s_battery, s_task, s_charging):
        print(s_t, s_battery, s_task, s_charging)
        print(self.q[s_t, s_battery, s_task, s_charging])
        return self.env.get_action_from_idx(np.argmax(self.q[s_t, s_battery, s_task, s_charging]))
    
    def policy_summary(self):
        policy = np.argmax(self.q, axis=-1)
        print("Learned policy shape:", policy.shape)