"""Test the implementation of the battery charge scheduling"""

from utils import calculate_battery_model_from_transition, get_probabilistic_reward_model
from receding_horizon_control import RecedingHorizonControl

def main():
    """
    Main codebase
    """
    a = RecedingHorizonControl(3, 10)
    a.initiate(60, 0, 1, [1])
    print(a.actual_reward)
    a.obtain_prism_model(0)
    next_state = a.get_next_state(0)
    print(a.get_state(next_state))

if __name__ == "__main__":
    main()
