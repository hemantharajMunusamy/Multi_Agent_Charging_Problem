# BatteryManagementSystem
Battery Management of a Fleet of Robots for Lifelong Operation.

## Usage
### Run BMS to get result
1. run bms.py using python3 with a scanrio json file.
    ```
    python3 bms.py -s ../../test/scenario_1_rulebased.json
    ```
2. output will be available inside output directory with scenario name mentioned in the json file.
### Scenario File
- **algorithm** : 
    - algorithm_type : Type of algorithm either **rule_based** or **goal_programming**.
        - e.g. rule_based
    - battery_min: 
        - e.g. 20
    - n_thread : the total number of CPU core used to run MILP algorithm for faster execution.  
        - e.g. 16
    - time_horizon : the total number of unit in horizon length. if time_horizon is 2 means that 2 * dt minutes lookahead. 
        - e.g. 18
- **battery_configuration**:
- **charge_station** : the total number of charge stations
- **demand**
    - demand_type: "RANDOM", "TRIANGULAR_STEP"
    - min_value: 30
    - max_value: 40
    - time_period: 1
    - step: 1
- **dt**: interval time where robot behaviour stay same in minutes.
    - e.g. 3 
- **goal_programming_parameter**
    - alpha_weights : [0.0, 0.05, 0.05, 0.0, 0.9]
    - obj1_work_mode_lower_bound: 30
    - obj2_battery_lower_bound: 30
    - obj3_battery_upper_bound: 70
- **robot_initial_battery** :  the initial robots' battery percentage 
    - e.g. [38.0,37.0,32.0,30.0,35.0]
- **scenario** : "scenario_1_rulebased"
- **time** : How much time is required to run simulation in minutes
    - e.g. 120
## License

## Reference