from dataclasses import asdict, dataclass
from enum import Enum
from typing import List
from data_model.data import Algorithm, DemandType


@dataclass(frozen=False)
class DemandDto(object):
    demand_type: "DemandType"
    min_value: int
    max_value: int
    time_period: int
    step: int


@dataclass(frozen=False)
class GoalProgrammingParameterDto(object):
    alpha_weights: List[float]
    obj1_work_mode_lower_bound: int
    obj2_battery_lower_bound: int
    obj3_battery_upper_bound: int


@dataclass(frozen=False)
class AlgorithmDto(object):
    algorithm_type: "Algorithm"
    # MILP bounding minimum battery level
    battery_min: float
    # Time Horizon MPC parameter for MILP
    time_horizon: int
    # Number of thread
    n_thread: int = 1
    # Number of time dt for control time
    ntimes_ut_dt: int = 1


@dataclass
class BatteryConfigurationDto(object):
    config: str


@dataclass(frozen=False)
class BatteryScenarioDto(object):
    algorithm: AlgorithmDto
    battery_configuration: BatteryConfigurationDto
    # The scenario file which contains forecasted demand
    demand: DemandDto
    # Interval time during robot behaviour stay same in minutes
    dt: int
    goal_programming_parameter: GoalProgrammingParameterDto
    robot_initial_battery: List[float]
    # How much time is simulation required to run in minutes
    time: int
    scenario: str
    # Number of charge station
    charge_station: int = 1
