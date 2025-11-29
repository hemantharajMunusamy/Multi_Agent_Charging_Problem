from dataclasses import astuple
from re import I, L
import matplotlib.pyplot as plt
import enum
import csv
import json
import threading
import time
import argparse
from data_model.data import Charge_Station_Mode, Algorithm, Event, Mode_Of_Operation, DemandType, Data
from black_board import *
from DemandSignal import DemandSignal, DemandType
from data_model.BatteryScenarioDto import DemandDto, AlgorithmDto, GoalProgrammingParameterDto, BatteryConfigurationDto, BatteryScenarioDto
from rule_based_approach.rule_based_approach import *
from threshold_rule_based.threshold_rule_based import ThresholdRuleBasedApproach
from goal_programming.goal_programming import *
from battery_charge_scheduling.battery_charge_scheduling import BatteryChargeScheduling
import os

sim_to_real = 0.5
TIMEINTERVAL = 0.1
OUTPUTPATH = ""
# TIMEHORIZON = 1
DISCHARGERATE = 100.0 / (2 * 60.0)
CHARGERATE = 100.0 / 20.0
QUEUERATE = 100.0 / (8 * 60.0)
# BMIN = 20


class Charge_Station:
    def __init__(self, g_id, m_id, global_blackboard):
        # Global ID
        self.global_id = g_id
        self.module_id = m_id
        # Centralized Data Store
        self.black_board = global_blackboard
        self.node_id = self.black_board.register_node(
            self.global_key(), f"Node:Charge_Station {self.global_id}"
        )

        self.charge_id = m_id
        self.mode = Charge_Station_Mode.Free
        self.rid = -1

        self.setup()

    def global_key(self):
        return (self.global_id * 100) + self.module_id

    def setup(self):
        self.black_board.register(self.global_key(), 1, Data.GID)
        self.black_board.register(self.global_key(), 1, Data.Charge_Station_Mode)

        self.black_board.write((self.global_key(), Data.GID), self.global_key())
        self.black_board.write((self.global_key(), Data.Charge_Station_Mode), self.mode)
        self.black_board.write((self.global_key(), Data.Charge_Station_Robot), self.rid)


class Battery:
    def __init__(self, g_id, m_id, global_blackboard):
        # Global ID
        self.global_id = g_id
        self.module_id = m_id
        # Centralized Data Store
        self.black_board = global_blackboard
        self.node_id = self.black_board.register_node(
            self.global_key(), f"Node:Battery {self.global_id}"
        )

        self.rid = m_id
        self.charge_robot_info = [-1, -1, False]
        self.charge_time = 0
        self.critical = False
        self.battery_filepath = OUTPUTPATH
        open(file=self.battery_filepath, mode="w+", encoding='utf-8').close()
        self.battery_file = open(file=self.battery_filepath, mode="a", encoding='utf-8')
        self.battery_writer = csv.writer(self.battery_file)
        if self.module_id == 0:
            self.battery_writer.writerow(
                ["time", "sim2real", "rid", "cid", "gid", "mode", "battery", "critical"]
            )
        self.battery_percentage = None
        self.mode = Mode_Of_Operation.Off_Mode
        self.battery_rate = 0.0
        self.start_thread = False
        self.thread = None
        self.dt = TIMEINTERVAL

        self.time = 0.0

        self.setup()

    def global_key(self):
        return (self.global_id * 100) + self.module_id

    def setup(self):
        self.black_board.register(self.global_key(), 1, Data.GID)
        self.black_board.register(self.global_key(), 1, Data.Battery)
        self.black_board.register(self.global_key(), 1, Data.Battery_Mode)
        self.black_board.register(self.global_key(), 1, Data.Charge_Robot_Info)

        self.black_board.write((self.global_key(), Data.GID), self.global_key())
        self.black_board.write((self.global_key(), Data.Battery_Mode), self.mode)
        self.black_board.write((self.global_key(), Data.Charge_Time), 0)
        self.black_board.write(
            (self.global_key(), Data.Charge_Robot_Info), self.charge_robot_info
        )

    def set_battery(self, battery):
        self.battery_percentage = battery

    def get_battery(self, battery):
        return self.battery_percentage

    def set_mode(self, mode):
        self.mode = mode

    def initiate_mode(self):
        self.battery_percentage = self.black_board.read(
            (self.global_key(), Data.Battery)
        )
        self.mode = self.black_board.read((self.global_key(), Data.Battery_Mode))

    def update_charge_mode(self):
        self.charge_robot_info = self.black_board.read(
            (self.global_key(), Data.Charge_Robot_Info)
        )
        if self.charge_robot_info[0] != -1:
            if self.charge_robot_info[0] != 0:
                self.charge_time = self.black_board.read(
                    (self.global_key(), Data.Charge_Time)
                )
                charge_time = self.charge_time - self.dt
                if charge_time < 0:
                    self.mode = Mode_Of_Operation.Work_Mode
                    self.black_board.write(
                        (self.charge_robot_info[0], Data.Charge_Station_Mode),
                        Charge_Station_Mode.Free,
                    )
                    self.charge_station_id = -1
                    self.black_board.write(
                        (self.global_key(), Data.Charge_Robot_Info), [-1, -1, False]
                    )
                elif charge_time >= 0:
                    self.mode = Mode_Of_Operation.Charge_Mode

    def update_charge_time(self):
        self.charge_robot_info = self.black_board.read(
            (self.global_key(), Data.Charge_Robot_Info)
        )

        if self.charge_robot_info[0] != -1:

            if self.charge_robot_info[0] != 0:
                self.charge_time = self.black_board.read(
                    (self.global_key(), Data.Charge_Time)
                )
                self.charge_time = self.charge_time - self.dt

                # if (self.charge_time <= 0) and (self.mode == Mode_Of_Operation.Charge_Mode):
                if self.charge_time < 0:
                    self.mode = Mode_Of_Operation.Work_Mode
                    self.black_board.write(
                        (self.charge_robot_info[0], Data.Charge_Station_Mode),
                        Charge_Station_Mode.Free,
                    )
                    self.charge_station_id = -1
                    self.black_board.write(
                        (self.global_key(), Data.Charge_Robot_Info), [-1, -1, False]
                    )
                elif self.charge_time >= 0:
                    self.mode = Mode_Of_Operation.Charge_Mode
            else:
                if self.mode != Mode_Of_Operation.Off_Mode:
                    self.mode = Mode_Of_Operation.Queue_Mode
        else:
            # forcefully removed robot from charge station
            if self.mode == Mode_Of_Operation.Charge_Mode:
                self.mode = Mode_Of_Operation.Work_Mode
            elif self.mode == Mode_Of_Operation.Queue_Mode:
                self.mode = Mode_Of_Operation.Work_Mode
        # else:
        #    print(f"{self.global_key()} Unknown State in Update charge time")

        if self.charge_robot_info[0] != -1:
            print(
                f"Robot {self.rid} Charge Station ID {self.charge_robot_info[0]} Queue ID: {self.charge_robot_info[1]}"
            )
        self.black_board.write((self.global_key(), Data.Charge_Time), self.charge_time)
        self.black_board.write((self.global_key(), Data.Battery_Mode), self.mode)

    def set_rate(self):
        if self.mode == Mode_Of_Operation.Charge_Mode:
            # self.battery_rate = 6.
            # self.battery_rate = 3.33
            self.battery_rate = CHARGERATE * self.dt
        elif self.mode == Mode_Of_Operation.Work_Mode:
            # self.battery_rate = -4.
            # self.battery_rate = -0.33
            self.battery_rate = -DISCHARGERATE * self.dt
        elif self.mode == Mode_Of_Operation.Queue_Mode:
            # self.battery_rate = -0.033
            self.battery_rate = -QUEUERATE * self.dt
        elif self.mode == Mode_Of_Operation.Off_Mode:
            self.battery_rate = 0.0
        elif self.mode == Mode_Of_Operation.On_Mode:
            # self.battery_rate = -0.01
            self.battery_rate = -(100.0 / (10 * 60.0)) * self.dt

    def update_battery(self):
        # print(f"Battery Percentage {self.battery_percentage} ,Rate {self.battery_rate}")
        self.battery_percentage += self.battery_rate
        self.battery_percentage = min(self.battery_percentage, 100.0)
        self.battery_percentage = max(self.battery_percentage, 0.0)

        if self.battery_percentage < 30.0:
            self.critical = True

        # if (self.critical) and (self.mode == Mode_Of_Operation.Work_Mode):
        #    self.critical = False

        # kself.black_board.write((self.global_key(), Data.Critical), self.critical)
        self.black_board.write(
            (self.global_key(), Data.Battery), self.battery_percentage
        )

    def store_data(self):
        self.battery_file = open(file=self.battery_filepath, mode="a", encoding='utf-8')
        self.battery_writer = csv.writer(self.battery_file)
        self.battery_writer.writerow(
            [
                self.time,
                sim_to_real,
                self.rid,
                self.charge_robot_info[0],
                self.charge_robot_info[1],
                self.mode.value,
                self.battery_percentage,
                self.charge_robot_info[2],
            ]
        )
        self.battery_file.close()

    # def start_behaviour(self):
    #     self.start_thread = True
    #     self.thread = threading.Thread(target=self.behaviour, args=[])
    #     self.thread.start()
    #     self.thread.join()

    # def parallel_behaviour(self):
    #     while self.start_thread:
    #         self.set_rate()
    #         self.battery_percentage += self.battery_rate
    #         self.battery_percentage = min(self.battery_percentage, 100.)
    #         self.battery_percentage = max(self.battery_percentage, 0.)
    #         #print(f"Robot {self.rid} Percentage {self.battery_percentage}")
    #         self.battery_file = open(self.battery_filepath, "a")
    #         self.battery_writer = csv.writer(self.battery_file)
    #         self.battery_writer.writerow([self.time,sim_to_real,self.rid, self.mode.value, self.battery_percentage])
    #         self.battery_file.close()
    #         time.sleep(self.dt *sim_to_real)

    def behaviour(self):
        self.initiate_mode()
        self.time += self.dt
        if self.mode is not None:
            self.update_charge_time()
            self.set_rate()
            self.update_battery()
            self.store_data()
            self.update_charge_mode()


class Robot:
    def __init__(self, g_id, m_id, global_blackboard, battery):
        # Global ID
        self.global_id = g_id
        self.module_id = m_id
        # Centralized Data store
        self.black_board = global_blackboard
        self.node_id = self.black_board.register_node(
            self.global_key(), f"Node:Rule_Based_Approach {self.global_id}"
        )

        self.battery = Battery(
            self.global_id, self.module_id, self.black_board )

        self.initial_battery = battery
        self.initialized = False

        self.initiate()

    def initiate(self):
        self.black_board.write((self.global_key(), Data.RID), self.module_id)
        self.black_board.write((self.global_key(), Data.Battery), self.initial_battery)
        self.black_board.write(
            (self.global_key(), Data.Battery_Mode), Mode_Of_Operation.Work_Mode
        )
        # self.black_board.write((self.global_key(),Data.Goal), Point(self.goal.x, self.goal.y))
        self.initialized = True

    def global_key(self):
        return (self.global_id * 100) + self.module_id

    def set_battery_percentage(self, battery_):
        self.initial_battery = battery_

    def start_behaviour(self):
        self.battery.start_behaviour()

    def behaviour(self):
        if not self.initialized:
            self.initiate()
            return
        self.battery.behaviour()


class Fleet:
    def __init__(self, g_id, m_id, global_blackboard):
        # Global ID
        self.global_id = g_id
        self.module_id = m_id
        # Centralized Data Store
        self.black_board = global_blackboard
        self.node_id = self.black_board.register_node(
            self.global_key(), f"Node:Battery {self.global_id}"
        )


        self.robot = {}
        self.robot_count = 0

        self.charge_station = {}
        self.charge_station_count = 0

        self.battery_manager = None
        self.count = 0

    def global_key(self):
        return (self.global_id * 100) + self.module_id

    def add_robot(self, battery):
        self.robot[self.robot_count] = Robot(
            2, self.robot_count, self.black_board, battery)
        self.robot_count += 1

    def add_charge_station(self):
        self.charge_station[self.charge_station_count] = Charge_Station(
            3, self.charge_station_count, self.black_board)
        self.charge_station_count += 1

    def set_battery_manager(
        self,
        algorithm: AlgorithmDto,
        demand: DemandSignal,
        goal_programming_parameter: GoalProgrammingParameterDto,
    ):
        if algorithm.algorithm_type == Algorithm.RULE_BASED.value:
            self.battery_manager = Rule_Based_Approach(
                3, 0, self.black_board)
            self.battery_manager.set_battery_parameter(
                CHARGERATE, DISCHARGERATE, QUEUERATE, TIMEINTERVAL
            )
            self.battery_manager.set_battery_min(algorithm.battery_min)
            self.battery_manager.set_dt(TIMEINTERVAL)
        elif algorithm.algorithm_type == Algorithm.THRESHOLD_RULE_BASED.value:
            self.battery_manager = ThresholdRuleBasedApproach(
                3, 0, self.black_board
            )
            self.battery_manager.set_battery_parameter(
                CHARGERATE, DISCHARGERATE, QUEUERATE, TIMEINTERVAL
            )
            self.battery_manager.set_battery_min(algorithm.battery_min)
            self.battery_manager.set_dt(TIMEINTERVAL)
        elif algorithm.algorithm_type == Algorithm.MILP.value:
            self.battery_manager = Goal_Programming(
                3, 0, self.black_board)
            self.battery_manager.set_algorithm_type(algorithm.algorithm_type)
            self.battery_manager.set_n_thread(algorithm.n_thread)
            self.battery_manager.setBMIN(algorithm.battery_min)
            self.battery_manager.setTimeHorizon(algorithm.time_horizon)
            self.battery_manager.setBatteryParameter(
                CHARGERATE, DISCHARGERATE, QUEUERATE, TIMEINTERVAL
            )
        elif algorithm.algorithm_type == Algorithm.GOAL_PROGRAMMING.value:
            self.battery_manager = Goal_Programming(
                3, 0, self.black_board)
            self.battery_manager.set_algorithm_type(algorithm.algorithm_type)
            self.battery_manager.set_goal_programming_parameter(
                goal_programming_parameter
            )
            self.battery_manager.set_n_thread(algorithm.n_thread)
            self.battery_manager.setBMIN(algorithm.battery_min)
            self.battery_manager.setTimeHorizon(algorithm.time_horizon)
            self.battery_manager.setBatteryParameter(
                CHARGERATE, DISCHARGERATE, QUEUERATE, TIMEINTERVAL
            )
            self.battery_manager.setDemandSignal(demand)
        elif (
            algorithm.algorithm_type
            == Algorithm.GOAL_PROGRAMMING_SAMPLED_DATA_CONTROL.value
        ):
            self.battery_manager = Goal_Programming(
                3, 0, self.black_board
            )
            self.battery_manager.set_algorithm_type(algorithm.algorithm_type)
            self.battery_manager.set_goal_programming_parameter(
                goal_programming_parameter
            )
            self.battery_manager.set_ntimes_ut_dt(algorithm.ntimes_ut_dt)
            self.battery_manager.set_n_thread(algorithm.n_thread)
            self.battery_manager.setBMIN(algorithm.battery_min)
            self.battery_manager.setTimeHorizon(
                algorithm.time_horizon * algorithm.ntimes_ut_dt
            )
            self.battery_manager.setBatteryParameter(
                CHARGERATE, DISCHARGERATE, QUEUERATE, TIMEINTERVAL
            )
            self.battery_manager.setDemandSignal(demand)
        elif (
            algorithm.algorithm_type
            == Algorithm.BATTERY_CHARGE_SCHEDULING.value
        ):
            self.battery_manager = BatteryChargeScheduling(
                3, 0, self.black_board
            )
            self.battery_manager.set_bmin(algorithm.battery_min)
            self.battery_manager.set_battery_parameter(
                CHARGERATE, DISCHARGERATE, QUEUERATE, TIMEINTERVAL
            )
            self.battery_manager.set_demand_signal(demand)
            self.battery_manager.setTimeHorizon(
                algorithm.time_horizon
            )

    def behaviour(self):
        self.battery_manager.solve()
        print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
        for robot in self.robot:
            self.robot[robot].behaviour()
        print("________________________________________")
        self.count += 1


def fleet_loop(fleet, sim_time, interval):
    simulation_time = sim_time
    for i in range(0, simulation_time, interval):
        fleet.behaviour()
        time.sleep(0.04 * sim_to_real)


def main():
    global TIMEINTERVAL
    global OUTPUTPATH
    global TIMEHORIZON

    # Get necessary input for running algorithm
    parser = argparse.ArgumentParser(description="Battery Management System")
    parser.add_argument(
        "-s",
        dest="scenario",
        type=str,
        help="scenario file which contain parameter to run the algorithm",
    )
    args = parser.parse_args()
    if args.scenario is None:
        print("Error: Scenario file is required.")
        exit()

    scenario_json_obj = None
    with open(args.scenario, "r") as file:
        scenario_json_obj = json.load(file)

    algorithm_json = scenario_json_obj.get("algorithm")
    algorithm_dto = AlgorithmDto(**algorithm_json)
    demand_json = scenario_json_obj.get("demand")
    demand_dto = None

    if algorithm_dto.algorithm_type in [
        Algorithm.GOAL_PROGRAMMING.value,
        Algorithm.GOAL_PROGRAMMING_SAMPLED_DATA_CONTROL.value,
    ]:
        demand_dto = DemandDto(**demand_json)

    goal_programming_parameter_json = scenario_json_obj.get(
        "goal_programming_parameter"
    )
    goal_programming_parameter_dto = None
    if algorithm_dto.algorithm_type in [
        Algorithm.GOAL_PROGRAMMING.value,
        Algorithm.GOAL_PROGRAMMING_SAMPLED_DATA_CONTROL.value,
    ]:
        goal_programming_parameter_dto = GoalProgrammingParameterDto(
            **goal_programming_parameter_json
        )

    del scenario_json_obj["algorithm"]
    del scenario_json_obj["demand"]
    del scenario_json_obj["goal_programming_parameter"]
    battery_scenario_dto = BatteryScenarioDto(
        **scenario_json_obj,
        algorithm=algorithm_dto,
        demand=demand_dto,
        goal_programming_parameter=goal_programming_parameter_dto,
    )

    TIMEINTERVAL = battery_scenario_dto.dt
    TIMEHORIZON = battery_scenario_dto.algorithm.time_horizon

    global_blackboard = BlackBoard()

    number_of_robot = len(battery_scenario_dto.robot_initial_battery)

    OUTPUTPATH = f"{battery_scenario_dto.scenario}.csv"

    demand_signal = None
    if algorithm_dto.algorithm_type in [
        Algorithm.GOAL_PROGRAMMING.value,
        Algorithm.GOAL_PROGRAMMING_SAMPLED_DATA_CONTROL.value,
    ]:
        demand_signal = DemandSignal(
            getattr(DemandType, demand_dto.demand_type),
            demand_dto.min_value,
            demand_dto.max_value,
            demand_dto.time_period,
            demand_dto.step,
        )

    fleet = Fleet(1, 0, global_blackboard)
    fleet.set_battery_manager(
        battery_scenario_dto.algorithm, demand_signal, goal_programming_parameter_dto
    )

    for robot_battery in battery_scenario_dto.robot_initial_battery:
        fleet.add_robot(float(robot_battery))
        print(robot_battery)

    for _ in range(battery_scenario_dto.charge_station):
        fleet.add_charge_station()

    t = threading.Thread(
        target=fleet_loop,
        args=[fleet, battery_scenario_dto.time, battery_scenario_dto.dt],
    )
    t.start()
    t.join()


if __name__ == "__main__":
    main()
