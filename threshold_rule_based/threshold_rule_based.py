import time
from math import ceil, floor
from operator import itemgetter, mod
from data_model.data import Charge_Station_Mode, Mode_Of_Operation
from black_board import *
import csv
from pathlib import Path

OUTPUTPATH = Path("./output/goal_programming_output_milp.csv")

class ThresholdRuleBasedApproach:
    """Conventional fixed-threshold rule based controller (TRB) - charge if the battery falls below a fixed threshold (25% or 40%) and remain charging 
       until the battery is charged to 70%. The robot also charges if no tasks are available. 
    """
    def __init__(self, g_id, m_id, global_blackboard):
        #Global ID
        self.global_id = g_id
        self.module_id = m_id
        #Centralized Data store
        self.black_board = global_blackboard
        self.node_id = self.black_board.register_node(self.global_key(), f"Node:Rule_Based_Approach {self.global_id}")
        
        self.Tcharge = 15
        self.tau_c = 10.
        self.tau_d = 1.
        self.Bmin = 30.
        self.Bth = 40.
        self.Bmax = 70.
        self.Nc = -1
        self.Nc_free = -1
        self.battery_percentage = []
        self.assign_robot = []
        self.Nr = -1
        self.time = 0
        self.dt = -1
        self.charge_rate = -1
        self.queue_rate = -1
        self.discharge_rate = -1

        self.assign_robot_to_queue: list = []
        self.remove_robot_from_charge: list = []
        self.assign_robot_to_charge: list = []

        self.free_charge_station = set()

        self.goal_programming_filepath = OUTPUTPATH
        self.goal_programming_filepath.open(mode='w', encoding='utf-8').close()
        self.goal_programming_file = self.goal_programming_filepath.open(mode="a", encoding='utf-8')
        self.battery_writer = csv.writer(self.goal_programming_file)

    def global_key(self):
        return (self.global_id * 100) + self.module_id

    def setup(self):
        self.black_board.register(self.global_key(), 0, Data.GID)
        self.black_board.register(self.global_key(), 0, Data.RID)
        self.black_board.register(self.global_key(), 0, Data.Battery)
        self.black_board.register(self.global_key(), 0, Data.Battery_Mode)

    def set_battery_parameter(self, charge_rate: float, discharge_rate: float, queue_rate: float, dt: float):
        self.tau_c = charge_rate
        self.tau_d = discharge_rate
        self.charge_rate = self.tau_c * dt
        self.queue_rate = queue_rate * dt
        self.discharge_rate = self.tau_d * dt

    def set_battery_min(self, bmin):
        self.Bmin = bmin

    def set_dt(self, dt: float) -> None:
        self.dt = dt

    def initialize_parameter(self):
        self.Nc = 0
        self.find_free_charge_station()

    def find_free_charge_station(self):
        self.free_charge_station = set()
        for gid, mode, rid in self.charge_station:
            self.Nc += 1
            if mode == Charge_Station_Mode.Free:
                if gid in self.free_charge_station:
                    print(f"Duplicated in initialize parameter {self.free_charge_station}")
                self.free_charge_station.add(gid)

    def get_battery_percentage(self):
        battery_percentage = self.black_board.merge_all([Data.GID, Data.RID, Data.Battery, Data.Battery_Mode])
        self.Nr = len(battery_percentage)
        #only get active robot battery percentage 
        self.battery_percentage = []
        for r in range(len(battery_percentage)):
            if battery_percentage[r][3] != Mode_Of_Operation.Off_Mode:
                self.battery_percentage.append(battery_percentage[r])
        print(f"Battery Percentage {self.battery_percentage}")
    
    def get_charge_station(self):
        self.charge_station = self.black_board.merge_all([Data.GID, Data.Charge_Station_Mode, Data.Charge_Station_Robot])
        print(f"Charge Station {self.charge_station}")

    def algorithm(self):
        self.assign_robot_to_queue = []
        self.remove_robot_from_charge = []
        self.assign_robot_to_charge = []

        self.get_battery_percentage()
        self.get_charge_station()
        self.initialize_parameter()

        # Remove robot from charge state when it is charged above Bmax
        for idx, robot_battery in enumerate(self.battery_percentage):
            if robot_battery[3] == Mode_Of_Operation.Charge_Mode and robot_battery[2] > self.Bmax:
                self.remove_robot_from_charge.append(robot_battery[0])
        # Get all the robot for charging based on their battery less than threshold (Bth)
        robot_require_charging = []
        for idx, robot_battery in enumerate(self.battery_percentage):
            if robot_battery[3] != Mode_Of_Operation.Charge_Mode and robot_battery[2] <= self.Bth:
                robot_require_charging.append((robot_battery[0], robot_battery[2]))

        # Sort the robot based on their battery and assign robot to available charge state and remaining robot to queue
        available_charge_station = len(self.free_charge_station) + len(self.remove_robot_from_charge)
        sorted_robot_require_charging = sorted(robot_require_charging, key=itemgetter(1)) #sort based on battery

        for robot_battery in sorted_robot_require_charging:
            if available_charge_station > 0:
                self.assign_robot_to_charge.append(robot_battery[0])
                available_charge_station -= 1
            else :
                self.assign_robot_to_queue.append(robot_battery[0])
        
    def find_n_remove_dead_robot(self):
        self.get_battery_percentage()
        min_robot_id = -1
        min_robot_battery = 1000
        for r in range(len(self.battery_percentage)):
            if (self.battery_percentage[r][2] != Mode_Of_Operation.Off_Mode) and (self.battery_percentage[r][2] < min_robot_battery):
                min_robot_id = r
                min_robot_battery = self.battery_percentage[r][2]
        print("=========================0----------------0----------------")
        print(min_robot_id)
        print(min_robot_battery)
        if min_robot_id != -1:
            r_id = self.battery_percentage[min_robot_id][0]
            print(r_id)
            self.black_board.write((r_id, Data.Battery_Mode), Mode_Of_Operation.Off_Mode)

    def is_robot_below_bmin(self, battery_percentage: list) -> bool :
        """
            Check any of the robots battery percentage reach below bmin return true otherwise false
        """
        for r in battery_percentage:
            if r[2] < self.Bmin:
                return True
        return False

    def store_data(self, Nr, demand_target, demand_actual, computation_time):
        self.goal_programming_file = self.goal_programming_filepath.open(mode="a", encoding='utf-8')
        self.battery_writer = csv.writer(self.goal_programming_file)
        self.battery_writer.writerow([self.time, Nr,self.Nc, demand_target, demand_actual,computation_time])
        self.goal_programming_file.close()

    def solve(self):
        self.time += self.dt
       

        self.get_battery_percentage()
        self.get_charge_station()
        self.initialize_parameter()

        computation_time = None
        for i in range(self.Nr):
            start_time = time.time()
            self.algorithm()
            computation_time = time.time() - start_time
            battery_percentage = self.battery_percentage.copy()

            print(self.battery_percentage)    
            print(f"Robot goint to get Queue Station {self.assign_robot_to_queue}")
            print(f"Robot going to get charge Station {self.assign_robot_to_charge}")
            print(f"Robot going to remove from charge Station {self.remove_robot_from_charge}")

            for i in range(len(battery_percentage)):
                if battery_percentage[i][1] in self.assign_robot_to_queue:
                    battery_percentage[i][2] -= self.queue_rate
                elif battery_percentage[i][1] in self.assign_robot_to_charge:
                    battery_percentage[i][2] += self.charge_rate
                else:
                    battery_percentage[i][2] -= self.discharge_rate

            print(battery_percentage)
            if self.is_robot_below_bmin(battery_percentage):
                self.find_n_remove_dead_robot()
                continue
            break

        print(self.battery_percentage)
        print(f"Robot goint to get Queue Station {self.assign_robot_to_queue}")
        print(f"Robot going to get charge Station {self.assign_robot_to_charge}")
        print(f"Robot going to remove from charge Station {self.remove_robot_from_charge}")

        demand_actual = len(self.battery_percentage) - len(self.assign_robot_to_charge) - len(self.assign_robot_to_queue)
        self.store_data(self.Nr, self.Nr, demand_actual, computation_time)

        count = 0
        for gid in self.assign_robot_to_queue:
            cs_id = 0
            self.black_board.write((gid, Data.Charge_Robot_Info), [cs_id, count, True])
            count += 1

        for gid in self.remove_robot_from_charge:
            cs_id, q_id,iscritic = self.black_board.read((gid, Data.Charge_Robot_Info))
            if cs_id in self.free_charge_station:
                print(f"Duplicate in free charge station {self.free_charge_station}")
            self.free_charge_station.add(cs_id)
            if cs_id == 0:
                exit()
            self.black_board.write((cs_id, Data.Charge_Station_Mode), Charge_Station_Mode.Free)
            self.black_board.write((gid, Data.Charge_Robot_Info), [-1, -1, False])
            self.black_board.write((gid, Data.Charge_Time), 0)

        for gid in self.assign_robot_to_charge:
            self.black_board.write((gid, Data.Charge_Time), self.Tcharge)
            cs_id = self.free_charge_station.pop()
            battery = self.black_board.read((gid, Data.Battery))

            critical = False
            if battery < self.Bmin:
                critical = True

            if cs_id == 0:
                exit()
            self.black_board.write((gid, Data.Charge_Robot_Info), [cs_id, 0, critical])
            self.black_board.write((cs_id, Data.Charge_Station_Mode), Charge_Station_Mode.Occupied)

            print(f"GID {gid}, CSID {cs_id}, Charge_Time {self.Tcharge}")

        
        allocated_cs = {}

        for gid, rid, battery, mode in self.battery_percentage:
            charge_robot_info = self.black_board.read((gid, Data.Charge_Robot_Info))

            if charge_robot_info[0] == -1:
                continue
                
            if charge_robot_info[0] in allocated_cs:
                allocated_cs[charge_robot_info[0]].append(gid)
                #print(f"Wrong allocation Charge Station {charge_robot_info[0]} **************** {allocated_cs[charge_robot_info[0]]}")
            else:
                allocated_cs[charge_robot_info[0]] = [gid]

        print(f"Allocated Charge Station {allocated_cs}")