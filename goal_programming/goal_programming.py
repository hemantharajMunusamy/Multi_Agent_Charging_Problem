import time
from math import ceil, floor
from operator import itemgetter, mod
from data_model.data import *
from data_model.BatteryScenarioDto import GoalProgrammingParameterDto
from black_board import *
import pulp as lp
import csv
from pathlib import Path

OUTPUTPATH = Path("./output/goal_programming_output_milp.csv")


class Goal_Programming:
    def __init__(self, g_id, m_id, global_blackboard):
        # Global ID
        self.global_id = g_id
        self.module_id = m_id
        # Centralized Data store
        self.black_board = global_blackboard
        self.node_id = self.black_board.register_node(
            self.global_key(), f"Node:Goal_Programming {self.global_id}"
        )

        self.Nr = None
        self.Nc = None
        self.m = None

        self.charge_rate = None
        self.discharge_rate = None
        self.queue_rate = None
        self.dt = None
        self.ntimes_ut_dt = (
            None  # ut (control sample time) = self.ntimes_ut_dt * self.dt
        )
        self.time = 0
        self.BMIN = None
        self.algo_type = None
        self.n_thread = None

        self.wModeLowerBoundMinus = None
        self.bLowerBoundMinus = None
        self.bUpperBoundPlus = None

        self.wModeLowerBound = None
        self.bLowerBound = None
        self.bUpperBound = None
        self.alpha_weights = None

        self.demandSignal = None

        self.C = None
        self.W = None
        self.prev_work_mode = None
        self.prev_time = None

        self.goal_programming_filepath = OUTPUTPATH
        self.goal_programming_filepath.open(mode="w+", encoding='utf-8').close()
        self.goal_programming_file = self.goal_programming_filepath.open(mode="a", encoding='utf-8')
        self.battery_writer = csv.writer(self.goal_programming_file)
        if self.module_id == 0:
            self.battery_writer.writerow(
                [
                    "time",
                    "nr",
                    "cr",
                    "demand_target",
                    "demand_actual",
                    "computation_time",
                ]
            )

        self.battery_percentage = []
        self.robotGid = []

    def global_key(self):
        return (self.global_id * 100) + self.module_id

    def setup(self):
        self.black_board.register(self.global_key(), 0, Data.GID)
        self.black_board.register(self.global_key(), 0, Data.RID)
        self.black_board.register(self.global_key(), 0, Data.Battery)
        self.black_board.register(self.global_key(), 0, Data.Battery_Mode)

    def set_algorithm_type(self, algo_type: Algorithm):
        self.algo_type = algo_type

    def set_n_thread(self, n_thread: int):
        self.n_thread = n_thread

    def set_goal_programming_parameter(
        self, goal_programming_parameter: GoalProgrammingParameterDto
    ):
        self.alpha_weights = goal_programming_parameter.alpha_weights
        self.bUpperBound = goal_programming_parameter.obj3_battery_upper_bound
        self.bLowerBound = goal_programming_parameter.obj2_battery_lower_bound
        self.wModeLowerBound = goal_programming_parameter.obj1_work_mode_lower_bound

    def set_ntimes_ut_dt(self, ntimes_ut_dt: int):
        self.ntimes_ut_dt = ntimes_ut_dt

    def setBMIN(self, bmin):
        self.BMIN = bmin

    def setDemandSignal(self, demand):
        self.demandSignal = demand

    def setTimeHorizon(self, timeHorizon: int):
        self.m = timeHorizon

    def setBatteryParameter(
        self, charge_rate: float, discharge_rate: float, queue_rate: float, dt: float
    ):
        self.dt = dt
        self.charge_rate = charge_rate * self.dt
        self.discharge_rate = discharge_rate * self.dt
        self.queue_rate = queue_rate * self.dt

    def initialize_parameter(self):
        self.Nr = len(self.battery_percentage)
        self.Nc = len(self.charge_station)

    def get_battery_percentage(self):
        battery_percentage = self.black_board.merge_all(
            [Data.GID, Data.RID, Data.Battery, Data.Battery_Mode]
        )
        # only get active robot battery percentage
        self.battery_percentage = []
        for r in range(len(battery_percentage)):
            if battery_percentage[r][3] != Mode_Of_Operation.Off_Mode:
                self.battery_percentage.append(battery_percentage[r])
        print(f"Battery Percentage {self.battery_percentage}")

    def get_charge_station(self):
        self.charge_station = self.black_board.merge_all(
            [Data.GID, Data.Charge_Station_Mode, Data.Charge_Station_Robot]
        )
        print(f"Charge Station {self.charge_station}")

    def maxMILP(self):
        self.get_battery_percentage()
        self.get_charge_station()
        self.initialize_parameter()
        # Create a LP Minimization problem
        Lp_prob = lp.LpProblem("Problem", lp.LpMaximize)

        # Create problem Variables
        self.X = []  # battery percentage
        for r in range(self.Nr):  # Number of robot
            x = []
            for t in range(self.m):  # Time Horizon
                x.append(
                    lp.LpVariable(
                        f"x{r}{t}", lowBound=20, upBound=100, cat=lp.LpContinuous
                    )
                )
            self.X.append(x)

        self.C = (
            []
        )  # Charging mode of the robot Cij {i - ith robot and jth time in minutes slot}
        for r in range(self.Nr):
            c = []
            for t in range(self.m):
                c.append(
                    lp.LpVariable(f"c{r}{t}", lowBound=0, upBound=1, cat=lp.LpBinary)
                )
            self.C.append(c)

        self.W = (
            []
        )  # Working mode of the robot Wij {ith robot and jth time in minutes slot}
        for r in range(self.Nr):
            w = []
            for t in range(self.m):
                w.append(
                    lp.LpVariable(f"w{r}{t}", lowBound=0, upBound=1, cat=lp.LpBinary)
                )
            self.W.append(w)

        # Objective Function Maximize the working mode of the robot
        Lp_prob += lp.lpSum(self.W)

        # Battery Min Condition
        for r in range(self.Nr):
            for t in range(self.m):
                Lp_prob += self.X[r][t] >= self.BMIN

        # Initialize robot battery
        for r in range(len(self.battery_percentage)):
            Lp_prob += self.X[r][0] == self.battery_percentage[r][2]

        # Battery Dynamic
        for r in range(self.Nr):
            for t in range(self.m - 1):
                Lp_prob += (
                    self.X[r][t + 1]
                    - self.X[r][t]
                    - (self.C[r][t] * self.charge_rate)
                    + (self.W[r][t] * self.discharge_rate)
                    + ((1 - self.C[r][t] - self.W[r][t]) * self.queue_rate)
                    == 0
                )

        # Fixed Charger constraint at any time for all the robot:
        for t in range(self.m):
            Lp_prob += lp.lpSum(self.C[r][t] for r in range(self.Nr)) <= self.Nc

        # Robot can be in any one of the one mode possible
        for r in range(self.Nr):
            for t in range(self.m):
                Lp_prob += self.C[r][t] + self.W[r][t] <= 1

        # solver = lp.CPLEX_PY()
        # solver.buildSolverModel(Lp_prob)
        # solver.solverModel.parameer.timelimit.set(60)
        # solver.callSolver(Lp_prob)
        # solver = solver.findSolutionValues(Lp_prob)
        # status = Lp_prob.solve(lp.PULP_CBC_CMD(timeLimit=60))   # Solver
        # status = Lp_prob.solve(lp.GLPK_CMD(timeLimit=60))

        status = Lp_prob.solve(lp.PULP_CBC_CMD(msg=False, timeLimit=60))

        print(self.charge_rate, self.discharge_rate, self.queue_rate)
        print([self.X[0][t].value() for t in range(self.m)])
        print(f"Work {[self.W[r][0].value() for r in range(self.Nr)]}")
        sumList = [self.C[r][0].value() for r in range(self.Nr)]
        sum = 0
        for val in sumList:
            sum += val
        print(f"Charge {sumList} and Total {sum}")
        print(lp.LpStatus[status])  # The solution status
        print(lp)

    def defn_sampled_control_variable(self):
        # Create problem Variables
        self.X = []  # battery percentage
        for r in range(self.Nr):  # Number of robot
            x = []
            for t in range(self.m + 1):  # Time Horizon [Modified]
                x.append(
                    lp.LpVariable(f"x_r{r}_t{t}", upBound=100, cat=lp.LpContinuous)
                )
            self.X.append(x)

        self.C = (
            []
        )  # Charging mode of the robot Cij {i - ith robot and jth time in minutes slot}
        for r in range(self.Nr):
            c = []
            for t in range(self.m):  # Modified
                if (t % self.ntimes_ut_dt) == 0:  # Sampled control
                    c.append(
                        lp.LpVariable(
                            f"c_r{r}_t{t}", lowBound=0, upBound=1, cat=lp.LpBinary
                        )
                    )
                else:
                    c.append(None)
            self.C.append(c)

        self.W = (
            []
        )  # Working mode of the robot Wij {ith robot and jth time in minutes slot}
        for r in range(self.Nr):
            w = []
            for t in range(self.m):  # Modified
                if (t % self.ntimes_ut_dt) == 0:  # Sampled control
                    w.append(
                        lp.LpVariable(
                            f"w_r{r}_t{t}", lowBound=0, upBound=1, cat=lp.LpBinary
                        )
                    )
                else:
                    w.append(None)
            self.W.append(w)

        print(self.W)

        self.WorkMode = []  # Number of the working robot at a time
        for t in range(self.m):
            self.WorkMode.append(
                lp.LpVariable(f"workMode{t}", lowBound=0, cat=lp.LpInteger)
            )

    def defn_variable(self):
        # Create problem Variables
        self.X = []  # battery percentage
        for r in range(self.Nr):  # Number of robot
            x = []
            for t in range(self.m + 1):  # Time Horizon [Modified]
                x.append(
                    lp.LpVariable(f"x_r{r}_t{t}", upBound=100, cat=lp.LpContinuous)
                )
            self.X.append(x)

        self.C = (
            []
        )  # Charging mode of the robot Cij {i - ith robot and jth time in minutes slot}
        for r in range(self.Nr):
            c = []
            for t in range(self.m):
                c.append(
                    lp.LpVariable(
                        f"c_r{r}_t{t}", lowBound=0, upBound=1, cat=lp.LpBinary
                    )
                )
            self.C.append(c)

        self.W = (
            []
        )  # Working mode of the robot Wij {ith robot and jth time in minutes slot}
        for r in range(self.Nr):
            w = []
            for t in range(self.m):
                w.append(
                    lp.LpVariable(
                        f"w_r{r}_t{t}", lowBound=0, upBound=1, cat=lp.LpBinary
                    )
                )
            self.W.append(w)

        self.WorkMode = []  # Number of the working robot at a time
        for t in range(self.m):
            self.WorkMode.append(
                lp.LpVariable(f"workMode{t}", lowBound=0, cat=lp.LpInteger)
            )

    def defn_goal_programming_variable(self):
        self.wModeLowerBoundMinus = lp.LpVariable(
            f"wModeLowerBoundMinus", lowBound=0, cat=lp.LpContinuous
        )
        self.bLowerBoundMinus = lp.LpVariable(
            f"bLowerBoundMinus", lowBound=0, cat=lp.LpContinuous
        )
        self.bUpperBoundPlus = lp.LpVariable(
            "bUpperBoundPlus", lowBound=0, cat=lp.LpContinuous
        )

        self.demandPlus = []
        for t in range(self.m):
            self.demandPlus.append(
                lp.LpVariable(f"demandPlus{t}", lowBound=0, cat=lp.LpContinuous)
            )

        self.demandMinus = []
        for t in range(self.m):
            self.demandMinus.append(
                lp.LpVariable(f"demandMinus{t}", lowBound=0, cat=lp.LpContinuous)
            )

    def sum_working_robot_sampled_control(self, lp_prob):
        for t in range(self.m):
            t_hat = t - (t % self.ntimes_ut_dt)
            lp_prob += (
                self.WorkMode[t] - lp.lpSum(self.W[r][t_hat] for r in range(self.Nr))
                == 0
            )

    def sum_working_robot(self, lp_prob):
        for t in range(self.m):
            lp_prob += (
                self.WorkMode[t] - lp.lpSum(self.W[r][t] for r in range(self.Nr)) == 0
            )

    def fixed_charge_station_sampled_control(self, lp_prob):
        for t in range(self.m):
            if (t % self.ntimes_ut_dt) == 0:
                lp_prob += lp.lpSum(self.C[r][t] for r in range(self.Nr)) <= self.Nc

    def fixed_charge_station(self, lp_prob):
        # Fixed Charger constraint at any time for all the robot:
        for t in range(self.m):
            lp_prob += lp.lpSum(self.C[r][t] for r in range(self.Nr)) <= self.Nc

    def battery_dynamic_sampled_control(self, lpProb):
        # Battery Dynamic
        for r in range(self.Nr):
            for t in range(self.m):  # [Modified]
                t_hat = t - (t % self.ntimes_ut_dt)
                lpProb += (
                    self.X[r][t + 1]
                    - self.X[r][t]
                    - (self.C[r][t_hat] * self.charge_rate)
                    + (self.W[r][t_hat] * self.discharge_rate)
                    + ((1 - self.C[r][t_hat] - self.W[r][t_hat]) * self.queue_rate)
                    == 0
                )

    def batteryDynamic(self, lpProb):
        # Battery Dynamic
        for r in range(self.Nr):
            for t in range(self.m):  # [Modified]
                lpProb += (
                    self.X[r][t + 1]
                    - self.X[r][t]
                    - (self.C[r][t] * self.charge_rate)
                    + (self.W[r][t] * self.discharge_rate)
                    + ((1 - self.C[r][t] - self.W[r][t]) * self.queue_rate)
                    == 0
                )

    def mode_constraint_sampled_control(self, lpProb):
        # Robot can be in any one of the one mode possible
        for r in range(self.Nr):
            for t in range(self.m):
                if (t % self.ntimes_ut_dt) == 0:
                    lpProb += self.C[r][t] + self.W[r][t] <= 1

    def modeConstraint(self, lpProb):
        # Robot can be in any one of the one mode possible
        for r in range(self.Nr):
            for t in range(self.m):
                lpProb += self.C[r][t] + self.W[r][t] <= 1

    def initializeBattery(self, lpProb):
        # Initialize robot battery
        for r in range(len(self.battery_percentage)):
            lpProb += self.X[r][0] == self.battery_percentage[r][2]

    def goal_programming_variable_constraint_sampled_control(self, lp_prob):
        for r in range(self.Nr):
            for t in range(self.m):
                t_hat = t - (t % self.ntimes_ut_dt)
                lp_prob += (
                    self.X[r][t]
                    + self.wModeLowerBoundMinus
                    - (self.wModeLowerBound * self.W[r][t_hat])
                    >= 0.0
                )

        for r in range(self.Nr):
            for t in range(self.m):  # [Modified]
                lp_prob += (
                    self.X[r][t] + self.bLowerBoundMinus - self.bLowerBound >= 0.0
                )

        for r in range(self.Nr):
            for t in range(self.m):  # [Modified]
                lp_prob += self.X[r][t] - self.bUpperBoundPlus - self.bUpperBound <= 0.0

        for t in range(self.m):
            lp_prob += (
                self.WorkMode[t]
                - self.demandPlus[t]
                - self.demandSignal.get_demand_value(self.time + (t * self.dt))
                <= 0.0
            )

        for t in range(self.m):
            lp_prob += (
                self.WorkMode[t]
                + self.demandMinus[t]
                - self.demandSignal.get_demand_value(self.time + (t * self.dt))
                >= 0.0
            )

    def goal_programming_variable_constraint(self, lp_prob):
        for r in range(self.Nr):
            for t in range(self.m):
                lp_prob += (
                    self.X[r][t]
                    + self.wModeLowerBoundMinus
                    - (self.wModeLowerBound * self.W[r][t])
                    >= 0.0
                )

        for r in range(self.Nr):
            for t in range(self.m + 1):  # [Modified]
                lp_prob += (
                    self.X[r][t] + self.bLowerBoundMinus - self.bLowerBound >= 0.0
                )

        for r in range(self.Nr):
            for t in range(self.m + 1):  # [Modified]
                lp_prob += self.X[r][t] - self.bUpperBoundPlus - self.bUpperBound <= 0.0

        for t in range(self.m):
            lp_prob += (
                self.WorkMode[t]
                - self.demandPlus[t]
                - self.demandSignal.get_demand_value(self.time + (t * self.dt))
                <= 0.0
            )

        for t in range(self.m):
            lp_prob += (
                self.WorkMode[t]
                + self.demandMinus[t]
                - self.demandSignal.get_demand_value(self.time + (t * self.dt))
                >= 0.0
            )

    def maximizeWorkObjective(self, lpProb):
        # Objective Function Maximize the working mode of the robot
        lpProb += lp.lpSum(self.WorkMode)

    def weightedGoalProgramming(self, lpProb):
        # Objective Function
        lpProb += (
            (self.alpha_weights[1] * self.wModeLowerBoundMinus)
            + (self.alpha_weights[2] * self.bLowerBoundMinus)
            + (self.alpha_weights[3] * self.bUpperBoundPlus)
            + (
                self.alpha_weights[4]
                * (lp.lpSum(self.demandMinus) + lp.lpSum(self.demandPlus))
            )
        )

    def demand_goal_programming(self, lp_prob):
        lp_prob += lp.lpSum(self.demandMinus) + lp.lpSum(self.demandPlus)

    def store_data(self, demand_target, demand_actual, computation_time):
        self.goal_programming_file = self.goal_programming_filepath.open(mode="a", encoding='utf-8')
        self.battery_writer = csv.writer(self.goal_programming_file)
        self.battery_writer.writerow(
            [
                self.time,
                self.Nr,
                self.Nc,
                demand_target,
                demand_actual,
                computation_time,
            ]
        )
        self.goal_programming_file.close()

    def milp_bms(self):
        self.time += self.dt
        self.get_battery_percentage()
        self.get_charge_station()
        self.initialize_parameter()
        # Create a LP Minimization problem
        Lp_prob = None
        Lp_prob = lp.LpProblem("Problem", lp.LpMaximize)
        self.defn_variable()
        self.maximizeWorkObjective(Lp_prob)
        # Battery Min Condition
        for r in range(self.Nr):
            for t in range(self.m + 1):  # Modified
                Lp_prob += self.X[r][t] >= self.BMIN
        self.initializeBattery(Lp_prob)
        self.sum_working_robot(Lp_prob)
        self.batteryDynamic(Lp_prob)
        self.fixed_charge_station(Lp_prob)
        self.modeConstraint(Lp_prob)
        start_time = time.time()
        status = Lp_prob.solve(
            lp.PULP_CBC_CMD(msg=False, timeLimit=60, threads=self.n_thread)
        )
        end_time = time.time()
        if status == lp.LpStatusInfeasible:
            print("Infeasible solution")
            return False
        print(f"Initial battery {[self.X[r][0].value() for r in range(self.Nr)]}")
        print([self.X[0][t].value() for t in range(self.m)])
        print(f"Objective value {Lp_prob.objective.value()}")
        print(f"WorkModeDecision {[self.W[r][0].value() for r in range(self.Nr)]}")
        print(f"ChargeModeDecision {[self.C[r][0].value() for r in range(self.Nr)]}")
        print(f"WorkMode {[self.WorkMode[t].value() for t in range(self.m)]}")

        sumList = [self.C[r][0].value() for r in range(self.Nr)]
        sum = 0
        for val in sumList:
            sum += val
        print(f"Charge {sumList} and Total {sum}")
        print(lp.LpStatus[status])  # The solution status
        # print(Lp_prob)
        return True

    def goal_programming_bms(self):
        self.time += self.dt
        self.get_battery_percentage()
        self.get_charge_station()
        self.initialize_parameter()
        # Create a LP Minimization problem
        Lp_prob = None
        Lp_prob = lp.LpProblem("Problem", lp.LpMinimize)
        self.defn_variable()
        self.defn_goal_programming_variable()
        self.weightedGoalProgramming(Lp_prob)
        # Battery Min Condition
        for r in range(self.Nr):
            for t in range(self.m + 1):  # Modified
                Lp_prob += self.X[r][t] >= self.BMIN
        self.initializeBattery(Lp_prob)
        self.sum_working_robot(Lp_prob)
        self.batteryDynamic(Lp_prob)
        self.fixed_charge_station(Lp_prob)
        self.modeConstraint(Lp_prob)
        self.goal_programming_variable_constraint(Lp_prob)
        start_time = time.time()
        status = Lp_prob.solve(
            lp.PULP_CBC_CMD(msg=False, timeLimit=60, threads=self.n_thread)
        )
        end_time = time.time()
        if status == lp.LpStatusInfeasible:
            print("Infeasible solution")
            return False
        print(f"Initial battery {[self.X[r][0].value() for r in range(self.Nr)]}")
        print([self.X[0][t].value() for t in range(self.m)])
        print(f"Objective value {Lp_prob.objective.value()}")
        print(f"WorkModeDecision {[self.W[r][0].value() for r in range(self.Nr)]}")
        print(f"ChargeModeDecision {[self.C[r][0].value() for r in range(self.Nr)]}")
        print(f"WorkMode {[self.WorkMode[t].value() for t in range(self.m)]}")
        print(f"battery Upper Bound Plus {self.bUpperBoundPlus.value()}")
        print(f"battery Lower Bound Minus {self.bLowerBoundMinus.value()}")
        print(
            f"Demand {[self.demandSignal.get_demand_value(self.time + (t * self.dt)) for t in range(self.m)]}"
        )
        print(f"WorkMode {[self.WorkMode[t].value() for t in range(self.m)]}")
        print(f"DemandMius {[self.demandMinus[t].value() for t in range(self.m)]}")
        print(f"DemandPlus {[self.demandPlus[t].value() for t in range(self.m)]}")
        if Lp_prob.status == lp.LpStatusInfeasible:
            self.store_data(
                self.demandSignal.get_demand_value(self.time),
                0.0,
                end_time - start_time,
            )
        else:
            self.store_data(
                self.demandSignal.get_demand_value(self.time),
                self.WorkMode[0].value(),
                end_time - start_time,
            )
        sumList = [self.C[r][0].value() for r in range(self.Nr)]
        sum = 0
        for val in sumList:
            sum += val
        print(f"Charge {sumList} and Total {sum}")
        print(lp.LpStatus[status])  # The solution status
        return True

    def goal_programming_sampled_data_control_bms(self):
        # Condition to stop solving for goal programming
        if (int(self.time) % self.ntimes_ut_dt) != 0:
            self.time += self.dt
            self.store_data(
                self.demandSignal.get_demand_value(self.time),
                self.prev_work_mode,
                self.prev_time,
            )
            return True
        else:
            print(
                f"Timeeeeeeeeeeeeeeeeeeeeee      {self.time} eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee"
            )
            print(
                "====000000000000000=============================00000000000000000000========="
            )
            self.time += self.dt

        self.get_battery_percentage()
        self.get_charge_station()
        self.initialize_parameter()
        # Create a LP Minimization problem
        Lp_prob = None
        Lp_prob = lp.LpProblem("Problem", lp.LpMinimize)
        self.defn_sampled_control_variable()
        self.defn_goal_programming_variable()
        self.weightedGoalProgramming(Lp_prob)
        # Battery Min Condition
        for r in range(self.Nr):
            for t in range(self.m):  # Modified
                Lp_prob += self.X[r][t] >= self.BMIN
        self.initializeBattery(Lp_prob)
        self.sum_working_robot_sampled_control(Lp_prob)
        self.battery_dynamic_sampled_control(Lp_prob)
        self.fixed_charge_station_sampled_control(Lp_prob)
        self.mode_constraint_sampled_control(Lp_prob)
        self.goal_programming_variable_constraint_sampled_control(Lp_prob)
        start_time = time.time()
        status = Lp_prob.solve(
            lp.PULP_CBC_CMD(msg=False, timeLimit=60, threads=self.n_thread)
        )
        end_time = time.time()
        if status == lp.LpStatusInfeasible:
            print("Infeasible solution")
            return False
        print(f"Initial battery {[self.X[r][0].value() for r in range(self.Nr)]}")
        print([self.X[0][t].value() for t in range(self.m)])
        print(f"Objective value {Lp_prob.objective.value()}")
        print(f"WorkModeDecision {[self.W[r][0].value() for r in range(self.Nr)]}")
        print(f"ChargeModeDecision {[self.C[r][0].value() for r in range(self.Nr)]}")
        print(f"WorkMode {[self.WorkMode[t].value() for t in range(self.m)]}")
        print(f"battery Upper Bound Plus {self.bUpperBoundPlus.value()}")
        print(f"battery Lower Bound Minus {self.bLowerBoundMinus.value()}")
        print(
            f"Demand {[self.demandSignal.get_demand_value(self.time + (t * self.dt)) for t in range(self.m)]}"
        )
        print(f"WorkMode {[self.WorkMode[t].value() for t in range(self.m)]}")
        print(f"DemandMius {[self.demandMinus[t].value() for t in range(self.m)]}")
        print(f"DemandPlus {[self.demandPlus[t].value() for t in range(self.m)]}")
        if Lp_prob.status == lp.LpStatusInfeasible:
            self.store_data(
                self.demandSignal.get_demand_value(self.time),
                0.0,
                end_time - start_time,
            )
        else:
            self.prev_work_mode = self.WorkMode[0].value()
            self.prev_time = end_time - start_time
            self.store_data(
                self.demandSignal.get_demand_value(self.time),
                self.WorkMode[0].value(),
                end_time - start_time,
            )
        sumList = [self.C[r][0].value() for r in range(self.Nr)]
        sum = 0
        for val in sumList:
            sum += val
        print(f"Charge {sumList} and Total {sum}")
        print(lp.LpStatus[status])  # The solution status
        return True

    def algorithm(self):
        self.time += self.dt
        self.get_battery_percentage()
        self.get_charge_station()
        self.initialize_parameter()
        # Create a LP Minimization problem
        Lp_prob = None
        if self.algo_type == Algorithm.MILP.value:
            Lp_prob = lp.LpProblem("Problem", lp.LpMaximize)
        elif self.algo_type == Algorithm.GOAL_PROGRAMMING.value:
            Lp_prob = lp.LpProblem("Problem", lp.LpMinimize)

        self.defn_variable()
        if self.algo_type == Algorithm.GOAL_PROGRAMMING.value:
            self.defn_goal_programming_variable()

        # self.weightedGoalProgramming(Lp_prob)
        if self.algo_type == Algorithm.MILP.value:
            self.maximizeWorkObjective(Lp_prob)
        elif self.algo_type == Algorithm.GOAL_PROGRAMMING.value:
            self.weightedGoalProgramming(Lp_prob)
        else:
            self.demand_goal_programming(Lp_prob)

        # Battery Min Condition
        for r in range(self.Nr):
            for t in range(self.m + 1):  # Modified
                Lp_prob += self.X[r][t] >= self.BMIN

        self.initializeBattery(Lp_prob)
        self.sum_working_robot(Lp_prob)
        self.batteryDynamic(Lp_prob)
        self.fixed_charge_station(Lp_prob)
        self.modeConstraint(Lp_prob)

        if self.algo_type == Algorithm.GOAL_PROGRAMMING.value:
            self.goal_programming_variable_constraint(Lp_prob)

        start_time = time.time()
        status = Lp_prob.solve(
            lp.PULP_CBC_CMD(msg=False, timeLimit=60, threads=self.n_thread)
        )
        end_time = time.time()

        if status == lp.LpStatusInfeasible:
            print("Infeasible solution")
            return False

        print(f"Initial battery {[self.X[r][0].value() for r in range(self.Nr)]}")
        print([self.X[0][t].value() for t in range(self.m)])
        print(f"Objective value {Lp_prob.objective.value()}")
        print(f"WorkModeDecision {[self.W[r][0].value() for r in range(self.Nr)]}")
        print(f"ChargeModeDecision {[self.C[r][0].value() for r in range(self.Nr)]}")
        print(f"WorkMode {[self.WorkMode[t].value() for t in range(self.m)]}")

        if self.algo_type == Algorithm.GOAL_PROGRAMMING.value:
            print(f"battery Upper Bound Plus {self.bUpperBoundPlus.value()}")
            print(f"battery Lower Bound Minus {self.bLowerBoundMinus.value()}")
            print(
                f"Demand {[self.demandSignal.get_demand_value(self.time + (t * self.dt)) for t in range(self.m)]}"
            )
            print(f"WorkMode {[self.WorkMode[t].value() for t in range(self.m)]}")
            print(f"DemandMius {[self.demandMinus[t].value() for t in range(self.m)]}")
            print(f"DemandPlus {[self.demandPlus[t].value() for t in range(self.m)]}")
            if Lp_prob.status == lp.LpStatusInfeasible:
                self.store_data(
                    self.demandSignal.get_demand_value(self.time),
                    0.0,
                    end_time - start_time,
                )
            else:
                self.store_data(
                    self.demandSignal.get_demand_value(self.time),
                    self.WorkMode[0].value(),
                    end_time - start_time,
                )

        sumList = [self.C[r][0].value() for r in range(self.Nr)]
        sum = 0
        for val in sumList:
            sum += val
        print(f"Charge {sumList} and Total {sum}")
        print(lp.LpStatus[status])  # The solution status
        # print(Lp_prob)
        return True

    def find_n_remove_dead_robot(self):
        min_robot_id = -1
        min_robot_battery = 1000
        for r in range(len(self.battery_percentage)):
            if (self.battery_percentage[r][2] != Mode_Of_Operation.Off_Mode) and (
                self.battery_percentage[r][2] < min_robot_battery
            ):
                min_robot_id = r
                min_robot_battery = self.battery_percentage[r][2]
        print("=========================0----------------0----------------")
        print(min_robot_id)
        print(min_robot_battery)
        if min_robot_id != -1:
            r_id = self.battery_percentage[min_robot_id][0]
            print(r_id)
            self.black_board.write(
                (r_id, Data.Battery_Mode), Mode_Of_Operation.Off_Mode
            )

    def solve(self):
        self.get_battery_percentage()
        self.get_charge_station()
        self.initialize_parameter()
        print(
            "============000000000000000000000000000000000000000000==================="
        )
        for i in range(self.Nr):
            if self.algo_type == Algorithm.GOAL_PROGRAMMING_SAMPLED_DATA_CONTROL.value:
                if self.goal_programming_sampled_data_control_bms():
                    break
            else:
                if self.algorithm():
                    break
            self.find_n_remove_dead_robot()

        self.robotGid = []
        for r in range(self.Nr):
            self.robotGid.append(self.battery_percentage[r][0])

        self.chargeStationGid = []
        for c in range(self.Nc):
            self.chargeStationGid.append(self.charge_station[c][0])

        if (self.C == None) or (self.W == None):
            return

        count = 0
        for r in range(self.Nr):
            if self.C[r][0].value() + self.W[r][0].value() == 0:
                print(f"Robot {r} is sent to queue")
                self.black_board.write(
                    (self.robotGid[r], Data.Charge_Robot_Info), [0, count, True]
                )
                count += 1

        for r in range(self.Nr):
            if self.W[r][0].value() == 1:
                print(f"Robot {r} is sent to work")
                self.black_board.write(
                    (self.robotGid[r], Data.Charge_Robot_Info), [-1, 0, False]
                )
        count = 0
        for r in range(self.Nr):
            if self.C[r][0].value() == 1:
                print(f"Robot {r} is in charging")
                cs_id = self.chargeStationGid[count]
                self.black_board.write(
                    (self.robotGid[r], Data.Charge_Time), self.dt + 1
                )
                battery = self.black_board.read((self.robotGid[r], Data.Battery))
                self.black_board.write(
                    (self.robotGid[r], Data.Charge_Robot_Info), [cs_id, 0, critical]
                )
                self.black_board.write(
                    (cs_id, Data.Charge_Station_Mode), Charge_Station_Mode.Occupied
                )
                count += 1
                print(f"GID {self.robotGid[r]}, CSID {cs_id}")
