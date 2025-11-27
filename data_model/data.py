import enum
from logging import critical
from geometry import *

class Charge_Station_Mode(enum.Enum):
    Occupied = 0
    Free = 1

class Mode_Of_Operation(enum.Enum):
    Charge_Mode = 0
    Work_Mode = 1
    Queue_Mode = 2
    Off_Mode = 3
    On_Mode = 4

#Data Stored in BlackBoard
class Data(enum.Enum):
    #Global Address : int 
    GID = 0
    #robot id Data: int
    RID = 1
    #battery : float
    Battery = 2
    #Robot Mode : Enum(Mode_Of_Operation)
    Battery_Mode = 3
    #Charge Station Status
    Charge_Station_Mode = 4
    #Charge Station Robot 
    Charge_Station_Robot = 5
    #Charge Time : int 
    Charge_Time = 6
    #Charge Robot Info : [Charge_Station_ID (int), Queue_ID(int), Critical(bool)]
    Charge_Robot_Info = 7
    
#Events Stored in BlackBoard
class Event(enum.Enum):
    #Assign robot initial pose
    Initialize_Pose = 0
    #Robot Reached assigned Goal [Trigger for getting new goal]
    Goal_Reached = 1
    #Robot got new goal [Response of Goal Reached]
    Got_New_Goal = 2
    #Robot Moved to new location 
    Moved = 3
    #Local planner made local plan
    Got_LocalPlan = 4
    #Robot got time profile segment
    Got_Segment = 5
    #Robot controller waypoints
    Debug_Control_Profile = 6
    #Join Into Robot partition
    Join_Robot = 7
    #Leave from Robot partition
    Leave_Robot = 8
    #Start Simulation
    Start = 9
    #Initialize Layout
    Initialize_Layout = 10


class Algorithm(enum.Enum):
    RULE_BASED = 'rule_based'
    THRESHOLD_RULE_BASED = 'threshold_rule_based'
    MILP = 'milp'
    GOAL_PROGRAMMING = 'goal_programming'
    GOAL_PROGRAMMING_SAMPLED_DATA_CONTROL = 'goal_programming_sampled_data_control'
    def __str__(self):
        return self.value


class DemandType(enum.Enum):
    CONST = "const"
    SIN = "sin"
    RAMP = "ramp"
    SQUARE = "square"
    TRIANGULAR = "triangular"
    TRIANGULAR_STEP = "triangular_step"
    RANDOM = "random"