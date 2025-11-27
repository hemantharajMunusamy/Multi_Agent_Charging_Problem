from dataclasses import asdict, dataclass
import enum


class PlotType(enum.Enum):
    BAR = "bar"
    WORK_MODE_AVERAGE_COMPARE = "work_mode_average_compare"
    WORK_MODE_AVERAGE_COMPARE_VARYING_CHARGE_STATION = (
        "work_mode_average_compare_varying_charge_station"
    )
    WORK_MODE_AVERAGE_COMPARE_VARYING_NUMBER_OF_ROBOTS = (
        "work_mode_average_compare_varying_number_of_robots"
    )
    ENERGY_CONSUMPTION_COMPARE_VARYING_CHARGE_STATION = (
        "energy_consumption_compare_varying_charge_station"
    )
    RANDOM = "random"
    TRIANGULAR_ACHIEVABLE = "triangular_achievable"
    TRIANGULAR_NOT_ACHIEVABLE = "triangular_not_achievable"
    TRIANGULAR_NOT_ACHIEVABLE_TH = "triangular_not_achievable_th"
    TRIANGULAR_NOT_ACHIEVABLE_DT = "triangular_not_achievable_dt"
    TRIANGULAR_NOT_ACHIEVABLE_NC = "triangular_not_achievable_nc"
    DIFFERENCE_DEMAND_COMPUTATIONAL_TIME = "difference_demand_computational_time"
    DIFFERENCE_DEMAND_DELTA_TIME = "difference_demand_delta_time"
    PARETO = "pareto"


@dataclass
class PlotterDto(object):
    plot_type: PlotType
    plot_data_1: str
    plot_data_2: str
    plot_data_3: str
    plot_data_4: str
    plot_data_5: str
