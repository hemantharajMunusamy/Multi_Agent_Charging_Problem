from dataclasses import asdict, dataclass
from typing import List
import enum


class DataProcessingType(enum.Enum):
    """List of all the available data processing functionality"""

    CALC_CONSUMED_ENERGY = "calc_consumed_energy"
    CALC_CONSUMED_ENERGY_AND_DEMAND = "calc_consumed_energy_and_demand"


@dataclass
class DataProcessingDto(object):
    """Definition of how to send data processing file"""

    data_processing_type: DataProcessingType
    file_locations: List[str]
