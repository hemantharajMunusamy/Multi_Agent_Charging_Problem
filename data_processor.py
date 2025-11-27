import argparse
import json
import pandas as pd
from data_model.data_processor_dto import DataProcessingDto, DataProcessingType


def calculate_consumed_energy(files: list[str]) -> None:
    """Find average amount of charge consumed by the robot"""

    for i in range(len(files)):
        df = pd.read_csv(files[i])
        amount_charge: float = 0.0
        previous_charge = {}

        for time, rid, battery in zip(df["time"], df["rid"], df["battery"]):
            if rid in previous_charge:
                if int(time - previous_charge[rid][0]) == 3:
                    # print(previous_charge[rid], time, rid, battery)
                    if previous_charge[rid][1] < battery:
                        amount_charge += battery - previous_charge[rid][1]
                    previous_charge[rid] = (time, battery)
                    # print(amount_charge)
            else:
                previous_charge[rid] = (time, battery)

        print(amount_charge)


def calculate_demand_difference_and_computational_time(file: str) -> None:
    df = pd.read_csv(file)
    diff = df["demand_target"] - df["demand_actual"]
    print(diff.mean())
    print(df["computation_time"].mean())


def main() -> None:
    """Main"""

    # Get necessary input for running algorithm
    parser = argparse.ArgumentParser(description="Data Processing for BMS")
    parser.add_argument(
        "-dp",
        dest="data_processing_file",
        type=str,
        help="File needs to be processed which contains bms data",
    )

    args = parser.parse_args()
    if args.data_processing_file is None:
        print("Error: Data Processing File is required.")
        exit()

    processing_json_obj = None
    with open(args.data_processing_file, "r") as file:
        processing_json_obj = json.load(file)

    processing_dto = DataProcessingDto(**processing_json_obj)

    if (
        processing_dto.data_processing_type
        == DataProcessingType.CALC_CONSUMED_ENERGY.value
    ):
        calculate_consumed_energy(processing_dto.file_locations)
    elif (
        processing_dto.data_processing_type
        == DataProcessingType.CALC_CONSUMED_ENERGY_AND_DEMAND.value
    ):
        calculate_demand_difference_and_computational_time(
            processing_dto.file_locations[0]
        )
        calculate_consumed_energy([processing_dto.file_locations[1]])
    else:
        print("Not Found")


if __name__ == "__main__":
    main()
