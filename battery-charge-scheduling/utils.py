""" Utility function to support the battery charge scheduler"""
import os
import yaml


def read_battery_model(charging_fname:str, discharging_fname:str):
    """ Read the battery model from the given file name"""
    try:
        with open (charging_fname, encoding='utf-8') as f_charge:
            charge_model = yaml.load(f_charge, Loader=yaml.SafeLoader)

        with open (discharging_fname, encoding='utf-8') as f_discharge:
            discharge_model = yaml.load(f_discharge, Loader=yaml.SafeLoader)

        return charge_model, discharge_model
    
    except FileNotFoundError:
        raise ValueError("Files aren't found.")
    

def calculate_battery_model_from_transition(charging_fname:str, discharging_fname:str):
    """
        Given: b_curr: {b_next1 : b_next1_count, b_next2 : b_next2_count, ..., b_nextn : b_next2_count}
        Using battery transition statistics to find a transition probabilities
    """
    return read_battery_model(charging_fname, discharging_fname)