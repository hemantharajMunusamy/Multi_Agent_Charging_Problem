import matplotlib.pyplot as plt
import pandas as pd

def main():
    battery_data = pd.read_csv('./scenario_1_rulebased_nc_3.csv')
    print(battery_data)
    for i in range(40):
        plt.plot(battery_data[battery_data['rid'] == i]['time'],battery_data[battery_data['rid'] == i]['battery'], marker='*')
    plt.show()


if __name__ == "__main__":
    main()
