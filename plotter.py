import enum

import numpy as np
import pandas as pd
import sys
import argparse
import json
import csv
import random as rand
import math
import time
from data_model.PlotterDto import *
from DemandSignal import DemandSignal, DemandType
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

FONTSIZE = 16
TICKFONTSIZE = 12

PLT = None


class barPlot:
    def __init__(self, filename):
        self.binSize = 10
        self.multiplot = 3
        self.batteryX = [i for i in range(0, 100, self.binSize)]
        self.color = ["r", "g", "b", "c"]
        self.label = None
        self.batteryHeight = None
        self.batteryPercentageBin = None
        self.width = None
        self.ylabel = None
        self.file_name = f"{filename}.pdf"
        self.pp = PdfPages(f"{filename}.pdf")

    def setBinSize(self, sz):
        self.binSize = sz
        self.batteryX = [i for i in range(self.binSize // 2, 100, self.binSize)]

    def setParameterSingleBarPlot(self, data, binSz):
        self.width = 1.0
        self.ylabel = "Number of robots"
        self.multiplot = 1
        self.setBinSize(binSz)
        self.batteryHeight = self.setHeight(data)
        self.plotBin()

    def multi3DPlot(self):
        fig = PLT.figure()
        ax1 = fig.add_subplot(111, projection="3d")

        y = [1 for i in range(len(self.batteryHeight[0]))]

        x1 = [1 for i in range(len(self.batteryHeight[0]))]
        y1 = [10 * i for i in range(len(self.batteryHeight[0]))]
        z1 = [5 for i in range(len(self.batteryHeight[0]))]

        x2 = [5 for i in range(len(self.batteryHeight[0]))]
        x3 = [10 for i in range(len(self.batteryHeight[0]))]
        x4 = [15 for i in range(len(self.batteryHeight[0]))]

        print(self.batteryHeight)
        ax1.bar3d(x1, y1, z1, y, y, self.batteryHeight[0], color="r")
        b1 = plt.Rectangle((0, 0), 1, 1, fc="r")
        ax1.bar3d(x2, y1, z1, y, y, self.batteryHeight[1], color="g")
        b2 = plt.Rectangle((0, 0), 1, 1, fc="g")
        ax1.bar3d(x3, y1, z1, y, y, self.batteryHeight[2], color="b")
        b3 = plt.Rectangle((0, 0), 1, 1, fc="b")
        ax1.bar3d(x4, y1, z1, y, y, self.batteryHeight[3], color="c")
        b4 = plt.Rectangle((0, 0), 1, 1, fc="c")

        ax1.legend(
            [b1, b2, b3, b4],
            ["T=0", "T=42", "T=81", "T=120"],
            bbox_to_anchor=(1.165, 1.1),
        )
        # ax1.xaxis.set_ticks([0, 42, 81, 120])
        # ax1.xaxis.set_ticklabels(["", "0", "", "42", "", "81", "", "120"])
        ax1.xaxis.set_ticklabels(["", "0", "", "42", "", "81", "", "120"])

        ax1.set_xlabel("Time Instance", labelpad=7)
        # ax1.text(x=5, y=-1, z=0, rotation=1.3, s="Time Instance", fontsize=12)
        ax1.set_ylabel("Battery Percentage", labelpad=7)
        ax1.set_zlabel("Number of robots", labelpad=6)

        ax1.view_init(elev=22, azim=-132)

        fig.savefig("scenario_1_milp_bar_plot.pdf")
        # plt.show()

    def setParameterMultiBarPlot(self, data, binSz, dataSplit, robotSize):
        self.width = 2.5
        self.ylabel = "Number of robots"
        self.multiplot = dataSplit
        self.setBinSize(binSz)
        self.batteryHeight = []
        self.label = ["T=0", "T=42", "T=81", "T=120"]
        # for datS in range(0, len(data), len(data) // dataSplit):
        data_idx = [int(i) for i in np.linspace(0, len(data) - robotSize, dataSplit)]
        print(np.linspace(0, len(data) - robotSize, dataSplit))
        for datS in data_idx:
            print(f"data split {datS}")
            # self.batteryHeight.append(self.setHeight(data[datS:datS+(len(data)//dataSplit)]))
            self.batteryHeight.append(self.setHeight(data[datS : datS + robotSize]))
        self.multi3DPlot()
        # self.multiPlotBin()
        self.pp.close()

    def setParameterMultiStackBarPlot(self, data, binSz, dataSplit):
        self.width = 2.5
        self.ylabel = "Number of robots"
        self.multiplot = dataSplit
        self.setBinSize(binSz)
        self.batteryHeight = []
        self.label = []
        for datS in range(0, len(data), len(data) // dataSplit):
            print(f"data split {datS}")
            self.label.append(f"{datS}")
            # self.batteryHeight.append(self.setHeight(data[datS:datS+(len(data)//dataSplit)]))
            self.batteryHeight.append(self.setHeight(data[datS : datS + 10]))
        self.multiStackPlotBin()

    def calculateHist(self, data):
        self.batteryPercentageBin = dict(
            [i, 0] for i in range(self.binSize // 2, 100, self.binSize)
        )
        for dat in data:
            for bin in self.batteryPercentageBin.keys():
                delta = self.binSize // 2
                if (bin - delta) < dat <= (bin + delta):
                    self.batteryPercentageBin[bin] = self.batteryPercentageBin[bin] + 1

    def setHeight(self, data):
        self.calculateHist(data)
        return [
            self.batteryPercentageBin[i]
            for i in range(self.binSize // 2, 100, self.binSize)
        ]

    def multiPlotBin(self):
        start = -self.multiplot * (self.width // 2)
        count = 0
        print(self.label)
        f = plt.figure()
        for batteryHeight in self.batteryHeight:
            batteryX = [bx + (start * self.width) for bx in self.batteryX]
            start += (self.width - 0.5) // 2
            print(batteryHeight)
            print(batteryX)
            plt.bar(
                x=batteryX,
                height=batteryHeight,
                width=self.width,
                color=self.color[count],
                label="0e090",
            )
            plt.xlabel("Battery Percentage", fontsize=FONTSIZE)
            plt.ylabel(self.ylabel, fontsize=FONTSIZE)
            plt.legend(self.label)
            count += 1
        f.savefig(self.file_name)
        self.pp.savefig(plt.gcf())
        self.pp.close()
        # plt.show()

    def multiStackPlotBin(self):
        start = -self.multiplot * (self.width // 2)
        count = 0
        fig, ax = plt.subplots()
        bottom = [0 for _ in range(self.binSize)]
        print(self.label)
        f = plt.figure()
        for batteryHeight in self.batteryHeight:
            plt.bar(
                x=self.batteryX,
                height=batteryHeight,
                width=self.width,
                color=self.color[count],
                bottom=bottom,
            )
            plt.xlabel("Battery Percentage")
            plt.ylabel(self.ylabel)
            plt.legend(self.label)
            bottom = [bottom[i] + batteryHeight[i] for i in range(len(bottom))]
            count += 1
        f.savefig("test.pdf")
        self.pp.savefig(plt.gcf())
        self.pp.close()
        # plt.show()

    def plotBin(self):
        plt.bar(x=self.batteryX, height=self.batteryHeight, width=self.width)
        plt.ylabel(self.ylabel)
        # plt.show()


class WorkModePlot:
    def __init__(self, file: str):
        self.timeMode = None
        self.color = ["c", "g", "r", "b"]
        self.xlabel = "Time (min)"
        self.ylabel = "Number of working robots"
        self.files = []
        self.df = None
        self.file_name = f"{file}.pdf"

    def getWorkPlot(self, files, legends):
        count = 0
        f = PLT.figure()
        for file in files:
            self.df = None
            self.df = pd.read_csv(file)
            self.calculateWorking()
            print(self.color[count])
            self.plotWorking(self.color[count], legends[count])
            count += 1
        # save_figure(self.file_name, f)
        f.savefig(self.file_name)
        # PLT.show()

    def calculateWorking(self):
        self.timeMode = {i: 0 for i in range(0, 123, 3)}
        for i in range(len(self.df)):
            if self.df.loc[i, "mode"] == 1:
                self.timeMode[self.df.loc[i, "time"]] += 1

    def plotWorking(self, clr, leg):
        data = sorted(self.timeMode.items())
        x, y = zip(*data)
        print(sum(y) / len(y))
        PLT.plot(
            x,
            y,
            # color=clr,
            label=leg,
        )
        print(leg)
        # legend(plt)
        PLT.legend()
        # x_label(self.xlabel, plt)
        PLT.xlabel(self.xlabel)
        # x_ticks(plt)
        PLT.xticks()
        # y_label(self.ylabel, plt)
        PLT.ylabel(self.ylabel)
        # y_ticks(plt)
        PLT.yticks()


class BatteryDataPlot:
    def __init__(self, file):
        self.clr = ["c", "g", "r", "b"]
        self.x_label = "Time (min)"
        self.y_label = "Battery Percentage"
        self.file = file
        self.df = pd.read_csv(file)
        self.x = [i for i in range(len(self.df[self.df["rid"] == 0]["battery"]))]

    # def getWorkPlot(self,files,legends):
    #     count = 0
    #     for file in files:
    #         self.df = None
    #         self.df = pd.read_csv(file)
    #         self.calculateWorking()
    #         print(self.color[count])
    #         self.plotBattery(self.color[count], legends[count])
    #         count += 1

    def plot_working(self):
        self.working_mode = {i: 0 for i in range(3, 123, 3)}
        for i in range(len(self.df)):
            if self.df.loc[i, "mode"] == 1:
                self.working_mode[self.df.loc[i, "time"]] += 1
        data = sorted(self.working_mode.items())
        x, y = zip(*data)
        # plt.stem(x, y, label="working robot",markerfmt="Dr",bottom=30)
        PLT.plot(
            x,
            y,
            label="Working Robots",
            linestyle="dashed",
            color="#FFA500",
            # linewidth=1.2,
        )
        # x_label("Time", plt)
        PLT.xlabel("Time (min)")
        # x_ticks(plt)
        # y_label("Number of robots in working mode", plt)
        PLT.ylabel("Number of robots in working mode")
        # y_ticks(plt)
        # legend(plt)
        PLT.legend()

    def plot_battery(self):
        fig = PLT.subplots()
        for i in range(40):
            PLT.plot(
                self.x,
                self.df[self.df["rid"] == i]["battery"],
                color=self.clr[i % len(self.clr)],
            )
        # plt.legend()
        # x_label(self.x_label, plt)
        PLT.xlabel(self.x_label)
        # x_ticks(plt)
        # y_label(self.y_label, plt)
        PLT.ylabel(self.y_label)
        # y_ticks(plt)
        # PLT.show()


def update_plt(plt) -> None:
    plt.rcParams.update(
        {
            "figure.figsize": (6, 4),
            "font.size": 12,
            "lines.linewidth": 2,
            "lines.markersize": 6,
            "axes.labelsize": 12,
            "axes.titlesize": 12,
            "xtick.labelsize": 10,
            "ytick.labelsize": 10,
            "legend.fontsize": 10,
            "figure.dpi": 300,
        }
    )


def construct_s2_trapezoidal_not_achievable(file1: str):
    demand = DemandSignal(DemandType.TRIANGULAR_STEP, 37, 40, 39, 3)
    # demand = DemandSignal(DemandType.TRIANGULAR_STEP, 33, 38, 39, 2)
    data = []
    f = PLT.figure()
    for i in np.linspace(3, 120, 40):
        data.append(demand.get_demand_value(i))

    # data = [
    #     34,
    #     35,
    #     35,
    #     36,
    #     37,
    #     38,
    #     38.0,
    #     38.0,
    #     37.0,
    #     36.0,
    #     35.0,
    #     35.0,
    #     33,
    #     34,
    #     35,
    #     35,
    #     36,
    #     37,
    #     38,
    #     38.0,
    #     38.0,
    #     37.0,
    #     36.0,
    #     35.0,
    #     35.0,
    #     33,
    #     34,
    #     35,
    #     35,
    #     36,
    #     37,
    #     38,
    #     38.0,
    #     38.0,
    #     37.0,
    #     36.0,
    #     35.0,
    #     35.0,
    #     33,
    #     34,
    # ]

    PLT.plot(
        np.linspace(3, 120, 40),
        data,
        label="Demand",
        color="#A9A9A9",
        # linewidth=3.5
    )

    battery_plot = BatteryDataPlot(file1)
    battery_plot.plot_working()
    PLT.ylim(30, 40.5)
    # PLT.show()
    # save_figure("scenario_2_triangular_not_achievable.pdf", f)
    # f.savefig("scenario_2_triangular_not_achievable.pdf")
    f.savefig("scenario_2_triangular_not_achievable_partial.pdf")


def construct_s2_trapezoidal_achievable(file1: str):
    demand = DemandSignal(DemandType.TRIANGULAR_STEP, 32, 35, 39, 3)
    # demand = DemandSignal(DemandType.TRIANGULAR_STEP, 34, 37, 39, 3)

    data = []
    f = PLT.figure()
    for i in np.linspace(3, 120, 40):
        data.append(demand.get_demand_value(i))
    PLT.plot(
        np.linspace(3, 120, 40),
        data,
        label="Demand",
        color="#A9A9A9",
        # linewidth=3.5
    )

    battery_plot = BatteryDataPlot(file1)
    battery_plot.plot_working()
    PLT.ylim(30, 40.5)
    # x_ticks(plt)
    # y_ticks(plt)
    # save_figure("scenario_2_triangular_achievable.pdf", f)
    f.savefig("scenario_2_triangular_achievable.pdf")


def construct_s2_random(file1: str):
    demand = DemandSignal(DemandType.RANDOM, 30, 40, 1, 1)
    data = []
    f = PLT.figure()
    for i in np.linspace(3, 120, 40):
        data.append(demand.get_demand_value(i))
    PLT.plot(
        np.linspace(3, 120, 40), data, label="Demand", color="#A9A9A9", linewidth=3.5
    )

    battery_plot = BatteryDataPlot(file1)
    battery_plot.plot_working()

    # save_figure("scenario_2_random.pdf", f)
    f.savefig("scenario_2_random.pdf")


def construct_s1_compare_working_mode(file1: str, file2: str):
    files = [file2, file1]
    leg = ["Rule-Based", "MILP-Based"]
    a = WorkModePlot("scenario_1_average")
    a.getWorkPlot(files=files, legends=leg)


def construct_s1_compare_working_mode_varying_charge_station(files: str):
    for i in range(5):
        if files[i] == "":
            continue
        df = pd.read_csv(files[i])
        diff = df["demand_target"] - df["demand_actual"]
        print(diff.mean(), df["computation_time"].mean())
        PLT.plot(df["demand_target"])
        PLT.plot(df["demand_actual"])
    # PLT.show()


def construct_s1_bar_plot(file: str):
    plotReader = BatteryDataPlot(file)
    plotReader.plot_battery()
    a = barPlot("scenario_1_rulebased")
    a.multiplot = 5
    # a.setParameterSingleBarPlot(plotReader.df["battery"], 10)
    a.setParameterMultiBarPlot(plotReader.df["battery"], 10, 4, 40)


def construct_s2_trapezoidal_not_achievable_th(files):
    for i in range(5):
        df = pd.read_csv(files[i])
        diff = df["demand_target"] - df["demand_actual"]
        print(diff.mean(), df["computation_time"].mean())
        PLT.plot(df["demand_target"])
        PLT.plot(df["demand_actual"])

    # PLT.show()


def construct_s2_trapezoidal_not_achievable_nc():
    pass


def plot_diff_demand(no_charge_station: set, horizon_data: dict, diff_demand: dict):
    f = PLT.figure()
    for nc in no_charge_station:
        PLT.plot(
            horizon_data[nc],
            diff_demand[nc],
            label=r"$N_{C}$ = " + str(int(nc)),
            marker="o",
        )

    PLT.xticks([3, 6, 9, 12, 15])
    # plt.annotate(r'Time t=m $\Delta$ t', xy=(16,2),xytext=(17, 2),arrowprops=dict(facecolor='red', shrink=0.05))
    # x_label(r"Time horizon t= m$\Delta$t", plt)
    PLT.xlabel(r"Time horizon $T_{h} = m \Delta$t")
    # y_label("Robot Shortfall", plt)
    PLT.ylabel("Robot Shortfall")
    # y_ticks(plt)
    # legend(plt)
    PLT.legend()
    # PLT.show()
    # save_figure("scenario_3_diff_demand.pdf", f)
    f.savefig("scenario_3_diff_demand.pdf")


def plot_computational_time(
    no_charge_station: set, horizon_data: dict, computational_time: dict
):
    f = PLT.figure()
    for nc in no_charge_station:
        plt.plot(
            horizon_data[nc],
            computational_time[nc],
            label=r"$N_{C}$ = " + str(int(nc)),
            marker="o",
        )

    PLT.xticks([3, 6, 9, 12, 15])
    # plt.annotate(r'Time t=m $\Delta$ t', xy=(16,2),xytext=(17, 2),arrowprops=dict(facecolor='red', shrink=0.05))
    # x_label(r"Time horizon t= m$\Delta$t", plt)
    PLT.xlabel(r"Time horizon $T_{h} = m \Delta$t")
    # x_ticks(plt)
    # y_label("Computation Time (s)", plt)
    PLT.ylabel("Computation Time (s)")
    PLT.yscale("log")
    # y_ticks(plt)
    # legend(plt)
    PLT.legend()
    # PLT.show()
    # save_figure("scenario_3_computational_time.pdf", f)
    f.savefig("scenario_3_computational_time.pdf")


def construct_s3_diff_demand_computational_time(file: str):
    df = pd.read_csv(file)
    number_charge_station = set()
    horizon_data = {}
    diff_demand = {}
    computional_time = {}

    for val in df["nc"]:
        number_charge_station.add(val)

    for val in number_charge_station:
        horizon_data[val] = []
        diff_demand[val] = []
        computional_time[val] = []

    for nc, horizon, demand, com_time in zip(
        df["nc"], df["horizon"], df["diff_demand"], df["computational_time"]
    ):
        horizon_data[nc].append(horizon)
        diff_demand[nc].append(demand)
        computional_time[nc].append(com_time)

    plot_diff_demand(number_charge_station, horizon_data, diff_demand)
    plot_computational_time(number_charge_station, horizon_data, computional_time)


def construct_s4_trapezoidal_not_achievable_dt(files):
    for i in range(5):
        df = pd.read_csv(files[i])
        diff = df["demand_target"] - df["demand_actual"]
        print(diff.mean(), df["computation_time"].mean())
        PLT.plot(df["time"], df["demand_target"], marker=".", linestyle="none")
        PLT.plot(df["time"], df["demand_actual"], marker=".")
        # plt.plot(df['demand_actual'])

    # PLT.show()


def construct_s4_diff_demand_delta_time(file):
    df = pd.read_csv(file)
    for nc in range(5, 7):
        f = PLT.figure()
        for h in range(2, 5):
            out = df[(df["charge_station"] == nc) & (df["horizon_length"] == h)]
            print(out)
            PLT.plot(
                out["delta_time"],
                out["demand_difference_mean"],
                marker="o",
                label="m = " + str(h),
            )
        # x_label(r"$\Delta$t", plt)
        PLT.xlabel(r"$\Delta$t")
        # x_ticks(plt)
        # y_label("Robot Shortfall", plt)
        PLT.ylabel("Robot Shortfall")
        # y_ticks(plt)
        # legend(plt)
        PLT.legend()
        f.savefig(
            "scenario_4_robot_shortfal_with_fixed_charge_station_"
            + str(nc)
            + " varying_horizon.pdf",
        )
        # PLT.show()
    print(df[(df["charge_station"] == 5) & (df["horizon_length"] == 3)])


def construct_s1_compare_consumed_energy(file: str) -> None:
    df = pd.read_csv(file)
    width = 0.25
    f = PLT.figure()
    # plt.subplot(1, 2, 1)
    PLT.bar(
        df[(df["algo_type"] == 0)]["charge_station"] - width / 2,
        df[(df["algo_type"] == 0)]["consumed_charge"] * 40.0,
        width,
        label="Rule-Based",
    )

    PLT.bar(
        df[(df["algo_type"] == 1)]["charge_station"] + width / 2,
        df[(df["algo_type"] == 1)]["consumed_charge"] * 40.0,
        width,
        label="MILP-Based",
    )
    # x_label("Number of charge stations", plt)
    PLT.xlabel("Number of charge stations")
    # x_ticks(plt)
    # y_label("Total Consumed Energy (Wh)", plt)
    PLT.ylabel("Total Battery Percentage Charged (%)")
    # y_ticks(plt)
    # legend(plt)
    PLT.legend()

    # plt.subplot(1, 2, 2)
    # plt.bar(
    #     df[(df["algo_type"] == 0)]["charge_station"] - width / 2,
    #     df[(df["algo_type"] == 0)]["demand_difference_mean"],
    #     width,
    #     label="RuleBased",
    # )
    # plt.bar(
    #     df[(df["algo_type"] == 1)]["charge_station"] + width / 2,
    #     df[(df["algo_type"] == 1)]["demand_difference_mean"],
    #     width,
    #     label="GoalProgramming",
    # )
    # plt.xlabel("Number of charge stations")
    # plt.ylabel("Robot shortfall")
    # PLT.show()
    # save_figure("scenario_1_compare_consumed_energy.pdf", f)
    f.savefig("scenario_1_compare_consumed_energy.pdf")


def construct_s1_compare_varying_number_of_robots(file: str) -> None:
    df = pd.read_csv(file)
    f = PLT.figure()
    PLT.plot(
        df[(df["algo_type"] == 0)]["number_of_robots"],
        df[(df["algo_type"] == 0)]["computational_time"],
        label="Rule-Based",
        marker="o",
    )
    print(df[(df["algo_type"] == 1)]["number_of_robots"])
    x_2 = np.array(df[(df["algo_type"] == 1)]["number_of_robots"])
    y_2 = np.array(df[(df["algo_type"] == 1)]["computational_time"])
    print(x_2, y_2)
    PLT.plot(
        x_2,
        y_2,
        label="MILP Case-1",
        marker="o",
    )

    x_3 = np.array(df[(df["algo_type"] == 2)]["number_of_robots"])
    y_3 = np.array(df[(df["algo_type"] == 2)]["computational_time"])
    print(x_3, y_3)
    PLT.plot(
        x_3,
        y_3,
        label="MILP Case-2",
        marker="o",
    )

    PLT.xticks([20, 40, 60, 80, 100, 120, 140, 160, 180, 200])
    PLT.yscale("log")
    # x_label("Number of robots", plt)
    PLT.xlabel("Number of robots")
    # y_label("Computation Time (s)", plt)
    PLT.ylabel("Computation Time (s)")
    # legend(plt)
    PLT.legend()
    # PLT.xlim(19.5, 200.5)
    PLT.grid(True)
    PLT.tight_layout()
    # PLT.show()
    # save_figure("scenario_1_compare_computational_time.pdf", f)
    PLT.savefig("scenario_1_compare_computational_time.pdf")


def find_pareto_front(s5: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    pareto_front: pd.DataFrame = pd.DataFrame(
        columns=[
            "w1",
            "w2",
            "w3",
            "w4",
            "w5",
            "demand_difference",
            "computation_time",
            "consumed_charge",
        ]
    )

    nonpareto_front: pd.DataFrame = pd.DataFrame(
        columns=[
            "w1",
            "w2",
            "w3",
            "w4",
            "w5",
            "demand_difference",
            "computation_time",
            "consumed_charge",
        ]
    )
    pareto = {}
    print(s5.index)
    for i in s5.index:
        is_dominated = False
        dominated_keys = []
        print(s5)
        for key in pareto.keys():
            if (s5.loc[key]["demand_difference"] < s5.loc[i]["demand_difference"]) and (
                s5.loc[key]["consumed_charge"] < s5.loc[i]["consumed_charge"]
            ):
                is_dominated = True
                break
            elif (
                s5.loc[i]["demand_difference"] < s5.loc[key]["demand_difference"]
            ) and (s5.loc[i]["consumed_charge"] < s5.loc[key]["consumed_charge"]):
                dominated_keys.append(key)

        # remove all dominated solution and add non-dominated into the dominated keys
        for key in dominated_keys:
            nonpareto_front.loc[len(nonpareto_front)] = s5.loc[key].copy()
            del pareto[key]

        if (len(dominated_keys) == 0) and (is_dominated):
            nonpareto_front.loc[len(nonpareto_front)] = s5.loc[i].copy()

        if not is_dominated:
            pareto[i] = s5.loc[i].copy()

    sorted_pareto = dict(
        sorted(pareto.items(), key=lambda item: item[1]["demand_difference"])
    )
    for key in sorted_pareto:
        pareto_front.loc[len(pareto_front)] = s5.loc[key].copy()

    return (pareto_front, nonpareto_front)


def construct_s5_pareto(file: str) -> None:
    df = pd.read_csv(file)

    # (pareto, nonpareto) = find_pareto_front(df[df["w5"] >= 0.0001])
    (pareto, nonpareto) = find_pareto_front(df[df["demand_difference"] <= 10.0])

    print(len(pareto), len(nonpareto))

    # for i in range(len(pareto)):
    #     print(pareto.loc[i])
    #     if pareto.loc[i]["w5"] > 0.1:
    #         pareto_gt_01[len(pareto_gt_01)] = pareto.loc[i]
    #     else:
    #         pareto_lt_01[len(pareto_lt_01)] = pareto.loc[i]

    #     print("GT", pareto_gt_01)
    #     print("LT", pareto_lt_01)

    # print(pareto_gt_01)
    # print(pareto_lt_01)
    f = PLT.figure()
    PLT.scatter(
        nonpareto["demand_difference"],
        nonpareto["consumed_charge"],
        marker=".",
        linewidths=0.01,
        # label="Dominated solutions",
    )

    print(
        "Non-pareto w1 => Min: ", nonpareto["w1"].min(), "Max: ", nonpareto["w1"].max()
    )
    print(
        "Non-pareto w2 => Min: ", nonpareto["w2"].min(), "Max: ", nonpareto["w2"].max()
    )
    print(
        "Non-pareto w3 => Min: ", nonpareto["w3"].min(), "Max: ", nonpareto["w3"].max()
    )
    print(
        "Non-pareto w4 => Min: ", nonpareto["w4"].min(), "Max: ", nonpareto["w4"].max()
    )
    print(
        "Non-pareto w5 => Min: ", nonpareto["w5"].min(), "Max: ", nonpareto["w5"].max()
    )

    print("pareto w1 => Min: ", pareto["w1"].min(), "Max: ", pareto["w1"].max())
    print("pareto w2 => Min: ", pareto["w2"].min(), "Max: ", pareto["w2"].max())
    print("pareto w3 => Min: ", pareto["w3"].min(), "Max: ", pareto["w3"].max())
    print("pareto w4 => Min: ", pareto["w4"].min(), "Max: ", pareto["w4"].max())
    print("pareto w5 => Min: ", pareto["w5"].min(), "Max: ", pareto["w5"].max())
    PLT.plot(
        pareto["demand_difference"],
        pareto["consumed_charge"],
        marker="*",
        color="orange",
        markersize=4,
        markeredgewidth=0.1,
        markeredgecolor="black",
        linewidth=1,
        # label="a1 > 0.0001",
    )

    # PLT.plot(
    #     pareto[(pareto["w5"] <= 0.1) & (pareto["w5"] > 0.00001)]["demand_difference"],
    #     pareto[(pareto["w5"] <= 0.1) & (pareto["w5"] > 0.00001)]["consumed_charge"],
    #     marker="*",
    #     color="red",
    #     markersize=4,
    #     markeredgewidth=0.1,
    #     markeredgecolor="black",
    #     linewidth=1,
    #     label="Non-Dominated 0.1 >= alpha > 0.000001",
    # )

    # PLT.plot(
    #     pareto[(pareto["w5"] <= 0.000001) & (pareto["w5"] >= 0.00000001)][
    #         "demand_difference"
    #     ],
    #     pareto[(pareto["w5"] <= 0.000001) & (pareto["w5"] >= 0.00000001)][
    #         "consumed_charge"
    #     ],
    #     marker="*",
    #     color="green",
    #     markersize=4,
    #     markeredgewidth=0.1,
    #     markeredgecolor="black",
    #     linewidth=1,
    #     label=".000001 >= a1>.00000001",
    # )

    # PLT.plot(
    #     pareto[(pareto["w5"] <= 0.00000001)]["demand_difference"],
    #     pareto[(pareto["w5"] <= 0.00000001)]["consumed_charge"],
    #     marker="*",
    #     color="yellow",
    #     markersize=4,
    #     markeredgewidth=0.1,
    #     markeredgecolor="black",
    #     linewidth=1,
    #     label=".00000001 >=a1",
    # )

    # x_label("Number of robots", plt)
    PLT.xlabel("Robot Shortfall")
    # PLT.yscale("log")
    # y_label("Computation Time (s)", plt)
    PLT.ylabel("Total Battery Percentage Charged (%)")
    # legend(plt)
    # PLT.legend()
    # PLT.xlim(19.5, 200.5)
    # PLT.grid(True)
    PLT.tight_layout()
    # PLT.show()
    # save_figure("scenario_1_compare_computational_time.pdf", f)
    PLT.savefig("scenario_5_pareto.pdf")


def main():
    global PLT
    PLT = plt
    update_plt(PLT)
    # Get necessary input for running algorithm
    parser = argparse.ArgumentParser(description="Plotter of BMS")
    parser.add_argument(
        "-plt",
        dest="plotter_data",
        type=str,
        help="Plotter data file which contains bms data",
    )
    args = parser.parse_args()
    if args.plotter_data is None:
        print("Error: plotter file is required.")
        exit()

    plotter_json_obj = None
    with open(args.plotter_data, "r") as file:
        plotter_json_obj = json.load(file)

    plotter_dto = PlotterDto(**plotter_json_obj)

    if plotter_dto.plot_type == PlotType.BAR.value:
        construct_s1_bar_plot(plotter_dto.plot_data_1)
    elif plotter_dto.plot_type == PlotType.WORK_MODE_AVERAGE_COMPARE.value:
        construct_s1_compare_working_mode(
            plotter_dto.plot_data_1, plotter_dto.plot_data_2
        )
    elif (
        plotter_dto.plot_type
        == PlotType.WORK_MODE_AVERAGE_COMPARE_VARYING_CHARGE_STATION.value
    ):
        files = [
            plotter_dto.plot_data_1,
            plotter_dto.plot_data_2,
            plotter_dto.plot_data_3,
            plotter_dto.plot_data_4,
            plotter_dto.plot_data_5,
        ]
        construct_s1_compare_working_mode_varying_charge_station(files)
    elif (
        plotter_dto.plot_type
        == PlotType.WORK_MODE_AVERAGE_COMPARE_VARYING_NUMBER_OF_ROBOTS.value
    ):
        construct_s1_compare_varying_number_of_robots(plotter_dto.plot_data_1)
    elif plotter_dto.plot_type == PlotType.RANDOM.value:
        construct_s2_random(plotter_dto.plot_data_1)
    elif plotter_dto.plot_type == PlotType.TRIANGULAR_ACHIEVABLE.value:
        construct_s2_trapezoidal_achievable(plotter_dto.plot_data_1)
    elif plotter_dto.plot_type == PlotType.TRIANGULAR_NOT_ACHIEVABLE.value:
        construct_s2_trapezoidal_not_achievable(plotter_dto.plot_data_1)
    elif plotter_dto.plot_type == PlotType.TRIANGULAR_NOT_ACHIEVABLE_TH.value:
        files = [
            plotter_dto.plot_data_1,
            plotter_dto.plot_data_2,
            plotter_dto.plot_data_3,
            plotter_dto.plot_data_4,
            plotter_dto.plot_data_5,
        ]
        construct_s2_trapezoidal_not_achievable_th(files)
    elif plotter_dto.plot_type == PlotType.TRIANGULAR_NOT_ACHIEVABLE_DT.value:
        files = [
            plotter_dto.plot_data_1,
            plotter_dto.plot_data_2,
            plotter_dto.plot_data_3,
            plotter_dto.plot_data_4,
            plotter_dto.plot_data_5,
        ]
        construct_s4_trapezoidal_not_achievable_dt(files)
    elif plotter_dto.plot_type == PlotType.TRIANGULAR_NOT_ACHIEVABLE_NC.value:
        construct_s2_trapezoidal_not_achievable_nc()
    elif plotter_dto.plot_type == PlotType.DIFFERENCE_DEMAND_COMPUTATIONAL_TIME.value:
        construct_s3_diff_demand_computational_time(plotter_dto.plot_data_1)
    elif plotter_dto.plot_type == PlotType.DIFFERENCE_DEMAND_DELTA_TIME.value:
        construct_s4_diff_demand_delta_time(plotter_dto.plot_data_1)
    elif (
        plotter_dto.plot_type
        == PlotType.ENERGY_CONSUMPTION_COMPARE_VARYING_CHARGE_STATION.value
    ):
        construct_s1_compare_consumed_energy(plotter_dto.plot_data_1)
    elif plotter_dto.plot_type == PlotType.PARETO.value:
        construct_s5_pareto(plotter_dto.plot_data_1)
    else:
        demand = DemandSignal(DemandType.RANDOM, 30, 40, 1, 1)
        data = []
        for i in np.linspace(3, 120, 40):
            data.append(demand.get_demand_value(i))

        # plt.stem(np.linspace(3, 120,40),data, label="demand",markerfmt="b", bottom=30)
        plt.plot(np.linspace(3, 120, 40), data, label="demand")
        # plt.show()
        #
        battery_plot = BatteryDataPlot(plotter_dto.plot_data_1)
        battery_plot.plot_working()
        # #
        battery_plot.plot_battery()
        # df[0:10], df[10:20]
        # df[400:410]
        #
        #
        # work = len(df[df['mode'] == 1]) / len(df)
        # charge = len(df[df['mode'] == 0]) / len(df)
        # queue = len(df[df['mode'] == 2]) / len(df)
        #
        # work, charge, queue
        #

        #

        # bin_size = 100
        # data = []
        # for i in range(3):
        #     print(f"{i * bin_size} {(i * bin_size) + 10}")
        #     plt.hist(df['battery'][i * bin_size:(i * bin_size) + 10], histtype='stepfilled',
        #              bins=[0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100], alpha=0.3)
        #     plt.show()


if __name__ == "__main__":
    main()
