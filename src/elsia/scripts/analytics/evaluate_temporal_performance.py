#!/usr/bin/env python

"""
This script is used to evaluate the temporal performances for 
various odometry types. In particular generate a plot/report
depicting how much time it takes per odometry update
For our case we will be evaluating between rf2o odometry 
and one given by ELSIA (previously known as SAO)
---------------------------------------------------------------
ELSIA - Exploitative Localization using Structure Inside Aisles
SAO   - Structure Aided Odometry 
"""
# ros imports
import rospy
from nav_msgs.msg import Odometry

# plotting and other python utils
from pylab import *
import pylab
import numpy
import argparse
import os
from scipy import optimize

rf2o_odom_updates = []
sao_odom_updates = []


def parseArgs():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--rf2o-topic",
        default="/odom_rf2o",
        help="odometry topic for rf2o"
    )
    parser.add_argument(
        "--sao-topic",
        default="/odom_sao",
        help="odometry topic for sao"
    )
    parser.add_argument(
        "--out-dir",
        required=True,
        help="Path to save the output plot file"
    )
    args = parser.parse_args()

    return (args.rf2o_topic, args.sao_topic, args.out_dir)

# callback function for rf2o


def rf2oCb(msg):
    global rf2o_odom_updates

    t = msg.header.stamp.to_sec()
    rf2o_odom_updates.append(t)

# callback function for sao


def saoCb(msg):
    global sao_odom_updates

    t = msg.header.stamp.to_sec()
    sao_odom_updates.append(t)


def getUpdateInterval(choice):
    current = None
    if choice == "rf2o":
        current = rf2o_odom_updates
    elif choice == "sao":
        current = sao_odom_updates

    if len(current) == 0:
        print("No data found! Aborting")
        exit(1)

    interval = []
    # get updates from rf2o_odom_updates
    for i in range(1, len(current)):
        interval.append(
            current[i] - current[i-1]
        )

    return sorted(interval)


def getGaussianPlotData(data):
    mean = np.mean(data)
    var = np.var(data)
    sigma = np.sqrt(var)
    x = np.linspace(min(data), max(data))

    return x, mean, sigma

# function to plot the temporal peformance
# i.e how much time it takes to produce odometry
# updates


def plotTemporalPerformance(out_dir):
    # Set Params for pylab plotter
    params = {
        'axes.labelsize': 12,  # 8
        'font.size': 14,
        'legend.fontsize': 14,
        'xtick.labelsize': 12,
        'ytick.labelsize': 12,
        'text.usetex': True,
        'figure.figsize': [5, 5]
    }
    rcParams.update(params)
    grid()

    rf2o_odom_update_interval = getUpdateInterval("rf2o")
    sao_odom_update_interval = getUpdateInterval("sao")

    # Find Gaussians for the above time intervals
    rf2o_x, rf2o_mean, rf2o_sigma = getGaussianPlotData(
        rf2o_odom_update_interval)
    sao_x, sao_mean, sao_sigma = getGaussianPlotData(sao_odom_update_interval)

    _, axs = subplots(2)

    axs[0].hist(rf2o_odom_update_interval, density=False,
                bins=50, label="rf2o", color="r")
    axs[0].plot(rf2o_x, normpdf(rf2o_x, rf2o_mean, rf2o_sigma), color="g")
    axs[1].hist(sao_odom_update_interval, density=False,
                bins=50, label="sao", color="b")
    axs[1].plot(sao_x, normpdf(sao_x, sao_mean, sao_sigma), color="g")

    axs[0].set_ylabel("Frequency")
    axs[0].legend()
    axs[0].grid()

    axs[1].set_xlabel("Update time interval [s]")
    axs[1].set_ylabel("Frequency")
    axs[1].legend()
    axs[1].grid()

    pdf_file_name = os.path.join(out_dir, "temporal_performance.pdf")
    report_file_name = os.path.join(out_dir, "temporal_performance.txt")
    # Check if the file already exists and print a message to the user if they want to overwrite
    # create a new file or do nothing
    while(os.path.exists(pdf_file_name)):
        print("{} already exists!".format(pdf_file_name))
        print("Select from one of the options below")
        print("1 -> Create another")
        print("2 -> Overwrite")

        choice = int(raw_input("Enter your choice!: "))
        if choice == 1:
            suffix = raw_input("Enter the new file name (without extension): ")
            pdf_file_name = os.path.join(out_dir, "{}.pdf".format(suffix))
            report_file_name = os.path.join(out_dor, "{}.txt".format(suffix))
        elif choice == 2:
            print("Proceeding to overwrite!")
            final_choice = raw_input("Are you sure (y/n)?")
            if (final_choice == "y"):
                break
            # For any other option it will simply display the entire message again!
        else:
            print("Unidentified choice!")

    pylab.savefig(pdf_file_name, format="pdf", bbox_inches="tight", pad_inches=.06)

    report_data = "rf2o_mean: {} rf2o_sigma: {} \nsao_mean: {}, sao_sigma: {}".format(rf2o_mean, rf2o_sigma, sao_mean, sao_sigma)
    with open(report_file_name, "w") as f:
        f.write(report_data)

    show()


def main():
    rf2o_topic, sao_topic, out_dir = parseArgs()

    # Check if the directory exists,
    # If not create the directory
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)

    # initiate the ros node
    rospy.init_node("evaluate_temporal_performace_node")

    rospy.Subscriber(rf2o_topic, Odometry, rf2oCb)

    rospy.Subscriber(sao_topic, Odometry, saoCb)

    rospy.spin()

    plotTemporalPerformance(out_dir)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
