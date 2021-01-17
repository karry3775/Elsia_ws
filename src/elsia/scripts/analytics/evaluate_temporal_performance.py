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

    if len(current) == 0 :
        print("No data found! Aborting")
        exit(1)

    interval = []
    # get updates from rf2o_odom_updates
    for i in range(1, len(current)):
        interval.append(
            current[i] - current[i-1]
            )

    return sorted(interval)
    

# function to plot the temporal peformance
# i.e how much time it takes to produce odometry 
# updates
def plotTemporalPerformance(out_dir):
    # Set Params for pylab plotter
    params = {
    'axes.labelsize': 12, #8
    'font.size': 12,
    'legend.fontsize': 14,
    'xtick.labelsize': 12,
    'ytick.labelsize': 12,
    'text.usetex': True,
    'figure.figsize': [4.5, 4.5]
    }
    rcParams.update(params)
    grid()

    rf2o_odom_update_interval = getUpdateInterval("rf2o")
    sao_odom_update_interval = getUpdateInterval("sao")

    plot(np.arange(len(rf2o_odom_update_interval)), rf2o_odom_update_interval,
         linewidth=2, color='r')
    plot(np.arange(len(sao_odom_update_interval)), sao_odom_update_interval,
         linewidth=2, color='b')
    leg = legend(["rf2o", "sao"], loc=1)
    frame = leg.get_frame()
    frame.set_facecolor('0.9')
    frame.set_edgecolor('0.9')

    xlabel("Update number")
    ylabel("Update interval [s]")

    file_name = os.path.join(out_dir, "temporal_performance.pdf")
    # Check if the file already exists and print a message to the user if they want to overwrite 
    # create a new file or do nothing
    while( os.path.exists(file_name) ):
        print("{} already exists!".format(file_name))
        print("Select from one of the options below")
        print("1 -> Create another")
        print("2 -> Overwrite")

        choice = int(raw_input("Enter your choice!: "))
        if choice == 1:
            suffix = raw_input("Enter the new file name (without extension): ")
            file_name = os.path.join(out_dir, "{}.pdf".format(suffix))
        elif choice == 2:
            print("Proceeding to overwrite!")
            final_choice = raw_input("Are you sure (y/n)?")
            if (final_choice == "y"):
                break
            # For any other option it will simply display the entire message again!
        else:
            print("Unidentified choice!")

    pylab.savefig(file_name, format="pdf", bbox_inches="tight", pad_inches=.06)
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