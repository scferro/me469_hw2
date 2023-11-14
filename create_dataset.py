import numpy as np
import csv

def import_odometry(filename): 
    # Imports data from specified odometry file. File should be in same directory as the python files
    print('Importing odometry data from file "' + filename + '"...')
    data = np.genfromtxt(filename, skip_header=4)
    countMax = len(data)
    count = 0
    newData = np.array([0.0, 0.0, 0.0])
    while count + 1 < countMax:
        # Iterate through the data and calculate time steps based on the provided time data
        t0 = data[count][0]
        t1 = data[count + 1][0]
        tStep = t1 - t0
        newPoint = np.array([data[count][0], data[count][1], data[count][2]])
        newData = np.vstack((newData, newPoint))
        count += 1
    print("Odometry data import complete!")
    return newData

def import_groundtruth(filename): 
    # Imports data from specified robot groundtruth file. File should be in same directory as the python files
    print('Importing groundtruth data from file "' + filename + '"...')
    data = np.genfromtxt(filename, skip_header=4)
    countMax = len(data)
    count = 0
    newData = np.array([0.0, 0.0, 0.0, 0.0])
    while count + 1 < countMax:
        # Iterate through the data and calculate time steps based on the provided time data
        t0 = data[count][0]
        t1 = data[count + 1][0]
        tStep = t1 - t0
        newPoint = np.array([data[count][0], data[count][1], data[count][2], data[count][3]])
        newData = np.vstack((newData, newPoint))
        count += 1
    print("Groundtruth data import complete!")
    newData = np.delete(newData, 0, 0)
    return newData

def write_to_csv(array, file_name):
    """
    Decription:
        Writes an array to a csv file
    Inputs:
        • array: The array to be written to the file
        • file_name: The filename to be used
    """
    with open(file_name, 'w', newline='') as file:
        print("Writing to CSV file...")
        writer = csv.writer(file)
        writer.writerows(array)
        print("Write complete!")


odomFile = "ds0_Odometry.dat"
groundTruthFile = "ds0_Groundtruth.dat"

odom = import_odometry(odomFile)
groundTruth = import_groundtruth(groundTruthFile)

odomCount = 0
gtCount = 0

newDataset = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

print("Creating dataset...")

while (odomCount < len(odom)) and (gtCount + 1 < len(groundTruth)):
    try:
        while (odom[odomCount][0] >= groundTruth[gtCount+1][0]) and (gtCount + 1 < len(groundTruth)):
            gtCount += 1
        newLine = np.array([float(odom[odomCount][0]), float(odom[odomCount][1]), float(odom[odomCount][2]), 
                            float(groundTruth[gtCount][0]), float(groundTruth[gtCount][1]), float(groundTruth[gtCount][2]), float(groundTruth[gtCount][3])
                            ])
        newDataset = np.vstack((newDataset, newLine))
        odomCount += 1
    except IndexError as e:
        break

newDataset = np.delete(newDataset, [0,1], axis=0)
newDataset = np.vstack((np.array(['Odom Time (s)', 'Velocity (m/s)', 'Omega (rad/s)', 'GT Time (s)', 'GT X (m)', 'GT Y (m)', 'GT Theta (rad)']), newDataset))

print("Dataset creation complete!")

write_to_csv(newDataset, 'dataset.csv')
