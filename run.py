import numpy as np
from algorithm import Algorithm


# Declare tuning parameters
popSize = 100 

orderM1Vel = 3
orderM2Vel = 3
orderM3Vel = 3
orderM4Vel = 3
orderM1Omega = 3
orderM2Omega = 3
orderM3Omega = 3
orderM4Omega = 3

popSize = 50
errorGoal = 0.05
maxIterations = 200
mOrderList = [1,1,1,1,1,1,1,1]
bBits = 3
cBits = 3
mutPopPct = 0.25
mutGenePct = 0.25

trainingDatasetFilename = 'learning_dataset.csv'

geneticAlgo = Algorithm(popSize, errorGoal, maxIterations, mOrderList, bBits, cBits, mutPopPct, mutGenePct)

dataset = geneticAlgo.import_dataset(trainingDatasetFilename)

bestGenome, distanceError = geneticAlgo.execute()