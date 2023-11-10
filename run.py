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

bBits = 5
cBits = 5

trainingDatasetFilename = ''

geneticAlgo = Algorithm()

bestGenome, distanceError = geneticAlgo.execute()