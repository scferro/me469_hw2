import numpy as np
from algorithm import Algorithm


# Declare tuning parameters
initPopSize = 300
popSize = 60
errorGoal = 0.05
maxIterations = 500
mOrderList = [2,2,1,1,1,1,  # Modification factors for Vel:     x, y, vel, omega, dVel, dOmega
              2,2,1,1,1,1]  # Modification factors for Omega:   x, y, vel, omega, dVel, dOmega
bBits = 3           # number of bits for b variables
cBits = 3           # number of bits for c variables
mutPopPct = 0.1     # percentage of population to experience mutation
mutGenePct = 0.1    # percentage of genes to be mutated
pctClone = 0.5      # percentage of a generation to be cloned

trainingDatasetFilename = 'learning_dataset.csv'

# Initialize algorithm object
geneticAlgo = Algorithm(popSize, initPopSize, errorGoal, maxIterations, mOrderList, bBits, cBits, mutPopPct, mutGenePct, pctClone)

# Import dataset
dataset = geneticAlgo.import_dataset(trainingDatasetFilename)

# Execute algorithm
bestGenome, distanceError = geneticAlgo.execute()