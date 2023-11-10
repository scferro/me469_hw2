import numpy as np

class Algorithm:
    def __init__(self, popSize, errorGoal, maxIterations, trainingDataset, mOrderList, bBits, cBits, mutPopPct, mutGenePct):
        self.population = []
        self.popFitness = []
        self.popSize = popSize
        self.errorGoal = errorGoal
        self.maxIterations = maxIterations
        self.dataset = trainingDataset
        self.mOrderList = mOrderList
        self.bBits = bBits
        self.cBits = cBits
        self.geneCount = sum(self.mOrderList) * (self.bBits + self.cBits)
        self.mutPopPct = mutPopPct
        self.mutGenePct = mutGenePct

    def execute(self):
        counter = 0
        self.create_initial_pop()
        minError, minErrorIndex = self.evaluate_pop_fitness()
        while (minError > self.errorGoal) and (counter < self.maxIterations):
            self.baby_makin()
            self.mutate_pop()
            minError, minErrorIndex = self.evaluate_pop_fitness()
            counter += 1
        bestGenome = self.population[minErrorIndex]
        return bestGenome, minError

    def create_initial_pop(self):
        counterPop = 0
        newPop = []
        while counter < self.popSize:
            counterGenes = 0
            genome = ''
            while counterGenes < self.geneCount:
                bit = np.randint(0,2)
                bitString = str(bit)
                genome += bitString
                counterGenes += 1
            newPop.append(genome)
            counterPop += 1
        self.population = newPop
        return newPop

    def evaluate_pop_fitness(self):
        finessList = []
        for genome in self.population:
            mList = self.calculate_mod_factors(genome)
            avgError = run_commands()
            fitnessList.append(avgError)
        self.popFitness = fitnessList
        return fitnessList

    def calculate_mod_factors(self, genome):
        mList = []
        counter = 0
        while counter < 8:


    def run_commands(self):
        pass

    def baby_makin(self):
        newPop = []
        counter = 0
        while counter < len(self.population):
            parent1, parent2 = self.select_parents()
            splitPoint = np.randint(1, self.geneCount)
            newGenome = parent1[0:splitPoint] + parent2[splitPoint:len(parent2)]
            newPop.append(newGenome)
        self.population = newPop
        return newPop

    def select_parents(self):
        selectionPop = self.population
        selectionFitness = self.popFitness
        totalFitness = sum(selectionFitness)
        randomSelect1 = totalFitness * np.random.rand()
        counter1 = -1
        while randomSelect1 >= 0:
            counter1 += 1
            randomSelect1 += -selectionFitness[counter1]
        parent1 = selectionPop[counter1]
        selectionPop.pop(counter1)
        selectionFitness.pop(counter1)
        randomSelect2 = totalFitness * np.random.rand()
        counter2 = -1
        while randomSelect2 >= 0:
            counter2 += 1
            randomSelect2 += -selectionFitness[counter2]
        parent2 = selectionPop[counter2]
        return parent1, parent2

    def mutate_pop(self):
        countPop = 0
        maxCountPop = int(self.mutPopPct * len(self.population))
        newPop = []
        for genome in self.population:
            randPop = np.random.rand()
            newGenome = ''
            if randPop < self.mutPopPct:
                for bit in genome:
                    randGene = np.random.rand()
                    newBit = bit
                    if randGene < self.mutGenePct:
                        if bit == '0':
                            newBit = '1'
                        elif bit == '1':
                            newBit = '0'
                    newGenome += newBit
            else:
                newGenome = genome
            newPop.append(newGenome)
        self.population = newPop
        return newPop
                
    def apply_mod_factors(self, vel, omega, xPos, yPos, mList):
        vel += vel * ((mList[0] * mList[1]) + mList[2] + mList[3])
        omega += omega * ((mList[4] * mList[5]) + mList[6] + mList[7])
        return vel, omega
    
    def motion_model(self, vel, omega, timeStep, pos, mList):
        # this fucntion calculates the new position and orientation of the robot based on the previous location and oriantationa and the command sent to the robot
        vel, omega = apply_mod_factors(vel, omega, pos[0], pos[1], mList)
        try:
            # Take the most recent calculated position for the "start" coordinates x0, y0, and intial theta
            x0 = pos[-1, 0]
            y0 = pos[-1, 1]
            theta = pos[-1, 2]
        except IndexError as e:
            # If no new positions have been added yet, the intial X, Y, and theta will be used
            x0 = pos[0]
            y0 = pos[1]
            theta = pos[2]
        if omega == 0.0:
            # This is used in cases where angular velocity = 0, calculates new position for a linear move based on current heading, linear velocity, and time step
            deltaX = np.cos(theta) * vel * timeStep
            deltaY = np.sin(theta) * vel * timeStep
            xNew = x0 + deltaX
            yNew = y0 + deltaY
            thetaNew = theta
            return np.array([xNew, yNew, thetaNew])
        else:
            # This is used in cases where angular velocity =/= 0 (turning), calculates new position for a linear move based on current heading, linear and angularvelocity, and time step
            radius = abs(vel/omega) # radius of rotation based on linear and angular speed
            angRot = omega * timeStep # angle change during maneuver based on angular speed
            angRotMod = angRot - (np.pi/2 - theta) # convert angle change to global coordinate system
            if omega > 0: 
                # Calculates the center of rotation for a LH turn
                curveCenterX = x0 + (np.cos(theta + np.pi/2) * radius)
                curveCenterY = y0 + (np.sin(theta + np.pi/2) * radius)
                thetaNew = theta + angRot
                # Calculate a new position based off the center of rotation, the radius of rotation, and the angle of rotation
                xNew = curveCenterX + (np.cos(angRotMod) * radius) 
                yNew = curveCenterY + (np.sin(angRotMod) * radius)
            elif omega < 0: 
                # Calculates the center of rotation for a RH turn
                curveCenterX = x0 + (np.cos(theta - np.pi/2) * radius)
                curveCenterY = y0 + (np.sin(theta - np.pi/2) * radius)
                thetaNew = theta + angRot
                # Calculate a new position based off the center of rotation, the radius of rotation, and the angle of rotation
                xNew = curveCenterX + (np.cos(angRotMod) * -radius) 
                yNew = curveCenterY + (np.sin(angRotMod) * -radius)
            return np.array([xNew, yNew, thetaNew])


