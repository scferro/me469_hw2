import numpy as np
import time

class Algorithm:
    def __init__(self, popSize, initPopSize, errorGoal, maxIterations, mOrderList, bBits, cBits, mutPopPct, mutGenePct, pctClone):
        self.population = []
        self.popFitness = []
        self.popSize = popSize
        self.initPopSize = initPopSize
        self.errorGoal = errorGoal
        self.maxIterations = maxIterations
        self.mOrderList = mOrderList
        self.bBits = bBits
        self.cBits = cBits
        self.geneCount = (sum(self.mOrderList) + len(self.mOrderList)) * (self.bBits + self.cBits)
        self.mutPopPct = mutPopPct
        self.mutGenePct = mutGenePct
        self.dataset = []
        self.pctClone = pctClone

    def execute(self):
        # main function to execute algorithm
        counter = 0
        self.create_initial_pop()
        minError, minErrorIndex = self.evaluate_pop_fitness()
        while (minError > self.errorGoal) and (counter < self.maxIterations):
            print(self.popFitness)
            print(self.population)
            print('Iteration ' + str(counter))
            print('Min Error: ' + str(minError))
            print('Min Error Index: ' + str(minErrorIndex))
            print('Best Genome: ' + str(self.population[minErrorIndex]))
            self.baby_makin(self.pctClone)
            minError, minErrorIndex = self.evaluate_pop_fitness()
            counter += 1
        bestGenome = self.population[minErrorIndex]
        return bestGenome, minError

    def create_initial_pop(self):
        # generate an initial random population
        counterPop = 0
        newPop = []
        while counterPop < self.initPopSize:
            counterGenes = 0
            genome = ''
            while counterGenes < self.geneCount:
                bit = np.random.randint(0,2)
                bitString = str(bit)
                genome += bitString
                counterGenes += 1
            newPop.append(genome)
            counterPop += 1
        self.population = newPop
        return newPop

    def evaluate_pop_fitness(self):
        # evaluate the fitness of the current population
        fitnessList = []
        for genome in self.population:
            avgError = self.run_commands(genome)
            if avgError == float('inf'):
                avgError = 10**100
            fitnessList.append(avgError)
        self.popFitness = fitnessList
        minError = min(fitnessList)
        minErrorIndex = fitnessList.index(minError)
        return minError, minErrorIndex

    def calculate_mod_factors(self, bList, cList, xPos, yPos, vel, omega, dVel, dOmega):
        # calculate the modification factors to be used for the current state
        mList = []
        aList = []
        varList = [xPos, yPos, vel, omega, dVel, dOmega, xPos, yPos, vel, omega, dVel, dOmega]
        counter1 = 0
        while counter1 < len(bList):
            aList.append(10**bList[counter1] * cList[counter1])
            counter1 += 1
        counter2 = 0
        for order in self.mOrderList:
            counter3 = 0
            m = 0
            while counter3 <= order:
                m += aList[0] * varList[counter2]**(order-counter3)
                aList.pop(0)
                counter3 += 1
            mList.append(m)
            counter2 += 1
        return mList

    def decode_genome(self, genome):
        # convert genome from binary to decimal
        counterMain = 0
        counterInner = 0
        countMainMax = len(genome)
        newString = ''
        newValue = 0
        bList = []
        cList = []
        while counterMain < countMainMax:
            bit = genome[counterMain]
            newString += bit
            if counterInner == (self.bBits - 1):
                newValue = int(newString, 2)
                bList.append(newValue)
                newString = ''
                newValue = 0
            elif counterInner == (self.cBits + self.bBits - 1):
                newValue = int(newString, 2)
                cList.append(newValue)
                newString = ''
                newValue = 0
                counterInner = -1
            counterMain += 1
            counterInner += 1
        bListScaled = []
        cListScaled = []
        for b in bList:
            b = b - 2**(self.bBits-1)
            bListScaled.append(b)
        for c in cList:
            c = (c - 2**(self.cBits-1) + 0.5)/(2**(self.cBits-1) - 0.5)
            cListScaled.append(c)
        return bListScaled, cListScaled

    def run_commands(self, genome):
        # Function to move robot according to imported odometry data
        # Iterate through the odometry commands
        resetGap = 60
        xPos = self.dataset[0][4]
        yPos = self.dataset[0][5]
        theta = self.dataset[0][6]
        timeInit = self.dataset[0][0]
        timePrev = timeInit  
        bList, cList = self.decode_genome(genome)
        prevPosGT = [xPos, yPos]
        errorList = []
        timeReset = resetGap
        prevCommand = self.dataset[0]
        for command in self.dataset:
            time = command[0] - timeInit
            timeStep = time - timePrev
            vel = command[1]
            omega = command[2]
            dVel = command[1] - prevCommand[1]
            dOmega = command[2] - prevCommand[2]
            mList = self.calculate_mod_factors(bList, cList, xPos, yPos, vel, omega, dVel, dOmega)
            xPos, yPos, theta = self.motion_model(vel, omega, timeStep, [xPos, yPos, theta], mList)
            if prevPosGT != [command[4], command[5]]:
                distError = ((command[4] - xPos)**2 + (command[5] - yPos)**2)**0.5
                errorList.append(distError)
            prevPosGT = [command[4], command[5]]
            if time > timeReset:
                xPos = command[4]
                yPos = command[5]
                theta = command[6]
                timeReset = time + resetGap
            timePrev = time
            prevCommand = command
        avgError = np.nansum(errorList) / len(errorList)
        return avgError

    def baby_makin(self, pctClone):
        # create new generation
        newPop = []
        counter1 = 0
        population = self.population
        fitness = self.popFitness
        while counter1 < self.popSize * pctClone:
            minFitness = min(fitness)
            minFitnessIndex = fitness.index(minFitness)
            bestGenome = population[minFitnessIndex]
            newPop.append(bestGenome)
            population.pop(minFitnessIndex)
            fitness.pop(minFitnessIndex)
            counter1 += 1
        counter2 = 0
        while counter2 < self.popSize * (1 - pctClone) + 1:
            parent1, parent2 = self.select_parents()
            splitPoint = np.random.randint(1, self.geneCount)
            newGenome = parent1[0:splitPoint] + parent2[splitPoint:len(parent2)]
            newPop.append(newGenome)
            counter2 += 1
        self.population = newPop
        self.mutate_pop()
        return newPop

    def select_parents(self):
        # select parents for next generation based on weighting 
        selectionPop = self.population
        selectionFitness = self.popFitness
        totalFitness = 0
        for fitness in selectionFitness:
            totalFitness += (max(selectionFitness) - fitness)
        randomSelect1 = totalFitness * np.random.rand()
        counter1 = -1
        while randomSelect1 > 0:
            counter1 += 1
            randomSelect1 += -(max(selectionFitness) - selectionFitness[counter1])
        parent1 = selectionPop[counter1]
        randomSelect2 = totalFitness * np.random.rand()
        counter2 = -1
        while randomSelect2 > 0:
            counter2 += 1
            randomSelect2 += -(max(selectionFitness) - selectionFitness[counter2])
        parent2 = selectionPop[counter2]
        return parent1, parent2

    def mutate_pop(self):
        # mutate a portion of the population
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
                
    def apply_mod_factors(self, vel, omega, mList):
        # apply the modification factors to the current state
        vel += vel * ((mList[0] * mList[1]) + mList[2] + mList[3] + mList[4] + mList[5])
        omega += omega * ((mList[6] * mList[7] + mList[8] + mList[9] + mList[10] + mList[11]))
        return vel, omega
    
    def motion_model(self, vel, omega, timeStep, pos, mList):
        # this fucntion calculates the new position and orientation of the robot based on the previous location and oriantationa and the command sent to the robot
        vel, omega = self.apply_mod_factors(vel, omega, mList)
        x0 = pos[0]
        y0 = pos[1]
        theta = pos[2]
        xNew = 0
        yNew = 0
        thetaNew = 0
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
            return xNew, yNew, thetaNew
        

    def import_dataset(self, filename): 
        # Imports data from specified robot groundtruth file. File should be in same directory as the python files
        print('Importing dataset from file "' + filename + '"...')
        data = np.genfromtxt(fname=filename, delimiter=',', skip_header=1)
        countMax = len(data)
        count = 0
        newData = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        while count + 1 < countMax:
            # Iterate through the data and calculate time steps based on the provided time data
            newPoint = np.array([data[count][0], data[count][1], data[count][2], data[count][3], data[count][4], data[count][5], data[count][6]])
            newData = np.vstack((newData, newPoint))
            count += 1
        print("Training dataset import complete!")
        newData = np.delete(newData, 0, 0)
        self.dataset = newData
        return newData


