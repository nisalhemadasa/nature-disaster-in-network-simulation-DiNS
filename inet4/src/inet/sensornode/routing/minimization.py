import numpy as np
from scipy.optimize import minimize
import pandas as pd

def readObjectiveValue():
    objective = pd.read_csv("/home/virtualforest/omnetpp-workspace/inet4/examples/sensornetwork/recordings/gateway0/")
    #
    #
    return -objective #the negative value of the output from the omnet++ simulation

def constraint1(x):
    relationship1 = x[3] - x[2]
    relationship2 = x[2] - x[1]
    relationship3 = x[1] - x[0]
    return relationship1,relationship2,relationship3

bounds = [-14, 10]
constraint1 = {'type':'ineq','fun':constraint1}
solution = []

for i in range(1,11):
    classRanges = np.random.rand(i)
    x0 = np.asarray(classRanges)
    solution[i] = minimize(readObjectiveValue, x0, method='Newton-CG', bounds = bounds, constraints = constraint1)

optimumSolution = min(solution)

#-------------------------------------------------------

def generateClassLevels():
    classRanges = np.random.rand(i)
    return np.asarray(classRanges)





