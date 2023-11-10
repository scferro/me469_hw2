import numpy as np
from numpy.random import randint

geneCount = 10

cutoff = randint(1, geneCount)
print(cutoff)

string = '0000011111'
print(string[0:cutoff])
print(string[cutoff:len(string)])