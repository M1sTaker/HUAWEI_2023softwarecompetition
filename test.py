import math
import numpy as np

a = [1, 2,3,4]
for i in range(len(a)):
    for j in range(i+1, len(a)):
        print(i, j)