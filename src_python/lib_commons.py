# -*- coding: utf-8 -*-

import numpy as np

def list2str(vals):
    str_val = ["{:.3f} ".format(val) for val in vals]
    str_val = ", ".join(str_val)
    return str_val
    
def savetxt(filename, data):
    np.savetxt(filename, data, delimiter=",")

def loadtxt(filename):
    return np.loadtxt(filename, delimiter=",")