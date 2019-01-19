
import numpy as np


def list2str(vals):
    str_val = ["{:.3f} ".format(val) for val in vals]
    str_val = ", ".join(str_val)
    return str_val