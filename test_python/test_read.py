# -*- coding: utf-8 -*-


from open3d import *
import numpy as np
import copy


if __name__ == "__main__":
    filename="milk_cartoon_all_small_clorox.pcd"
    
    source = read_point_cloud("../../TestData/ColoredICP/frag_115.ply")