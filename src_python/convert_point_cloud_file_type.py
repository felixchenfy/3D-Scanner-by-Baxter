'''
Run this script to change point cloud file types. Examples:

$ python src_python/convert_point_cloud_file_type.py data/ColoredICP_1.ply data/ColoredICP_1.pcd
$ python src_python/convert_point_cloud_file_type.py data/ColoredICP_2.ply data/ColoredICP_2.pcd

'''

import numpy as np
from open3d import *
import sys

if __name__ == "__main__":
    try:
        filename_from = sys.argv[1]
        filename_to = sys.argv[2]
    except:
        print("Please input two arguments, which are the input and output filenames.")
        sys.exit()

    # convert data type by reading and writing
    pcd = read_point_cloud(filename_from)
    write_point_cloud(filename_to, pcd)

    # print
    print("Convert file success:")
    print("  from %s"%filename_from)
    print("  to %s"%filename_to)

