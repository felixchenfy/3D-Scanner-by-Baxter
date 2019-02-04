
---------------------------------------
02-02 ANALYSIS

Besides object itself, the algorithm needs additional info for better registration.
However, if this addition info are not the same in different views, such as the different floor pixels under different camera view, it will on the contrary ruin the registration by its useless extra cloud points.

Thus, I'd better use other objects instead of colored paper to assist registration.

TODO: Check if there is a displacement between depth_cam's color and depth frame, by placing the chessboard on the floor and see if after tranformation its in the center of the frame. Also watch the depth_cam's topics and tfs.
(The bug arises from the fact that I'm using colored image and colored image frame during calibration, but the point cloud might actually be in the depth frame.)

TODO: Draw colored English words larger. Or, maybe I should use other obstacles to replace drawing. 


---------------------------------------
02-02 PROBLEMS

` Calibration error:
After rotate point clouds to the Baxter frames, I put 11 drillers clouds into one view, and then saw that the error is about: 5 cm in x and y direction, and 30 degrees. Besides, the z value is not 0, but 5 cm. There must be something wrong with the calibration.

After I manually tranlate the camera from by x,y,z　= +-0.1m (add one line to the fake node1), I found that z=0.1 will make the driller center at the coord frame.
So there should be a z=0.1 displacement apply to the current camera frame. Test it tommorrow.

` Node 2, pcl has errr for the 9th clouds, where the driller disappears.

` The Enlish words drawn on the paper is too thin. Even if I use 0.001 downsampling rate, it still cannot been clearly distinguished. It requires like 0.0002 to be very clear.

` With small downsampling voxel size, the points number is kept increasing.
20k for one cloud, then there are 200k for 10 clouds, due to the small registration error. This is ridiculous.


---------------------------------------
02-02 TESTINT RESULT
` DATASET:  driller with colored board
ALGORITHM: GLOBAL REGISTER, GOOD. PARAM: voxel_size=0.01~0.02
ALGORITHM: ICP, GOOD. PARAM: voxel_size(for ICP)=0.005, global_regi_ratio=2, colored_ICP_ratio={1,1/2,1/4} 
ALGORITHM: ICP + Colored-ICP, GOOD

` DATASET:driller without colored board
All algorithms are bad, including GLOBAL REGI and ICP.

