
3 MAIN nodes for the project is stored here, indexed as n1~n3.

Camera calibration and some other assistive scripts are also stored here.

"n1_fake_data_publisher.py" is a replacement for "n1_move_baxter.py" and is used for debug. It reads the Baxter poses and point cloud from file, and publishes them to node 2. Thus, I can test previously saved data, and debug without connecting to any hardware.