
'''
This script provides:
    getChessboardPose: get T_cam_to_chessboard
    and a serial functions for drawing the chessboard and its coordinate onto the image
'''

import numpy as np
import copy
import cv2

from lib_geo_trans import *

# ---------------------------------- Main functions


# Generate a unique chessboard frame, by:
#   1. Let chessboard z axis pointing to the camera
#   2. I manually draw a black dot near the center of board.
#       I will rotate the frame so that this dot's pos is (d_row=0.5, d_col=-0.5)
def helperMakeUniqueChessboardFrame(img, R, p):
    
    # -- 1. Make chessboard z axis pointing towards the camera
    vec_cam_to_chess = np.array(p).squeeze(axis=1)
    vec_chess_z = form_T(R, p).dot(np.array([[0],[0],[1],[0]])).squeeze(axis=1)[0:3]-np.array(p)
    if vec_cam_to_chess.dot(vec_chess_z)>0:
        R=R*rotx(np.pi)

    # -- 2. A manually drawn dot at required pos.
    return R,p

def getChessboardPose(img,
        camera_intrinsics,
        distortion_coeffs,
        SQUARE_SIZE = 0.0158,
        CHECKER_ROWS = 7,
        CHECKER_COLS = 9,
    ):
    # -- Check input
    if camera_intrinsics.shape[0]==9:
        camera_intrinsics=camera_intrinsics.reshape((3,3))
    
    # -- Get pos of object points (the chessboard corners) in chessboard frame
    # -- num in x axis: CHECKER_ROWS
    # -- num in y axis: CHECKER_COLS
    def create_object_points(square_size):
        objpoints = np.zeros((CHECKER_COLS*CHECKER_ROWS, 3), np.float32)
        objpoints[:, :2] = np.mgrid[0:CHECKER_ROWS, 0:CHECKER_COLS].T.reshape(-1, 2)
        objpoints[:, 0] -= (CHECKER_ROWS-1)/2.0
        objpoints[:, 1] -= (CHECKER_COLS-1)/2.0
        objpoints*=square_size
        return objpoints
    objpoints=create_object_points(SQUARE_SIZE)

    # -- Find the chess board corners in gray image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    flag_find_chessboard, corners = cv2.findChessboardCorners(
        gray, (CHECKER_ROWS, CHECKER_COLS), None) 

    # -- If found, estimate T_cam_to_chessboard 
    R = p = None
    if flag_find_chessboard == False:
        # print("chessboard not found")
        return False, None, None, None

    if flag_find_chessboard == True:
        # print("chessboard found")

        # Refine corners' pos in image
        # https://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html?highlight=cornersubpix
        def refineImageCorners(gray_image, corners):
            CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER + cv2.CALIB_CB_FAST_CHECK,
                100, 0.0005)
            grid_resolution = 7
            SIZE_OF_SEARCH_WINDOW=(grid_resolution, grid_resolution)
            HALF_SIZE_OF_ZERO_ZONE=(-1, -1)
            refined_corners = cv2.cornerSubPix(
                gray_image, corners, SIZE_OF_SEARCH_WINDOW, HALF_SIZE_OF_ZERO_ZONE, CRITERIA)
            return refined_corners
        corners = refineImageCorners(gray, corners)

        # solve PnP: transformation of chessboard wrt CAMERA
        objpoints = objpoints
        imgpoints = corners
        if 1: # If you are sure that corners are all correct, use this one
            err_value, R_vec, p = cv2.solvePnP(
                objpoints, imgpoints, camera_intrinsics, distortion_coeffs)
        else: 
            err_value, R_vec, p, inliers = cv2.solvePnPRansac(
                objpoints, imgpoints, camera_intrinsics, distortion_coeffs)
        R, _ = cv2.Rodrigues(R_vec)
        
        # Generate a unique chessboard frame, by:
        #   1. Let chessboard z axis pointing to the camera
        #   2. I manually draw a black dot near the center of board.
        #       I will rotate the frame so that this dot's pos is (d_row=0.5, d_col=-0.5)
        R, p = helperMakeUniqueChessboardFrame(R, p)

        return flag_find_chessboard, R, p, imgpoints


# ------------------------------------ Drawing Functions ------------------------------------

def drawChessboardToImage(img_display, imgpoints, CHECKER_ROWS, CHECKER_COLS):
    flag_find_chessboard = True
    img_display = cv2.drawChessboardCorners(
        img_display, (CHECKER_ROWS, CHECKER_COLS), imgpoints, flag_find_chessboard
    )

def drawPosTextToImage(img_display, p):
    font = cv2.FONT_HERSHEY_SIMPLEX
    sss = ["chessboard pose wrt to camera","x=", "y=", "z="]
    for i in range(-1,3):
        if i!=-1:
            s = "{:.2f}".format(p[i, 0])
        else:
            s=""
        TEST_ROWS = 50+i*30
        TEST_COLS = 50
        img_display = cv2.putText(
            img_display, sss[i+1]+s, (TEST_COLS, TEST_ROWS), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

def drawLineToImage(img_display, p0, p1, color='r', line_width=2):
    cv2.line(img_display,p0,p1,color,line_width)

def drawCoordinateToImage(img_display, R_cam_to_coord, p_cam_to_coord, camera_intrinsics, distortion_coeffs):
    T_cam_to_coord=form_T(R_cam_to_coord, p_cam_to_coord)
    length = 0.2 # m
    
    # -- Compute xyz-axis of the chessboard coordinate
    pts_coord=[[0,0,0],[1,0,0],[0,1,0],[0,0,1]]
    pts_coord = (np.array(pts_coord)*length).tolist()
    
    # -- Project them onto image
    pts_pixel = list()
    for pt_coord in pts_coord:
        p_image = world2pixel(pt_coord, T_cam_to_coord, camera_intrinsics, distortion_coeffs)
        u,v=[int(p_image[0]), int(p_image[1])] 
        pts_pixel.append((u,v))
 
    # -- Draw line
    colors=['r','g','b']
    colors_dict={'b':[255,0,0],'g':[0,255,0],'r':[0,0,255]}
    for i in range(3):
        p0 = pts_pixel[0]
        pi = pts_pixel[1+i]
        drawLineToImage(img_display, p0, pi, colors_dict[colors[i]], line_width=2)

    
def showImg(I):
    cv2.imshow('img', I)
    cv2.waitKey()
    cv2.destroyAllWindows()

# ------------------------------------ Test ------------------------------------
if __name__=="__main__":
    I = cv2.imread("../data_debug/img1.png")
    
    SQUARE_SIZE = 0.0158
    CHECKER_ROWS = 7
    CHECKER_COLS = 9
    camera_intrinsics = np.array([607.633212, 0.000000, 326.894378, 0.000000, 606.810293, 227.105055, 0.000000, 0.000000, 1.000000])
    distortion_coeffs = np.array([-0.166673, 0.380839, -0.004246, 0.002507, 0.000000])
    camera_intrinsics = camera_intrinsics.reshape((3,3))
    
    # Solve T_cam_to_chessboard
    res, R, p, imgpoints = getChessboardPose(I,
      camera_intrinsics, distortion_coeffs, SQUARE_SIZE, CHECKER_ROWS, CHECKER_COLS)
    if res==False:
        "\n\nmy ERROR: NO CHESSBOARD\n\n"

    print "\nR_cam_to_chessboard:\n", R
    print "\np_cam_to_chessboard:\n", p
    
    # Draw chessboard
    img_display=I.copy()
    drawChessboardToImage(img_display, imgpoints, CHECKER_ROWS, CHECKER_COLS)
    drawPosTextToImage(img_display, p)
    
    # Draw coordinate    
    drawCoordinateToImage(img_display, R, p, camera_intrinsics, distortion_coeffs)
    showImg(img_display)