import cv2 as cv
import cv2
import cv2.aruco as aruco
import numpy as np
import glob

# Función para detectar y dibujar el marcador ArUco
def detect_aruco(img, mtx, dist, rvecs, tvecs):
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    #(T, thresh) = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)
    
    # Utilizar getPredefinedDictionary en lugar de Dictionary_create
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

    parameters = aruco.DetectorParameters()
    
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
    
    if ids is not None:
        
        aruco.drawDetectedMarkers(img, corners, ids, borderColor=(0, 255, 0))
        #aruco.drawAxis(img, mtx, dist, rvecs, tvecs, 1)
        cv2.imshow("marker.png", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        if(ids[0] == 5):
        
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners[0], 0.05, mtx, dist)
            
            return rvecs, tvecs
        
        else:
            
            return None, None

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((4*6,3), np.float32)
objp[:,:2] = np.mgrid[0:6,0:4].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('/home/tfg/decman_ws/camera_calibrator/im*.jpg')

for fname in images:
    
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (6,4), None)
    # If found, add object points, image points (after refining them)
    
    if ret == True:
        
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Capturar imagen con el marcador ArUco
frame_with_aruco = cv2.imread("/home/tfg/dectmani_ws/src/camera_py/camera_py/im_color.jpg")

# Detectar el marcador ArUco y obtener la pose
rvecs, tvecs = detect_aruco(frame_with_aruco, mtx, dist, rvecs, tvecs)

# Relación entre el marcador ArUco y el robot
aruco_rotation_matrix, _ = cv2.Rodrigues(rvecs)
aruco_translation_vector = tvecs

object_point_from_camera = np.array([0, 0, 0.1])

# Aplicar la relación con el robot
object_point_from_aruco = np.dot(aruco_rotation_matrix, object_point_from_camera)
object_point_from_aruco = object_point_from_aruco + aruco_translation_vector

robot_translation_vector = np.array([0.1, 0.1, 0.2])

object_point_from_robot = object_point_from_aruco + robot_translation_vector

# Aquí puedes utilizar robot_translation_vector para obtener las coordenadas del tercer objeto en el sistema de coordenadas del robot
print("Coordenadas del objeto en el sistema de coordenadas del aruco:", object_point_from_robot)