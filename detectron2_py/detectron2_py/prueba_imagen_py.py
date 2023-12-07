import cv2
import numpy as np

im=cv2.imread('/home/tfg/dectmani_ws/src/detectron2_py/detectron2_py/im0.jpg')
cv2.imshow('imagen', im)
cv2.waitKey(0)
cv2.destroyAllWindows()