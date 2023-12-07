# Librerías para crear el nodo de ROS2
import rclpy
from rclpy.node import Node

# Librerías de mensajes de ROS2
from sensor_msgs.msg import Image

# Librerías para trabajar con matrices e imágenes
import cv2
from cv_bridge import CvBridge

# Definición de la clase del nodo de ROS2
class Camera(Node):

    def __init__(self):

        super().__init__('camera')

        # Puente para convertir entre imágenes de ROS y OpenCV
        self.cvbridge_ = CvBridge()

        # Subscribers
        self.subscription_color_image_ = self.create_subscription(Image, '/camera/color/image_raw', self.listener_callback_color_image, 10)
        self.subscription_color_image_

    # Callback para la imagen en color
    def listener_callback_color_image(self, msg):

        im_color = self.cvbridge_.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imwrite('/home/tfg/dectmani_ws/src/camera_py/camera_py/im_color.jpg', im_color)         

# Función principal para ejecutar el nodo
def main(args=None):
    
    rclpy.init(args=args)

    camera = Camera()

    rclpy.spin(camera)

    camera.destroy_node()
    rclpy.shutdown()

# Ejecuta la función principal si el script es el programa principal
if __name__ == '__main__':
    
    main()