# Librerías para crear el nodo de ROS2
import rclpy
from rclpy.node import Node

# Librerías de mensajes de ROS2
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from msg_srv_creator.msg import Mask

# Librerías para trabajar con matrices e imágenes
import cv2
import numpy as np
from cv_bridge import CvBridge

# Librerías Detectron2
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg

# Definición de la clase del nodo de ROS2
class Detectron2(Node):

    def __init__(self):

        super().__init__('detectron2')

        # Puente para convertir entre imágenes de ROS y OpenCV
        self.cvbridge_ = CvBridge()

        # Variables para almacenar imágenes
        self.color_image_ = Image()
        self.depth_image_ = Image()

        # Contador para el número de objetos detectados
        self.count_ = int()

        ## Configuración de Detectron2
        self.cfg_ = get_cfg()
        self.cfg_.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        self.cfg_.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5
        self.cfg_.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")
        self.predictor_ = DefaultPredictor(self.cfg_)

        # Publishers
        self.publisher_masks_image_ = self.create_publisher(Mask, '/detectron2/masks_image', 10)
        self.publisher_depth_image_ = self.create_publisher(Image, '/detectron2/depth_image', 10)

        # Subscribers
        self.subscription_color_image_ = self.create_subscription(Image, '/camera/color/image_raw', self.listener_callback_color_image, 10)
        self.subscription_color_image_
        self.subscription_depth_image_ = self.create_subscription(Image, '/camera/depth/image_raw', self.listener_callback_depth_image, 10)
        self.subscription_depth_image_

    # Callback para la imagen en color
    def listener_callback_color_image(self, msg):

        self.color_image_ = msg
        
        # Obtiene máscaras utilizando Detectron2
        mask_images = self.masks_obtainer(self.color_image_)
        
        # Obtiene el número de objetos detectados
        self.count_ = len(mask_images)

        if mask_images:
            
            # Publica las máscaras y la imagen de profundidad
            masks_msg = Mask()
            masks_msg.mask_images = mask_images
            self.publisher_masks_image_.publish(masks_msg)
            self.publisher_depth_image_.publish(self.depth_image_)
            self.get_logger().info('%i Objetos detectados' % self.count_)           
            
        else:
            
            self.get_logger().info('Ningún objeto detectado')

    # Callback para la imagen de profundidad
    def listener_callback_depth_image(self, msg):
        
        self.depth_image_ = msg         

    # Función para obtener máscaras utilizando Detectron2
    def masks_obtainer(self, mask_msg):

        cv_image = self.cvbridge_.imgmsg_to_cv2(mask_msg, 'rgb8')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        
        # Obtiene las predicciones de Detectron2
        outputs = self.predictor_(cv_image)

        # Inicializa un vector para almacenar la posición en la que se encuentran los objetos deseados
        vector = []

        # Almacena la posición de los objetos que son de la clase 41 (vasos o tazas)
        for iter in range(len(outputs['instances'])):
            
            if outputs['instances'].pred_classes[iter].item() == 41:
                
                vector.append(iter)

        # Crea el tensor de los objetos deseados mediante el vector
        masks = outputs['instances'].pred_masks[vector]

        # Obtiene la parte del array del tensor
        masks = masks.detach().to('cpu').numpy()

        # Inicializa una lista para guardar las máscaras de las imágenes en forma binarizada
        mask_images = []

        for k in range(len(masks)):  
            
            x, y = masks[k].shape  
            mask_image = np.zeros((x, y), dtype=np.uint8) 

            for i in range(x):
                
                for j in range(y):
                    
                    if masks[k][i][j]:
                        
                        mask_image[i][j] = 255

            mask_images.append(self.cvbridge_.cv2_to_imgmsg(mask_image, 'mono8'))

        return mask_images

# Función principal para ejecutar el nodo
def main(args=None):
    rclpy.init(args=args)

    detectron2 = Detectron2()

    rclpy.spin(detectron2)

    detectron2.destroy_node()
    rclpy.shutdown()

# Ejecuta la función principal si el script es el programa principal
if __name__ == '__main__':
    main()