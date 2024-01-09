# Librerías para crear el nodo de ROS2
import rclpy
from rclpy.node import Node

# Librerías de mensajes de ROS2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from msg_srv_creator.msg import Mask
from msg_srv_creator.msg import Bbox3d

# Librerías para trabajar con matrices e imágenes
import numpy as np
import cv2
from cv_bridge import CvBridge

class Bbox(Node):

    def __init__(self):

        super().__init__('bbox')

        ## Valores extrínsecos que relacionan ambas cámaras
        # Traslación
        self.M=np.array([[1, 0, -25.463199615478516], [0, 1, -0.20235499739646912]])
        # Rotación
        self.P=np.array([[0.9999510049819946, 0.009837210178375244, 0],[-0.009841140359640121, 0.9999340176582336, 0], [0, 0, 1]])
        
        ## Valores intrínsecos de cámara
        self.matriz_proyeccion = np.array([[509.0920104980469, 0.0, 299.10101318359375],
                                        [0.0, 509.0920104980469, 244.7949981689453],
                                        [0.0, 0.0, 1.0]], dtype=np.float32)
        
        self.fx = 509.0920104980469
        self.fy = 509.0920104980469
        self.centro_x = 299.10101318359375
        self.centro_y = 244.7949981689453
        
        self.matriz_proy_inversa = np.linalg.inv(self.matriz_proyeccion)
        
        # Dimensiones (ancho, alto, profundidad) del objeto
        self.dimensiones = [0.07, 0.095, 0.07]
        
        # Valores usados para el preprocesamiento de la imagen
        self.umbral=200
        self.kernel = np.ones((3, 3), np.uint8)

        # Definiciones de variables
        self.cvbridge_ = CvBridge()
        self.depth_image_ = Image()
        self.masks_ = Mask()
        self.bbox_3d = Bbox3d()
        self.center_point_ = Point()
        self.cv_depth_image = None
        self.cv_normalized_depth_image = None
        self.count_ = 0
        
        # Publishers
        self.publisher_bbox_3d_ = self.create_publisher(Bbox3d, '/bbox/bbox_3d', 10)
        self.publisher_center_point_ = self.create_publisher(Point, '/bbox/center_point', 10)

        # Subscribers
        self.subscription_masks_ = self.create_subscription(Mask, '/detectron2/masks_image', self.listener_callback_masks, 10)
        self.subscription_masks_
        self.subscription_depth_image_ = self.create_subscription(Image, '/detectron2/depth_image', self.listener_callback_depth_image, 10)
        self.subscription_depth_image_

    ## Función que se llama cada vez que se recibe un mensaje por el topic '/detectron2/masks_image'
    def listener_callback_masks(self, msg):

        self.masks_ = msg # Almacenamos el mensaje que llega del topic
        self.bbox_3d = Bbox3d()
        self.count_ = 0
        
        for i in range(len(self.masks_.mask_images)):
            
            # Comprobamos que se haya recibido el mensaje del topic de la imagen de profundidad
            if self.cv_normalized_depth_image is not None:
            
                # Convertimos la máscara a formato cv2 para poder trabajar con ella
                mask = self.cvbridge_.imgmsg_to_cv2(self.masks_.mask_images[i], self.masks_.mask_images[i].encoding)
            
                z = self.coordenada_z(self.cv_normalized_depth_image, mask) # Calculamos la coordenada Z a partir de la imagen de profundidad y la máscara
            
                if z != 0: # Si la coordenada Z distinta de 0 se realizan las operaciones, si es 0 el objeto no ha sido detectado correctamente
                
                    x, y = self.coordenadas_xy(mask, z) # Calculamos las coordenadas X e Y
                
                    self.calcular_bounding_box_3d((x, y, z), self.dimensiones) # Calculamos el bounding box 3D a partir de la coordenadas y las dimensiones del objeto

                    self.center_point_ = self.bbox_3d.centro_objeto[0]
                    
                    self.publisher_bbox_3d_.publish(self.bbox_3d) # Publicamos el bounding box 3D por el topic '/bbox/bbox_3d'
                    self.publisher_center_point_.publish(self.center_point_)
                    
                    self.count_ = self.count_ + 1
                
                    # Mostramos por pantalla la información obtenida
                    self.get_logger().info('\nObjeto %s\nCoordenada X: %s m\nCoordenada Y: %s m\nCoordenada Z: %s m\n' % (self.count_,x, y, z))
                    self.get_logger().info('\nCenter point:\n%s m\n' % self.center_point_)
            
                else:
                
                    # Mostramos por pantalla que el objeto no ha sido detectado correctamente
                    self.get_logger().info('Objeto %s no detectado' % self.count_)
            
        self.count_ = 0

    ## Función que se llama cada vez que se recibe un mensaje por el topic '/detectron2/depth_image'
    def listener_callback_depth_image(self, msg):
        
        self.depth_image_ = msg # Almacenamos el mensaje que llega del topic
        self.cv_depth_image = self.cvbridge_.imgmsg_to_cv2(self.depth_image_, 'passthrough') # Lo convertimos a formato cv2
        # Normalizamos la imagen de profundidad a valores entre 0 y 255 píxeles para poder comprenderla
        self.cv_normalized_depth_image = cv2.normalize(self.cv_depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
           
    ##  Función que calcula la coordenada Z del objeto mediante la imagen de profundidad y la máscara     
    def coordenada_z(self, cv_normalized_depth_image, mask):
        
        # Como la máscara se ha calculado mediante la imagen a color, y la imagen de profundidad no se realiza desde el mismo ángulo,
        # debemos obtener la imagen de profundidad como si se hubiera capturado desde el mismo ángulo que la imagen a color.
        # Para ello, mediante los parámetros extrínsecos que relacionan ambas cámaras podemos relacionar la máscara con la imagen de profundidad.
        
        # Realizamos la transformación
        cv_scaled_image_p = self.depth_to_color(cv_normalized_depth_image)
        
        # Obtenemos la máscara en la imagen de profundidad escalada
        cv_imagen_enmascarada_scaled = cv2.bitwise_and(cv_scaled_image_p, cv_scaled_image_p, mask=mask)

        # La pasamos a escala de grises
        # Verificamos que la imagen tenga el formato de color correcto
        if cv_imagen_enmascarada_scaled.shape[-1] == 3:
            # La pasamos a escala de grises solo si tiene 3 canales (BGR)
            cv_mask_depth = cv2.cvtColor(cv_imagen_enmascarada_scaled, cv2.COLOR_BGR2GRAY)
        else:
            # Si ya es escala de grises, simplemente la asignamos
            cv_mask_depth = cv_imagen_enmascarada_scaled
        
        # Binarizamos la máscara de profundidad
        cv_mask_depth[cv_mask_depth >= self.umbral] = 0
        cv_mask_depth[(cv_mask_depth < self.umbral) & (cv_mask_depth > 0)] = 255
        
        # Realizamos erosión y dilatación para eliminar puntos espurios
        cv_mask_depth = cv2.erode(cv_mask_depth, self.kernel, iterations=10)
        cv_mask_depth = cv2.dilate(cv_mask_depth, self.kernel, iterations=10)
        
        # Calculamos el centroide
        cX, cY, centroide_bool = self.calcular_centroide(cv_mask_depth)
        
        # Si se ha podido calcular el centroide correctamente calculamos la coordenada Z, en caso contrario será 0
        if centroide_bool:
            
            # A partir del centroide sacar la coordenada Z de la imagen de profundidad sin normalizar
            im_depth_scaled = self.depth_to_color(self.cv_depth_image) # Transformamos la imagen de profundidad sin normalizar
            z = 0.001*im_depth_scaled[cY][cX] # Pasamos a metros
            
        else:
            
            z = 0

        return z
    
    ## Función que transforma la imagen de profundidad al ángulo de captura de la imagen a color
    def depth_to_color(self, im):
        
        # Dimensiones originales de la imagen
        original_height, original_width = im.shape[:2]

        # Factor de escala
        scale_factor = 0.925

        # Calculamos el nuevo tamaño manteniendo las dimensiones originales
        new_height = int(original_height * scale_factor)
        new_width = int(original_width * scale_factor)

        # Calculamos los píxeles de relleno
        padding_top = (original_height - new_height) // 2
        padding_bottom = original_height - new_height - padding_top
        padding_left = (original_width - new_width) // 2
        padding_right = original_width - new_width - padding_left

        # Escalamos la imagen
        cv_scaled_image = cv2.resize(im, (new_width, new_height))

        # Añadimos los píxeles de relleno
        cv_scaled_image = cv2.copyMakeBorder(cv_scaled_image, padding_top, padding_bottom, padding_left, padding_right, cv2.BORDER_CONSTANT, value=0)

        # Aplicamos la transformación
        cv_scaled_image_a = cv2.warpAffine(cv_scaled_image, self.M, (im.shape[1], im.shape[0]))
        cv_scaled_image_p = cv2.warpPerspective(cv_scaled_image_a, self.P, (im.shape[1], im.shape[0]))
        
        return cv_scaled_image_p
    
    ## Función que calcula el centroide del objeto a partir de su máscara
    def calcular_centroide(self, mask):
        
        # Inicializamos las variables
        cX, cY = 0, 0
        centroide_bool = False
        
        # Obtenemos el contorno de la máscara
        contornos,hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        
        # Si únicamente tenemos un contorno, realizamos las operaciones
        if len(contornos) == 1:
            
            cnt = contornos[0]

            # Calculamos el momento para posteriormente utilizarlo en el cálculo de centroide
            Momento = cv2.moments(cnt)

            if Momento['m00'] != 0: # Comprobamos que no haya división por 0
                
                # Calculamos centroide
                cX=int(Momento['m10']/Momento['m00'])
                cY=int(Momento['m01']/Momento['m00'])
                
                centroide_bool = True # Se ha podido calcular el centroide
        
        return cX, cY, centroide_bool
    
    ## Función que calcula las coordenadas X e Y de una máscara
    def coordenadas_xy(self, mask, z):
        
        cX, cY, centroide_bool = self.calcular_centroide(mask) # Calculamos el centroide del objeto
        
        if centroide_bool: # Si se ha podido calcular el centroide realizamos las operaciones para pasarlo al mundo real
            
            centroide_real_x = (cX-self.centro_x) * z / self.fx
            centroide_real_y = (cY-self.centro_y) * z / self.fy
            
            x = centroide_real_x
            y = centroide_real_y
                        
        else: # En caso contrario se mantienen a 0
            
            x = 0
            y = 0
        
        return x, y
    
    ## Función que calcula el bounding box 3D del objeto a partir del centroide 3D y las dimensiones del objeto
    def calcular_bounding_box_3d(self, centroide, dimensiones):
        
        ancho, alto, profundidad = dimensiones
        x, y, z = centroide
        x = float(x)
        y = float(y)
        z = float(z) 
        
        # Calculamos centros y esquinas del bounding box
        centro_objeto = Point()
        centro_objeto.x = x
        centro_objeto.y = y
        centro_objeto.z = z + profundidad / 2
        self.bbox_3d.centro_objeto.append(centro_objeto)

        centro_frontal = Point()
        centro_frontal.x = x
        centro_frontal.y = y
        centro_frontal.z = z
        self.bbox_3d.centro_frontal.append(centro_frontal)

        centro_trasero = Point()
        centro_trasero.x = x
        centro_trasero.y = y
        centro_trasero.z = z + profundidad
        self.bbox_3d.centro_trasero.append(centro_trasero)

        centro_izquierdo = Point()
        centro_izquierdo.x = x - ancho / 2
        centro_izquierdo.y = y
        centro_izquierdo.z = z + profundidad / 2
        self.bbox_3d.centro_izquierdo.append(centro_izquierdo)

        centro_derecho = Point()
        centro_derecho.x = x + ancho / 2
        centro_derecho.y = y
        centro_derecho.z = z + profundidad / 2
        self.bbox_3d.centro_derecho.append(centro_derecho)

        esquina_frontal_inferior_izquierda = Point()
        esquina_frontal_inferior_izquierda.x = x - ancho / 2
        esquina_frontal_inferior_izquierda.y = y - alto / 2
        esquina_frontal_inferior_izquierda.z = z
        self.bbox_3d.esquina_frontal_inferior_izquierda.append(esquina_frontal_inferior_izquierda)

        esquina_frontal_superior_izquierda = Point()
        esquina_frontal_superior_izquierda.x = x - ancho / 2
        esquina_frontal_superior_izquierda.y = y + alto / 2
        esquina_frontal_superior_izquierda.z = z
        self.bbox_3d.esquina_frontal_superior_izquierda.append(esquina_frontal_superior_izquierda)

        esquina_frontal_inferior_derecha = Point()
        esquina_frontal_inferior_derecha.x = x + ancho / 2
        esquina_frontal_inferior_derecha.y = y - alto / 2
        esquina_frontal_inferior_derecha.z = z
        self.bbox_3d.esquina_frontal_inferior_derecha.append(esquina_frontal_inferior_derecha)

        esquina_frontal_superior_derecha = Point()
        esquina_frontal_superior_derecha.x = x + ancho / 2
        esquina_frontal_superior_derecha.y = y + alto / 2
        esquina_frontal_superior_derecha.z = z
        self.bbox_3d.esquina_frontal_superior_derecha.append(esquina_frontal_superior_derecha)

        esquina_trasera_inferior_izquierda = Point()
        esquina_trasera_inferior_izquierda.x = x - ancho / 2
        esquina_trasera_inferior_izquierda.y = y - alto / 2
        esquina_trasera_inferior_izquierda.z = z + profundidad
        self.bbox_3d.esquina_trasera_inferior_izquierda.append(esquina_trasera_inferior_izquierda)

        esquina_trasera_superior_izquierda = Point()
        esquina_trasera_superior_izquierda.x = x - ancho / 2
        esquina_trasera_superior_izquierda.y = y + alto / 2
        esquina_trasera_superior_izquierda.z = z + profundidad
        self.bbox_3d.esquina_trasera_superior_izquierda.append(esquina_trasera_superior_izquierda)

        esquina_trasera_inferior_derecha = Point()
        esquina_trasera_inferior_derecha.x = x + ancho / 2
        esquina_trasera_inferior_derecha.y = y - alto / 2
        esquina_trasera_inferior_derecha.z = z + profundidad
        self.bbox_3d.esquina_trasera_inferior_derecha.append(esquina_trasera_inferior_derecha)

        esquina_trasera_superior_derecha = Point()
        esquina_trasera_superior_derecha.x = x + ancho / 2
        esquina_trasera_superior_derecha.y = y + alto / 2
        esquina_trasera_superior_derecha.z = z + profundidad
        self.bbox_3d.esquina_trasera_superior_derecha.append(esquina_trasera_superior_derecha)

## Main
def main(args=None):
    
    rclpy.init(args=args)

    bbox = Bbox()

    rclpy.spin(bbox)

    bbox.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()