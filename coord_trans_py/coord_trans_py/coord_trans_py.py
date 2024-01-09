import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
import numpy as np

from sensor_msgs.msg import Image


class ArucoTFPublisher(Node):
    
    def __init__(self):
        super().__init__('aruco_tf_publisher')

        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)
        self.center_point_camera_ = PointStamped()
        self.center_point_aruco_ = PointStamped()
        self.center_point_robot_ = PointStamped()
        self.matriz_trans_aruco_robot_ = np.array([[1, 0, 0, -0.015], [0, 1, 0, 0.155], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.matriz_trans_inv_aruco_robot_ = np.linalg.inv(self.matriz_trans_aruco_robot_)

        # Crear publicador de transformaciones
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Publishers
        self.publisher_center_point_ = self.create_publisher(PointStamped, '/coord_trans/center_point_from_robot', 10)
        
        # Suscriptores
        self.subscription_center_point_ = self.create_subscription(PointStamped, '/calibration/center_point', self.listener_callback_center_point, 10)
            
    def listener_callback_center_point(self, msg):
    
        # Recibo el mensaje (punto central con respecto a la cámara)
        self.center_point_camera_ = msg
        
        lookup_time = self.get_clock().now()
        
        # Lo calculo con respecto al ArUCo
        self.center_point_aruco_ = self.tf_buffer_.transform(self.center_point_camera_, "aruco_frame_5")
        
        center_point_camera_aruco = np.array([[1, 0, 0, self.center_point_aruco_.point.x],
                                              [0, 1, 0, self.center_point_aruco_.point.y],
                                              [0, 0, 1, self.center_point_aruco_.point.z],
                                              [0, 0, 0, 1]])
        
        # Lo calculo con respecto al manipulador
        center_point_aruco_robot = np.dot(self.matriz_trans_inv_aruco_robot_, center_point_camera_aruco)
        
        # Abre el archivo en modo de escritura. Si no existe, se crea. Si existe, se sobrescribe.
        with open('/home/tfg/dectmani_ws/src/coord_trans_py/coord_trans_py/object_point_from_robot.txt', 'w') as archivo:
            archivo.write('[%s, %s, %s]' % (center_point_aruco_robot[0,3], center_point_aruco_robot[1,3], center_point_aruco_robot[2,3]))
        
        # Creo el PointStamped para visualizarlo en rviz y publicarlo por el topic
        self.center_point_robot_.header.frame_id = "robot"
        self.center_point_robot_.point.x = center_point_aruco_robot[0,3]
        self.center_point_robot_.point.y = center_point_aruco_robot[1,3]
        self.center_point_robot_.point.z = center_point_aruco_robot[2,3]
        
        print(self.center_point_robot_)
        
        # Se publica
        self.publisher_center_point_.publish(self.center_point_robot_)
       
        
def main(args=None):
    
    rclpy.init(args=args)

    aruco_publisher_node = ArucoTFPublisher()

    rclpy.spin(aruco_publisher_node)
    
    aruco_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
