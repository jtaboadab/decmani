# Librerías para crear el nodo de ROS2
import rclpy
from rclpy.node import Node

# Librerías de mensajes de ROS2
from geometry_msgs.msg import PointStamped

# Librería del brazo robot
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

# Otras librerías
import numpy as np
import keyboard


# Definición de la clase del nodo de ROS2
class Robot(Node):

    def __init__(self):

        super().__init__('robot')
        
        # Variables y deinición del robot 
        self.moving_time=5
        self.accel_time=1

        def main():
            self.bot = InterbotixManipulatorXS(
                robot_model='wx250',
                group_name='arm',
                gripper_name='gripper'        
            )
        
        # Variables
        self.center_point_ = PointStamped()

        # Subscribers
        self.subscription_center_point_ = self.create_subscription(PointStamped, '/coord_trans/center_point_from_robot', self.listener_callback_center_point, 10)
        self.subscription_center_point_

    # Callback para la imagen en color
    def listener_callback_center_point(self, msg):

        self.center_point_ = msg
        
        center_point = np.array([self.center_point_.point.x, self.center_point_.point.y, self.center_point_.point.z])
        
        x = center_point[0]
        y = center_point[1]
        
        print('Punto del objeto con respecto al robot:\n%s' % self.center_point_)
        
        if keyboard.is_pressed('space'):
            
            print('Cogiendo el objeto')
            self.bot.arm.go_to_home_pose(moving_time=self.moving_time, accel_time=self.accel_time)
    
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=0.25, roll=0, pitch=1.25, moving_time=self.moving_time, accel_time=self.accel_time)
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=0.1, roll=0, pitch=1.25, moving_time=self.moving_time, accel_time=self.accel_time)
            self.bot.gripper.grasp(2.0)
            
            print('Soltando el objeto')
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=0.25, roll=0, pitch=1.25, moving_time=self.moving_time, accel_time=self.accel_time)
            self.bot.arm.go_to_home_pose(moving_time=self.moving_time, accel_time=self.accel_time)
            self.bot.gripper.release(2.0)
            
        if keyboard.is_pressed('return'):
            
            self.bot.arm.go_to_sleep_pose(moving_time=self.moving_time, accel_time=self.accel_time)
            self.bot.shutdown()
        
        
# Función principal para ejecutar el nodo
def main(args=None):
    rclpy.init(args=args)

    robot = Robot()

    rclpy.spin(robot)

    robot.destroy_node()
    rclpy.shutdown()

# Ejecuta la función principal si el script es el programa principal
if __name__ == '__main__':
    main()