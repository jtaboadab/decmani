from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import ast

moving_time_1=3
accel_time_1=0.75

moving_time_2=2
accel_time_2=0.3

def main():
    bot = InterbotixManipulatorXS(
        robot_model='wx250',
        group_name='arm',
        gripper_name='gripper'   
    )
    
    while(True):
        
        # ESpera hasta pulsar 'enter'
        input()
        
        # Inicializa el punto destino
        desp=-0.08
        
        # El manipulador se sitúa en la pisición de inicio
        bot.arm.go_to_home_pose(moving_time=moving_time_1, accel_time=accel_time_1)
        
        while(True):
            
            # Abre el archivo en modo de lectura
            with open('/home/tfg/dectmani_ws/src/detectron2_py/detectron2_py/contador_objetos.txt', 'r') as archivo:

                # Lee el contenido del archivo como una cadena
                contenido = archivo.read()

                # Convierte la cadena en una lista de números utilizando ast.literal_eval
                contador_objetos = ast.literal_eval(contenido)
                
            # Si no hay objetos vuelve a la posición de reposo
            if contador_objetos == 0:
                
                bot.arm.go_to_home_pose(moving_time=moving_time_2, accel_time=accel_time_2)
                bot.arm.go_to_sleep_pose(moving_time=moving_time_1, accel_time=accel_time_1)
                break
                
            # Abre el archivo en modo de lectura
            with open('/home/tfg/dectmani_ws/src/coord_trans_py/coord_trans_py/object_point_from_robot.txt', 'r') as archivo:

                # Lee el contenido del archivo como una cadena
                contenido = archivo.read()

                # Convierte la cadena en una lista de números utilizando ast.literal_eval
                lista_numeros = ast.literal_eval(contenido)

                # Almacena el primer y segundo número en las variables x e y
                x = lista_numeros[0]
                y = lista_numeros[1]
                
            # Realiza la trayectoria planificada
            bot.arm.set_ee_pose_components(x=x, y=y, z=0.25, roll=0, pitch=1.25, moving_time=moving_time_1, accel_time=accel_time_1)
            bot.arm.set_ee_pose_components(x=x, y=y, z=0.1, roll=0, pitch=1.25, moving_time=moving_time_2, accel_time=accel_time_2)
            bot.gripper.grasp(2.0)
            
            bot.arm.set_ee_pose_components(x=x, y=y, z=0.3, roll=0, pitch=1.25, moving_time=moving_time_2, accel_time=accel_time_2)
            bot.arm.set_ee_pose_components(x=0.4, y=0-desp, z=0.3, roll=0, pitch=1.25, moving_time=moving_time_1, accel_time=accel_time_1)
            bot.arm.set_ee_pose_components(x=0.4, y=0-desp, z=0.16, roll=0, pitch=1.25, moving_time=moving_time_2, accel_time=accel_time_2)
            bot.gripper.release(2.0)
            
            bot.arm.set_ee_pose_components(x=0.4, y=0-desp, z=0.3, roll=0, pitch=1.25, moving_time=moving_time_2, accel_time=accel_time_2)
            
            # Cambiamos el punto destino para el siguiente vaso
            desp=desp+0.08
            
            # Si no hay objetos vuelve a la posición de reposo
            if contador_objetos == 0:
                
                bot.arm.go_to_home_pose(moving_time=moving_time_2, accel_time=accel_time_2)
                bot.arm.go_to_sleep_pose(moving_time=moving_time_1, accel_time=accel_time_1)
                break
        
        
if __name__ == '__main__':
    main()
    
    