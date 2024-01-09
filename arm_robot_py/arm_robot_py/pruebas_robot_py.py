from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

moving_time=5
accel_time=1

x=-0.17328662649155968
y=-0.38079327205971525

def main():
    bot = InterbotixManipulatorXS(
        robot_model='wx250',
        group_name='arm',
        gripper_name='gripper'   
    )
    
    bot.arm.go_to_home_pose(moving_time=moving_time, accel_time=accel_time)
    
    bot.arm.set_ee_pose_components(x= x, y=y, z=0.25, roll=0, pitch=1.25, moving_time=moving_time, accel_time=accel_time)
    bot.arm.set_ee_pose_components(x= x, y=y, z=0.1, roll=0, pitch=1.25, moving_time=moving_time, accel_time=accel_time)
    bot.gripper.grasp(2.0)
    
    bot.arm.set_ee_pose_components(x= x, y=y, z=0.25, roll=0, pitch=1.25, moving_time=moving_time, accel_time=accel_time)
    bot.arm.set_ee_pose_components(x=0.4, y=0, z=0.3, roll=0, pitch=1.25, moving_time=moving_time, accel_time=accel_time)
    bot.arm.set_ee_pose_components(x=0.4, y=0, z=0.16, roll=0, pitch=1.25, moving_time=moving_time, accel_time=accel_time)
    bot.gripper.release(2.0)
    
    bot.arm.go_to_sleep_pose(moving_time=moving_time, accel_time=accel_time)
    bot.shutdown()


if __name__ == '__main__':
    main()
    
