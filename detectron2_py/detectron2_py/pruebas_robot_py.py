from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

moving_time=5
accel_time=1

def main():
    bot = InterbotixManipulatorXS(
        robot_model='wx250',
        group_name='arm',
        gripper_name='gripper'   
    )
    
    bot.arm.go_to_home_pose(moving_time=moving_time, accel_time=accel_time)
    bot.arm.set_ee_pose_components(x=-0.05, y=-0.25, z=0.15, roll=0, pitch=0, moving_time=moving_time, accel_time=accel_time)
    bot.arm.set_ee_pose_components(x=-0.1, y=-0.35, z=0.15, roll=0, pitch=0, moving_time=moving_time, accel_time=accel_time)
    bot.gripper.grasp(2.0)
    bot.arm.set_ee_pose_components(x=-0.05, y=-0.35, z=0.20, roll=0, pitch=0, moving_time=moving_time, accel_time=accel_time)
    bot.arm.go_to_home_pose(moving_time=moving_time, accel_time=accel_time) 
    bot.arm.set_ee_pose_components(x=-0.05, y=0.25, z=0.20, roll=0, pitch=0, moving_time=moving_time, accel_time=accel_time)
    bot.arm.set_ee_pose_components(x=-0.05, y=0.35, z=0.15, roll=0, pitch=0, moving_time=moving_time, accel_time=accel_time)
    bot.gripper.release(2.0)
    bot.arm.go_to_home_pose(moving_time=moving_time, accel_time=accel_time)
    bot.arm.go_to_sleep_pose(moving_time=moving_time, accel_time=accel_time)
    bot.shutdown()


if __name__ == '__main__':
    main()