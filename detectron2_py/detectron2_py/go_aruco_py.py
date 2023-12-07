from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

moving_time=5
accel_time=1

def main():
    bot = InterbotixManipulatorXS(
        robot_model='wx250',
        group_name='arm',
        gripper_name='gripper'        
    )

    bot.arm.set_single_joint_position('wrist_angle', 0.25, moving_time, accel_time)
    bot.shutdown()

if __name__ == '__main__':
    main()
