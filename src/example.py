from gripper_controller import GripperController
import time


"""
Example script to control the finger joint angles
"""

homepos = [0, 0]
goalpos = [50, 80]


def main():
    global gc
    gc = GripperController(port="/dev/ttyUSB0",calibration=True)
    # motor_pos_list = gc.get_motor_pos()
    # print(motor_pos_list)

    # gc.write_desired_joint_angles(goalpos)

    # gc.wait_for_motion()

    # time.sleep(1)

    # gc.write_desired_joint_angles(homepos)

    # gc.wait_for_motion()

    # time.sleep(1)

    gc.terminate()


if __name__ == "__main__":
    main()