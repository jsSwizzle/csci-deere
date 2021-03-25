"""File to demo arm_controller module.
"""
from arm_controller.py_segment import PySegment
from arm_controller.py_chain import PyChain
from arm_controller.solver import Solver
from arm_controller.plotter import Plotter
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

if __name__ == '__main__':
    # construct segments for the chain
    # world_segment = PySegment('seg0', 'world', 0.0, 0.0, 0.0, [0.0,0.0,0.0], [0.0,0.0,56.0], None)
    # waist_segment = PySegment('seg1', 'waist', 90.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,42.93], 'Z', joint_no=0)
    # shoulder_segment = PySegment('seg2', 'shoulder', 30.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,120.0], 'Y', joint_no=1)
    # elbow_segment = PySegment('seg3', 'elbow', 35.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,118.65], 'Y', joint_no=2)
    # wrist_roll_segment = PySegment('seg4', 'wrist_roll', 140.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,60.028], 'Z', joint_no=4)
    # wrist_pitch_segment = PySegment('seg5', 'wrist_pitch', 80.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,30.17], 'Y', joint_no=5)
    #
    # # add the segments to the chain
    # myChain = PyChain()
    # myChain.append_segment(world_segment)
    # myChain.append_segment(waist_segment)
    # myChain.append_segment(shoulder_segment)
    # myChain.append_segment(elbow_segment)
    # myChain.append_segment(wrist_roll_segment)
    # myChain.append_segment(wrist_pitch_segment)
    #
    # # set the segments to their default values
    # for segment in myChain.segments:
    #     segment.current_val = segment.default_value

    myArm = PlotterArm()

    # display the current values
    current_angles = myArm._chain.get_current_values()
    print(f'Current Joint Angles: {current_angles}\n')

    mySolver = Solver(myChain)
    coords, rpy = mySolver.forward_solve(current_angles)
    print('INITIAL FK SOLVE ON CURRENT ANGLES')
    print(f'FK Coords: {coords}\nFK RPY: {rpy}\n')

    list_o_coords = mySolver.segmented_forward_solve(current_angles)

    target_x = input("Enter Target X Coord: ")
    target_y = input("Enter Target Y Coord: ")
    target_z = input("Enter Target Z Coord: ")
    input_angles = [float(target_x), float(target_y), float(target_z)]

    angles = mySolver.inverse_solve(current_angles, input_angles, [0.0, 0.0, 0.0])
    print(f'\nIK SOLVE TO {input_angles}')
    print(f'IK Solution: {angles}\n')

    # angles = mySolver.inverse_solve(current_angles, [-20.0, 250.0, 90.0], [0.0, 0.0, 0.0])
    # print('IK SOLVE TO [-20, 250, 90] WITH ZEROED RPY')
    # print(f'IK Solution: {angles}\n')

    i = 0
    for segment in myChain.segments:
        if segment.joint_rot != None:
            segment.current_val = angles[i]
            i += 1

    coords, rpy = mySolver.forward_solve(angles)
    print('PLUG IK SOLUTION BACK INTO FK')
    print(f'FK Coords: {coords}\nFK RPY: {rpy}\n')

    list_o_coords2 = mySolver.segmented_forward_solve(angles)

    ax = plt.axes(projection='3d')
    # Plotter.create_timelapse(list_o_coords, list_o_coords2, ax, 8)
    Plotter.create_lines([list_o_coords, list_o_coords2], ax)
    plt.show()
