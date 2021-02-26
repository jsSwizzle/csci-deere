"""File to demo arm_controller library.
"""
from arm_controller.py_segment import PySegment
from arm_controller.py_chain import PyChain
from arm_controller.solver import Solver

if __name__ == '__main__':
    world_segment = PySegment('seg0', 'world', 0.0, 0.0, 0.0, [0.0,0.0,0.0], [0.0,0.0,56.0], None)
    waist_segment = PySegment('seg1', 'waist', 90.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,42.93], 'Z')
    shoulder_segment = PySegment('seg2', 'shoulder', 150.0, 0.0, 180.0, [0.0,0.0,0.0], [-120.0,0.0,0.0], 'Y')
    elbow_segment = PySegment('seg3', 'elbow', 35.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,-118.65], 'Y')
    wrist_roll_segment = PySegment('seg4', 'wrist_roll', 140.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,-60.028], 'Z')
    wrist_pitch_segment = PySegment('seg5', 'wrist_pitch', 80.0, 0.0, 180.0, [0.0,0.0,0.0], [30.17,0.0,0.0], 'Y')
    myChain = PyChain()
    myChain.append_segment(world_segment)
    myChain.append_segment(waist_segment)
    myChain.append_segment(shoulder_segment)
    myChain.append_segment(elbow_segment)
    myChain.append_segment(wrist_roll_segment)
    myChain.append_segment(wrist_pitch_segment)

    for segment in myChain.segments:
        segment.current_val = segment.default_value

    current_angles = myChain.get_current_values()
    print(f'Current Joint Angles: {current_angles}\n')

    mySolver = Solver(myChain)
    coords, rpy = mySolver.forward_solve(current_angles)
    print('INITIAL FK SOLVE ON DEFAULTS')
    print(f'FK Coords: {coords}\nFK RPY: {rpy}\n')

    angles = mySolver.inverse_solve(current_angles, [0.0, 150.0, 90.0], [0.0, 0.0, 0.0])
    print('IK SOLVE TO (0, 300, 100) WITH ZEROED RPY')
    print(f'IK Solution: {angles}\n')

    coords, rpy = mySolver.forward_solve(angles)
    print('PLUG IK SOLUTION BACK INTO FK')
    print(f'FK Coords: {coords}\nFK RPY: {rpy}\n')
