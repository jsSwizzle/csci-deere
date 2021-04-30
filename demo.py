"""File to demo arm_controller module.
"""

# the only import we need is the PlotterArm itself
from arm_controller.arms.plotter_arm import PlotterArm

def demoProgram():
    print('-- ARM CONTROLLER DEMO PROGRAM --')

    print('\nfirst we create the arm object via:')
    print('arm = PlotterArm()')
    print('we must wait for the arm to move to its default position before control is returned')
    arm = PlotterArm()

    input('\nPress Enter to Continue')
    print('next we will grab the current position via:')
    print('xyz, rpy = arm.get_pos()')
    xyz, rpy = arm.get_pos()
    print(f'xyz = {xyz}\nrpy = {rpy}')

    input('\nPress Enter to Continue')
    print('lets try moving arm to a differnt position via:')
    print('arm.move_to(0.04, 0.06, 0.09)')
    arm.move_to(0.04, 0.06, 0.09)

    input('\nPress Enter to Continue')
    print('now we will slow down the speed of the arm via:')
    print('arm.set_speed(0.17, radians=True)')
    arm.set_speed(0.17, radians=True)

    input('\nPress Enter to Continue')
    print('now we will move the arm again, this time supplying an rpy:')
    print('arm.move_to(-0.04, 0.06, 0.08, 10, 0, 0)')
    arm.move_to(-0.04, 0.06, 0.08, 10, 10, 0)

    input('\nPress Enter to Continue')
    print('to confirm its new location lets get its position')
    xyz, rpy = arm.get_pos()
    print(f'xyz = {xyz}\nrpy = {rpy}')

    input('\nPress Enter to Contine')
    print('finally we can use set_joint by itself to set specific joints')
    print('first the shoulder joint: arm.set_joint("shoulder", 150, radians=False)')
    arm.set_joint('shoulder', 150, radians=False)
    print('and the wrist pitch: arm.set_joint("wrist_pitch", 90, radians=False)')
    arm.set_joint('wrist_pitch', 90, radians=False)

    input('\nPress Enter to Continue')
    print('do not forget to call exit() on the plotter to inform the process to exit and join on the arm proc variable')
    arm.exit()
    arm.proc.join()

# only call the demo program if this file is being ran
if __name__ == '__main__':
    demoProgram()
