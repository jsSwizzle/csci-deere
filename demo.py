"""File to demo arm_controller module.
"""
from arm_controller.arms.plotter_arm import PlotterArm
import time

def mainProgram():
    arm = PlotterArm()
    print('Made an arm... Please Enter Commands')
    while 1:
        user_input = input('>> ')

        if user_input == 'exit':
            print("Exiting Plotter Control")
            arm.anim_variables['exit'] = True
            break
        else:
            print('Unknown Command')

        time.sleep(0.5)
    arm.proc.join()

if __name__ == '__main__':
    mainProgram()
