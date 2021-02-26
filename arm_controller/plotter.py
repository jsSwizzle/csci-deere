import enum
import math
import numpy
# import squaternion

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


# from py_chain import PyChain
# from .py_segment import PySegment
# from .solver import Solver

def calculateDeltas(arr0: [], arr1: []):
    ret = []
    for i, (a0, a1) in enumerate(zip(arr0, arr1)):
        ret.append(a1 - a0)
    return ret


class Plotter:

    # def plot_arm(arm: PyChain):
    #     pass
    # segLens = []
    # for seg in arm._segment_info:
    #     segLen = arm._segment_info[seg]['segment_length']
    #     segLens.append(segLen)
    #
    # xs, ys = [0], [0]
    # currPt = [0, 0]
    # currAng = math.pi / 2
    # # solv = arm._solver.specific_inverse_solve(.25, .25, 1, 0, 0)  # arbitrary values
    # solv = arm._servo_info
    # solv.pop('jt6')
    # for i, a in enumerate(solv):
    #     currAng = solv[a]['default_value']
    #     xx = currPt[0] + (segLens[i] * numpy.cos(currAng))
    #     yy = currPt[1] + (segLens[i] * numpy.sin(currAng))
    #     xs.append(xx)
    #     ys.append(yy)
    #     currPt[0] = xx
    #     currPt[1] = yy
    #     # currAng = solv[a]['final_angle']
    # Plotter.plot_pts(xs, ys)

    # def plot_arm3D(arm: PyChain):
    #     segLens = []
    #     for seg in arm._segment_info:
    #         segLen = arm._segment_info[seg]['segment_length']
    #         segLens.append(segLen)
    #     xs, ys, zs = [0], [0], [0]
    #     currPt = [0, 0, 0]
    #     currAng = 0
    #     # solv = arm._solver.specific_inverse_solve(.35, .45, .10, 0, 0)  # arbitrary values
    #     solv = arm._servo_info
    #     solv.pop('jt6')
    #     for i, a in enumerate(solv):
    #         currAng = solv[a]['default_value']
    #         xx = currPt[0] + (segLens[i] * numpy.cos(currAng))
    #         yy = currPt[1] + (segLens[i] * numpy.sin(currAng))
    #         zz = currPt[2] + (segLens[i] * numpy.tan(currAng))
    #         xs.append(xx)
    #         ys.append(yy)
    #         zs.append(zz)
    #         currPt[0] = xx
    #         currPt[1] = yy
    #         currPt[2] = zz
    #         # currAng = solv[a]['final_angle']
    #     Plotter.plot_3Dpts(xs, ys, zs)

    def plot_pts(xs: [], ys: []):
        pass
        # lines = plt.plot(xs, ys)
        # plt.setp(lines, color='r', linewidth=2.0, marker='+', mew=1.0, mec='b')
        # plt.grid(True)
        # plt.xlabel('x')
        # plt.ylabel('y')
        # plt.step(.2, .2)
        # plt.show()

    def plot_3Dpts(xs: [], ys: [], zs: []):
        ax = plt.axes(projection='3d')
        lines = ax.plot3D(xs, ys, zs)
        plt.setp(lines, color='r', linewidth=2.0, marker='+', mew=1.0, mec='b')
        plt.grid(True)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.step(.2, .2)
        plt.show()

    def create_lines(coords: [[[]]], ax):
        lines = []
        for c in coords:
            xs = [item[0] for item in c]
            ys = [item[1] for item in c]
            zs = [item[2] for item in c]
            # print(f'{xs}, {ys}, {zs}')
            ln = ax.plot3D(xs, ys, zs)
            plt.setp(ln, marker='+', mec='k')
            lines.append(ln)
        ax.set_xlim3d(-300, 300)
        ax.set_ylim3d(-300, 300)
        ax.set_zlim3d(0, 300)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        return lines

    def create_timelapse(coords1: [[]], coords2: [[]], ax, steps: int):
        lines = []
        x0s, x1s = [item[0] for item in coords1], [item[0] for item in coords2]
        xds = calculateDeltas(x0s, x1s)
        y0s, y1s = [item[1] for item in coords1], [item[1] for item in coords2]
        yds = calculateDeltas(y0s, y1s)
        z0s, z1s = [item[2] for item in coords1], [item[2] for item in coords2]
        zds = calculateDeltas(z0s, z1s)
        ln0 = ax.plot3D(x0s, y0s, z0s)
        plt.setp(ln0, marker='+', mec='k')
        lines.append(ln0)
        currSteps = steps - 1
        while currSteps > 0:
            xs, ys, zs = [], [], []
            for i, (x, d) in enumerate(zip(x1s, xds)):
                xs.append(x - (d / steps * currSteps))
            for i, (y, d) in enumerate(zip(y1s, yds)):
                ys.append(y - (d / steps * currSteps))
            for i, (z, d) in enumerate(zip(z1s, zds)):
                zs.append(z - (d / steps * currSteps))
            ln = ax.plot3D(xs, ys, zs)
            plt.setp(ln, alpha=1 - (currSteps / steps))
            lines.append(ln)
            currSteps -= 1
        ln1 =ax.plot3D(x1s, y1s, z1s)
        plt.setp(ln1, marker='x', mec='k')
        lines.append(ln1)
        ax.set_xlim3d(-300, 300)
        ax.set_ylim3d(-300, 300)
        ax.set_zlim3d(0, 300)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        return lines


test_coordinates1 = [[0, 0, 0], [0, 0, 1], [-50, -50, 150], [0, 50, 20], [10, 10, 150]]
test_coordinates2 = [[0, 0, 0], [30, 0, 180], [-50, -50, 1.8], [0, 40, 220], [20, 150, 20]]
test_coordinates3 = [[0, 0, 0], [50, 0, 50], [70, -50, 1], [0, 40, 70], [120, 70, 120]]

# if __name__ == '__main__':
#     ax = plt.axes(projection='3d')
# # #
# #     Plotter.create_lines([test_coordinates1, test_coordinates2, test_coordinates3], ax);
#     Plotter.create_timelapse(test_coordinates1, test_coordinates3, ax, 4)
#     Plotter.create_timelapse(test_coordinates1, test_coordinates2, ax, 4)
#     plt.show()
    # plot_arm(FakeArm())
    # Plotter.plot_arm3D(FakeArm())
