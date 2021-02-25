import enum
import math
import numpy

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


# from py_chain import PyChain
# from .py_segment import PySegment
# from .solver import Solver

def calculateDeltas(a0: [], a1: []):
    ret = []
    for i, a in enumerate(a1):
        ret.append(a - a0[i])
    print(ret)
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

    def create_lines(coords, ax):
        lines = []
        for c in coords:
            xs = [item[0] for item in c]
            ys = [item[1] for item in c]
            zs = [item[2] for item in c]
            print(f'{xs}, {ys}, {zs}')
            lines.append(ax.plot3D(xs, ys, zs))
        return lines

    def create_timelapse(coord1, coord2, ax, steps: int):
        lines = []
        x0s, x1s = [item[0] for item in coord1], [item[0] for item in coord2]
        xds = calculateDeltas(x0s, x1s)
        y0s, y1s = [item[1] for item in coord1], [item[1] for item in coord2]
        yds = calculateDeltas(y0s, y1s)
        z0s, z1s = [item[2] for item in coord1], [item[2] for item in coord2]
        zds = calculateDeltas(z0s, z1s)
        lines.append(ax.plot3D(x0s, y0s, z0s))
        currSteps = steps - 1
        while currSteps > 0:
            xs = []
            for i, x in enumerate(x1s):
                xs.append(x - (xds[i] / steps * currSteps))
            ys = []
            for i, y in enumerate(y1s):
                ys.append(y - (yds[i] / steps * currSteps))
            zs = []
            for i, z in enumerate(z1s):
                zs.append(z - (zds[i] / steps * currSteps))
            lines.append(ax.plot3D(xs, ys, zs))
            currSteps -= 1
        lines.append(ax.plot3D(x1s, y1s, z1s))
        return lines


test_coordinates1 = [[0, 0, 0], [0, 0, 1], [-.5, -.5, 1.5], [0, .5, 2], [1, 1, 1.5]]
test_coordinates2 = [[0, 0, 0], [0, 0, 1.8], [-.5, -.5, 1.8], [0, .4, 2.2], [1, 1, 1]]

if __name__ == '__main__':
    ax = plt.axes(projection='3d')
    # Plotter.create_lines([test_coordinates1, test_coordinates2], ax);
    Plotter.create_timelapse(test_coordinates1, test_coordinates2, ax, 5)
    plt.show()
    # plot_arm(FakeArm())
    # Plotter.plot_arm3D(FakeArm())
