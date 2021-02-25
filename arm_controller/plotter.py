import enum
import math
import numpy

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from .py_chain import PyChain
from .py_segment import PySegment
from .solver import Solver


class Plotter:

    def plot_arm(arm: PyChain):
        pass
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

    # def plot_3Dpts(xs: [], ys: [], zs: []):
    #     ax = plt.axes(projection='3d')
    #     lines = ax.plot3D(xs, ys, zs)
    #     plt.setp(lines, color='r', linewidth=2.0, marker='+', mew=1.0, mec='b')
    #     plt.grid(True)
    #     ax.set_xlabel('x')
    #     ax.set_ylabel('y')
    #     ax.set_zlabel('z')
    #     ax.step(.2, .2)
    #     plt.show()

    # plot_arm(FakeArm())
    # Plotter.plot_arm3D(FakeArm())
