import numpy as np
from pinocchio import SE3
from pinocchio import StdVec_Bool as Mask
#from pinocchio.visualize import GepettoVisualizer
from pyhpp.gepetto import Viewer
from pyhpp.pinocchio import Device, LiegroupElement, urdf
from pyhpp.core import (
    createDiscretizedCollisionAndJointBound,
    Parameter,
    Problem,
    setVerbosityLevel)
from pyhpp.industrial.path_planner import CartesianRRT
from pyhpp.core import DiffusingPlanner
#from pyhpp.gepetto import Viewer

urdfFilename = (
    "package://example-robot-data/robots/ur_description/urdf/ur5_gripper.urdf"
)
srdfFilename = (
    "package://example-robot-data/robots/ur_description/srdf/ur5_gripper.srdf"
)

robot = Device.create("cell")
#viewer = Viewer("", robot)
#viewer.addURDFToScene(0, "ur5", "anchor", urdfFilename, srdfFilename, SE3.Identity())
urdf.loadModel(robot, 0, "ur5", "anchor", urdfFilename, srdfFilename, SE3.Identity())
#viewer.addURDFObstacleToScene("package://hpp-industrial/urdf/shelves.urdf", "shelves")
urdf.loadModel(robot, 0, "shelves", "anchor", "package://hpp-industrial/urdf/shelves.urdf", "",
               SE3(translation = np.array([.5, 0, 0]), rotation = np.identity(3)))

q = robot.currentConfiguration()
robot.currentConfiguration(q)

problem = Problem(robot)
pv = createDiscretizedCollisionAndJointBound(robot, 0.01)
problem.pathValidation(pv)
planner = CartesianRRT(problem, 6)
planner.maxIterations(1000)

# Set parameters for path planning algorithm
problem.setParameter("CartesianRRT/maxStepLength", Parameter.create(1.0))
cs = problem.configurationShooter()
q1 = None
# while True:
#     q1 = cs.shoot()
#     res, report = pv.validateConfiguration(q1)
#     if res: break
# while True:
#     q2 = cs.shoot()
#     res, report = pv.validateConfiguration(q2)
#     if res: break

# problem.initConfig(q1)
# problem.addGoalConfig(q2)

q= robot.currentConfiguration()
#p = planner.solve()

viz = Viewer(robot)

