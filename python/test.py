import numpy as np
from pinocchio import SE3
from pinocchio import StdVec_Bool as Mask
#from pinocchio.visualize import GepettoVisualizer
from pyhpp.gepetto import (
    Viewer)
from pyhpp.pinocchio import LiegroupElement
from pyhpp.manipulation.constraint_graph_factory import (
    Rule,
    ConstraintGraphFactory)
from pyhpp.manipulation import (
    Device,
    Graph,
    Problem,
    TransitionPlanner,
    urdf)
from pyhpp.core import (
    createDiscretizedCollisionAndJointBound,
    Parameter,
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
cylinderUrdf = "package://hpp-industrial/urdf/cylinder.urdf"
cylinderSrdf = "package://hpp-industrial/srdf/cylinder.srdf"

shelvesUrdf = "package://hpp-industrial/urdf/shelves.urdf"
shelvesSrdf = "package://hpp-industrial/srdf/shelves.srdf"

robot = Device("cell")
#viewer = Viewer("", robot)
urdf.loadModel(robot, 0, "ur5", "anchor", urdfFilename, srdfFilename, SE3.Identity())
urdf.loadModel(robot, 0, "cylinder", "freeflyer", cylinderUrdf, cylinderSrdf, SE3.Identity())
urdf.loadModel(robot, 0, "shelves", "anchor", shelvesUrdf, shelvesSrdf,
               SE3(translation = np.array([.8, 0, 0]), rotation = np.identity(3)))
# Set bounds on cylinder translation
m = robot.model()
r = m.idx_qs[m.getJointId('cylinder/root_joint')]
m.lowerPositionLimit[r:r+3] = np.array([0, -1., -.5])
m.upperPositionLimit[r:r+3] = np.array([1.5, 1., 1.5])

q = robot.currentConfiguration()
robot.currentConfiguration(q)

q1 = None

q = np.array([0.   , 0.   , 0.   , 0.   , 0.   , 0.   , 0.8  , 0.   , 0.292,
              0.   , 0.   , 0.   , 1.   ])
#p = planner.solve()

# Create the constraint graph
grippers = ["ur5/gripper"]
objects = ["cylinder"]
handlesPerObject = [["cylinder/h1"]]
contactSurfacesPerObject = [["cylinder/bottom"]]
envContactSurfaces = ["shelves/floor_0", "shelves/floor_1", "shelves/floor_2", "shelves/floor_3"]
rules = [Rule([".*"], [".*"], True)]

problem = Problem(robot)
cg = Graph("graph", robot, problem)
cg.maxIterations(100)
cg.errorThreshold(0.0001)
factory = ConstraintGraphFactory(cg)

factory.setGrippers(grippers)
factory.environmentContacts(envContactSurfaces)
factory.setObjects(objects, handlesPerObject, contactSurfacesPerObject)
factory.setRules(rules)
factory.generate()
cg.initialize()
problem.constraintGraph(cg)

edge = cg.getTransition("ur5/gripper > cylinder/h1 | f_01")
res = cg.generateTargetConfig(edge, q, q)
if res.success:
    q1 = res.configuration
    err = res.error

# Define initial and goal configuration

q = q1[:]
q[8] +=.25
state = cg.getState("free")
res = cg.applyStateConstraints(state, q)
assert(res.success)
q1 = res.configuration
res = cg.generateTargetConfig(edge, q1, q1)
assert(res.success)
q2 = res.configuration

problem.initConfig(q1)
problem.addGoalConfig(q2)

# Create a transition planner

planner = CartesianRRT(problem, 6)

# Set parameters for path planning algorithm
problem.setParameter("CartesianRRT/maxStepLength", Parameter.create(1.0))
tplanner = TransitionPlanner(problem)
tplanner.innerPlanner(planner)
tplanner.maxIterations(1000)
edge = cg.getTransition("Loop | f")
tplanner.setEdge(edge)

qInit = q1[:]
qGoal = np.zeros(2*robot.configSize())
qGoal.shape = (2, robot.configSize())
qGoal[0,:] = q2
qGoal[1,:] = q2

p = tplanner.planPath(qInit, qGoal, True)
