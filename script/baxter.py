#/usr/bin/env python
# Script which goes with baxter_description package.

from hpp.corbaserver.baxter import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import numpy as np

robot = Robot ('robot')
ps = ProblemSolver (robot)
cl = robot.client
xBaxter = 1.2; yBaxter = -1.65; zBaxter = 1.

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("room_description","room","room")

q1 = robot.getCurrentConfig ();
q1 [robot.rankInConfiguration ['torso_t0']] = 0
q1 [robot.rankInConfiguration ['head_pan']] = 0.2
q1 [robot.rankInConfiguration ['left_s0']] = -0.9 # turn +left/right-
q1 [robot.rankInConfiguration ['left_s1']] = -0.32 # +up/down-
q1 [robot.rankInConfiguration ['left_e0']] = 0
q1 [robot.rankInConfiguration ['left_e1']] = 0.68
q1 [robot.rankInConfiguration ['left_w0']] = 0
q1 [robot.rankInConfiguration ['left_w1']] = 0.4 # -up/down+
q1 [robot.rankInConfiguration ['right_s0']] = 0.84
q1 [robot.rankInConfiguration ['right_s1']] = 0.04
q1 [robot.rankInConfiguration ['right_e0']] = 0
q1 [robot.rankInConfiguration ['right_e1']] = -0.05
q1 [robot.rankInConfiguration ['right_w0']] = 0.18
q1 [robot.rankInConfiguration ['right_w1']] = 0.66
r(q1)
robot.isConfigValid(q1)

q2=q1[::]
q2 [robot.rankInConfiguration ['torso_t0']] = -1.57
q2 [robot.rankInConfiguration ['left_s0']] = -0.42
q2 [robot.rankInConfiguration ['left_s1']] = -0.41
q2 [robot.rankInConfiguration ['left_e0']] = -1.2
q2 [robot.rankInConfiguration ['left_e1']] = 0.62
q2 [robot.rankInConfiguration ['left_w0']] = -0.1
q2 [robot.rankInConfiguration ['left_w1']] = 0.6
q2 [robot.rankInConfiguration ['right_s0']] = 0.42
q2 [robot.rankInConfiguration ['right_s1']] = 0.56
q2 [robot.rankInConfiguration ['right_e0']] = 0
q2 [robot.rankInConfiguration ['right_e1']] = 0
q2 [robot.rankInConfiguration ['right_w0']] = 0.18
q2 [robot.rankInConfiguration ['right_w1']] = 0.49
r(q2)
robot.isConfigValid(q2)

ps.setInitialConfig (q1); ps.addGoalConfig (q2)


#ps.selectPathValidation ("Dichotomy", 0.)
#ps.selectPathPlanner ("VisibilityPrmPlanner")
#ps.saveRoadmap ('/local/mcampana/devel/hpp/data/baxter-in-room-PRM.rdm')
#ps.readRoadmap ('/local/mcampana/devel/hpp/data/baxter-in-room-RRT.rdm') # srand(50)
#ps.readRoadmap ('/local/mcampana/devel/hpp/data/baxter-in-room-PRM-dae.rdm') # srand :(
#ps.readRoadmap ('/local/mcampana/devel/hpp/data/baxter-in-room-RRT-dae.rdm')

ps.solve ()
ps.pathLength(0)
len(ps.getWaypoints (0))

ps.addPathOptimizer("Prune")
ps.optimizePath (0)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)
len(ps.getWaypoints (ps.numberPaths()-1))

ps.clearPathOptimizers()
#cl.problem.setAlphaInit (0.05)
ps.addPathOptimizer("GradientBased")
ps.optimizePath (0)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)
tGB = cl.problem.getTimeGB ()
timeValuesGB = cl.problem.getTimeValues ()
gainValuesGB = cl.problem.getGainValues ()
newGainValuesGB = ((1-np.array(gainValuesGB))*100).tolist()

ps.clearPathOptimizers()
ps.addPathOptimizer('RandomShortcut')
ps.optimizePath (0)
ps.pathLength(ps.numberPaths()-1)

ps.clearPathOptimizers()
ps.addPathOptimizer('PartialShortcut')
ps.optimizePath (0)
ps.pathLength(ps.numberPaths()-1)


pp(ps.numberPaths()-1)

r(ps.configAtParam(0,2))

## -------------------------------------
import matplotlib.pyplot as plt
from plotfunctions import optAndConcatenate, getValIndex, computeMeansVector, reducedVectors, curvPlot, curvSdPlot
# OPTIMIZE AND Concatenate RS PRS values:
globalTimeValuesRS = []; globalGainValuesRS = []
globalTimeValuesPRS = []; globalGainValuesPRS = []
nbOpt = 10 # number of launchs of RS and PRS
optAndConcatenate (cl, ps, 0, nbOpt, 'RandomShortcut', globalTimeValuesRS, globalGainValuesRS)
optAndConcatenate (cl, ps, 0, nbOpt, 'PartialShortcut', globalTimeValuesPRS, globalGainValuesPRS)

nbPoints = 100 # number of points in graph
tVec = np.arange(0,tGB,tGB/nbPoints)
moyVectorRS = []; sdVectorRS = []; moyVectorPRS = []; sdVectorPRS = [];
computeMeansVector (nbOpt, tVec, moyVectorRS, sdVectorRS, globalTimeValuesRS, globalGainValuesRS)
computeMeansVector (nbOpt, tVec, moyVectorPRS, sdVectorPRS, globalTimeValuesPRS, globalGainValuesPRS)

tReduceVectorRS = []; meanReduceVectorRS = []; sdReduceVectorRS = [];
tReduceVectorPRS = []; meanReduceVectorPRS = []; sdReduceVectorPRS = [];
reducedVectors (tVec, moyVectorRS, sdVectorRS, tReduceVectorRS, meanReduceVectorRS, sdReduceVectorRS)
reducedVectors (tVec, moyVectorPRS, sdVectorPRS, tReduceVectorPRS, meanReduceVectorPRS, sdReduceVectorPRS)

# Plot lengthGain (t);
plt.axis([-.1, tGB+0.1, 30, 103])
plt.xlabel('t (s)'); plt.ylabel('Relative remaining length (%)')
vectorLengthGB = len (timeValuesGB)
plt.plot([0,tGB], [newGainValuesGB[vectorLengthGB-1],newGainValuesGB[vectorLengthGB-1]], 'b--')
plt.plot(0, 100, 'bo'); plt.plot([0,timeValuesGB[0]], [100,100], 'b', linewidth=1.5)
plt = curvSdPlot (plt, tGB, tReduceVectorRS, meanReduceVectorRS, sdReduceVectorRS, '0.55', 0.8, 0.02)
plt = curvSdPlot (plt, tGB, tReduceVectorPRS, meanReduceVectorPRS, sdReduceVectorPRS, '0.55', 0.8, 0.02)
plt = curvPlot (plt, tGB, timeValuesGB, newGainValuesGB, 'o', 'b', 1.5)
plt.plot([0,tReduceVectorRS[0]], [100,100], 'r', linewidth=1.1)
plt = curvPlot (plt, tGB, tReduceVectorRS, meanReduceVectorRS, '*', 'r', 1.5)
plt.plot([0,tReduceVectorPRS[0]], [100,100], 'g', linewidth=0.8)
plt = curvPlot (plt, tGB, tReduceVectorPRS, meanReduceVectorPRS, '+', 'g', 1.5)
plt.show()

# For different alpha_init
tmax = max(max(tGB,tGB2),max(tGB3,tGB4))
plt.axis([-.3, tmax+0.3, 26, 102])
plt.xlabel('t (s)'); plt.ylabel('Relative remaining length (%)')
vectorLengthGB = len (timeValuesGB)
#plt.plot([0,tGB], [newGainValuesGB[vectorLengthGB-1],newGainValuesGB[vectorLengthGB-1]], 'b--')
plt.plot(0, 100, 'bo'); plt.plot([0,timeValuesGB[0]], [100,100], 'b', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB, newGainValuesGB, 'o', 'b', 1.5)
vectorLengthGB2 = len (timeValuesGB2)
plt.plot(0, 100, 'g*'); plt.plot([0,timeValuesGB2[0]], [100,100], 'g', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB2, newGainValuesGB2, '*', 'g', 1.5)
vectorLengthGB3 = len (timeValuesGB3)
plt.plot(0, 100, 'r+'); plt.plot([0,timeValuesGB3[0]], [100,100], 'r', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB3, newGainValuesGB3, '+', 'r', 1.5)
vectorLengthGB4 = len (timeValuesGB4)
plt.plot(0, 100, 'c+'); plt.plot([0,timeValuesGB4[0]], [100,100], 'c', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB4, newGainValuesGB4, '+', 'c', 1.5)
plt.show()

## -------------------------------------

ps.clearPathOptimizers(); ps.addPathOptimizer("GradientBased")
cl.problem.setAlphaInit (0.05)
ps.optimizePath (0); tGB2 = cl.problem.getTimeGB ()
timeValuesGB2 = cl.problem.getTimeValues (); gainValuesGB2 = cl.problem.getGainValues ()
newGainValuesGB2 = ((1-np.array(gainValuesGB2))*100).tolist() #percentage of initial length-value

cl.problem.setAlphaInit (0.3)
ps.optimizePath (0); tGB3 = cl.problem.getTimeGB ()
timeValuesGB3 = cl.problem.getTimeValues (); gainValuesGB3 = cl.problem.getGainValues ()
newGainValuesGB3 = ((1-np.array(gainValuesGB3))*100).tolist() #percentage of initial length-value

cl.problem.setAlphaInit (0.4) # PROBLEM WITH PRUNE
ps.optimizePath (0); tGB4 = cl.problem.getTimeGB ()
timeValuesGB4 = cl.problem.getTimeValues (); gainValuesGB4 = cl.problem.getGainValues ()
newGainValuesGB4 = ((1-np.array(gainValuesGB4))*100).tolist() #percentage of initial length-value

## -------------------------------------



# Add light to scene
lightName = "li"
r.client.gui.addLight (lightName, r.windowId, 0.005, [0.4,0.4,0.4,0.5])
r.client.gui.addToGroup (lightName, r.sceneName)
r.client.gui.applyConfiguration (lightName, [xBaxter,yBaxter,zBaxter+2,1,0,0,0])
#r.client.gui.applyConfiguration (lightName, [xBaxter+0.5,yBaxter+0.5,zBaxter+2,1,0,0,0])
r.client.gui.refresh ()


## Video recording
import time
pp.dt = 0.02
pp.speed=0.5
r(q1)
r.startCapture ("capture","png")
r(q1); time.sleep(0.2)
r(q1)
pp(0)
#pp(28)
r(q2); time.sleep(1);
r.stopCapture ()

## ffmpeg commands
ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4
x=0; for i in *png; do counter=$(printf %03d $x); ln "$i" new"$counter".png; x=$(($x+1)); done
ffmpeg -r 50 -i new%03d.png -r 25 -vcodec libx264 video.mp4
mencoder video.mp4 -channels 6 -ovc xvid -xvidencopts fixed_quant=4 -vf harddup -oac pcm -o video.avi
ffmpeg -i untitled.mp4 -vcodec libx264 -crf 24 video.mp4

## DEBUG commands
cl.obstacle.getObstaclePosition('decor_base')
robot.getJointOuterObjects('base_joint_xyz')
robot.isConfigValid(q1)
robot.distancesToCollision()
r( ps.configAtDistance(0,5) )
ps.optimizePath (0)
ps.clearRoadmap ()
ps.resetGoalConfigs ()
from numpy import *
argmin(robot.distancesToCollision()[0])
robot.getJointNames ()
robot.getConfigSize ()
robot.getLinkPosition('base_joint_xy')
robot.getJointNames ()

# To easily move stuff:
r.client.gui.applyConfiguration ("room/lamp_base", [0,0,0.04,1,0,0,0])
r.client.gui.refresh ()


## Debug Optimization Tools ##############
num_log = 26393
from parseLog import parseCollConstrPoints, parseNodes

collConstrNodes = parseNodes (num_log, '189: qFree_ = ')
collNodes = parseNodes (num_log, '182: qColl = ')

contactPoints = parseCollConstrPoints (num_log, '77: contact point = (')
x1_J1 = parseCollConstrPoints (num_log, '96: x1 in R0 = (')
x2_J1 = parseCollConstrPoints (num_log, '97: x2 in R0 = (')
x1_J2 = parseCollConstrPoints (num_log, '116: x1 in J2 = (')
x2_J2 = parseCollConstrPoints (num_log, '117: x2 in J2 = (') #x2_J2 <=> contactPoints


## same with viewer !
from viewer_display_library_OPTIM import transformInConfig, plotPoints, plotPointsAndLines, plot2DBaseCurvPath, plotDofCurvPath, plotPointBodyCurvPath
contactPointsViewer = transformInConfig (contactPoints)
x1_J1Viewer = transformInConfig (x1_J1)
x2_J1Viewer = transformInConfig (x2_J1)
x1_J2Viewer = transformInConfig (x1_J2)
x2_J2Viewer = transformInConfig (x2_J2)


sphereNamePrefix = "sphereContactPoints_"
plotPoints (r, sphereNamePrefix, contactPointsViewer, 0.02)
sphereSize=0.01
lineNamePrefix = "lineJ1_"; sphereNamePrefix = "sphereJ1_"
plotPointsAndLines (r, lineNamePrefix, sphereNamePrefix, x1_J1Viewer, x2_J1Viewer, sphereSize)
lineNamePrefix = "lineJ2_"; sphereNamePrefix = "sphereJ2_"
plotPointsAndLines (r, lineNamePrefix, sphereNamePrefix, x1_J2Viewer, x2_J2Viewer, sphereSize)

from viewer_display_library_OPTIM import plotPointBodyCurvPath
from hpp.corbaserver import Client
cl = robot.client
dt = 0.2
#plot2DBaseCurvPath(r, cl, dt, 0, "curvPath"+str(0), [1,0.3,0,1])
#plot2DBaseCurvPath (r, cl, dt, ps.numberPaths()-1, "curvPath"+str(ps.numberPaths()-1), [0,1,0.3,1])

jointName = 'left_w2'
plotPointBodyCurvPath (r, cl, robot, dt, 0, jointName, [0.13,0,0], 'pathPoint_'+jointName, [0.9,0.1,0.1,1])
plotPointBodyCurvPath (r, cl, robot, dt, 28, jointName, [0.13,0,0], 'pathPointRS_'+jointName, [0.1,0.1,0.9,1])
plotPointBodyCurvPath (r, cl, robot, dt, 27, jointName, [0.13,0,0], 'pathPointGB_'+jointName, [0.1,1,0.1,1])

jointName = 'right_w2'
plotPointBodyCurvPath (r, cl, robot, dt, 0, jointName, [0.13,0,0], 'pathPoint1_'+str(0)+jointName, [0.6,0.1,0.1,1])
plotPointBodyCurvPath (r, cl, robot, dt, 1, jointName, [0.13,0,0], 'pathPointRS1_'+str(1)+jointName, [0.1,0.1,0.6,1])
plotPointBodyCurvPath (r, cl, robot, dt, ps.numberPaths ()-1, jointName, [0,0,0], 'pathPointGB1_'+str(ps.numberPaths ()-1)+jointName, [0.2,0.6,0.2,1])

# test function in cl.robot
jointPosition = robot.getJointPosition ('left_w2')
pointInJoint = [0.13,0,0]
posAtester = cl.robot.computeGlobalPosition (jointPosition, pointInJoint)

r(q1)
robot.setCurrentConfig (q1)
sphereName = "machin"
r.client.gui.addSphere (sphereName,0.03,[0.1,0.1,0.1,1]) # black
configSphere = posAtester [::]
configSphere.extend ([1,0,0,0])
r.client.gui.applyConfiguration (sphereName,configSphere)
r.client.gui.addToGroup (sphereName, r.sceneName)
r.client.gui.refresh ()


