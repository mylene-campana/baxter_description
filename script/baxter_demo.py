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

lightName = "li1"
r.client.gui.addLight (lightName, r.windowId, 0.005, [0.5,0.5,0.5,0.5])
r.client.gui.addToGroup (lightName, r.sceneName)
r.client.gui.applyConfiguration (lightName, [0,0,3,1,0,0,0])
r.client.gui.refresh ()


q1 = robot.getCurrentConfig ();
q1 [robot.rankInConfiguration ['torso_t0']] = 0
q1 [robot.rankInConfiguration ['head_pan']] = 0.2
q1 [robot.rankInConfiguration ['left_s0']] = -0.9 # turn +left/right-
q1 [robot.rankInConfiguration ['left_s1']] = -0.32 # +up/down-
q1 [robot.rankInConfiguration ['left_e0']] = 0
q1 [robot.rankInConfiguration ['left_e1']] = 0.68
q1 [robot.rankInConfiguration ['left_w0']] = 0
q1 [robot.rankInConfiguration ['left_w1']] = 0.3 # -up/down+
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


ps.solve ()
ps.pathLength(0)
len(ps.getWaypoints (0))
pp(0)




ps.clearPathOptimizers()
ps.addPathOptimizer('RandomShortcut')
ps.optimizePath (0)
ps.pathLength(ps.numberPaths()-1)
pp(ps.numberPaths()-1)




ps.clearPathOptimizers()
#cl.problem.setAlphaInit (0.05)
ps.addPathOptimizer("GradientBased")
ps.optimizePath (0)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)
pp(ps.numberPaths()-1)
