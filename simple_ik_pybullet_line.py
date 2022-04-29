import HarfangHighLevel as hl
import pybullet as physics
from math import sin, cos, pi
import time

# init window
hl.Init(1024, 1024)
hl.AddFpsCamera(0.3, 0.3, -3)

physics.connect(physics.DIRECT)

bodyId = physics.loadURDF("3d_models/TwoJointRobot.urdf", useFixedBase=True)
numJoints = physics.getNumJoints(bodyId)

vec_target = hl.Vec3(0.2, 0.3, -0.35)

while not hl.UpdateDraw():
    physics.stepSimulation()

    if hl.ImGuiBegin("Target"):
        vec_target = hl.Vec3(cos(time.time()) * 0.5, 0.5 + sin(time.time()) * 0.2, 0)
        change, vec_target = hl.ImGuiSliderVec3("Target right hand", vec_target, -1, 1.0)
    hl.ImGuiEnd()

    # draw target
    hl.DrawCrossV(vec_target, hl.Color.Purple)

    # draw model
    prev = None
    for j in range(numJoints):
        link_state = physics.getLinkState(bodyId, j)

        p = hl.Vec3(link_state[0][0], link_state[0][1], link_state[0][2])
        hl.DrawCrossV(p, hl.Color.Red, 0.4)

        if prev is not None:
            hl.DrawLineV(p, prev)
        prev = p

    # ik
    target_right = [vec_target.x, vec_target.y, vec_target.z]
    jointPoses = physics.calculateInverseKinematics(bodyId, 5, target_right)

    # update joint on model
    for i in range(numJoints):
        jointInfo = physics.getJointInfo(bodyId, i)
        qIndex = jointInfo[3]
        if qIndex > -1:
            physics.setJointMotorControl2(bodyIndex=bodyId, jointIndex=i, controlMode=physics.POSITION_CONTROL, targetPosition=jointPoses[qIndex - 7])

