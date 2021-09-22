# # This is a sample Python script.
#
# # Press Shift+F10 to execute it or replace it with your code.
# # Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
#
#
# def print_hi(name):

#     # Use a breakpoint in the code line below to debug your script.
#     print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.
#
#
# # Press the green button in the gutter to run the script.
# if __name__ == '__main__':
#     print_hi('PyCharm')
#
# # See PyCharm help at https://www.jetbrains.com/help/pycharm/

# This is a sample Python script.
#
# # Press Shift+F10 to execute it or replace it with your code.
# # Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
#
#
# def print_hi(name):
#     # Use a breakpoint in the code line below to debug your script.
#     print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.
#
#
# # Press the green button in the gutter to run the script.
# if __name__ == '__main__':
#     print_hi('PyCharm')
#
# # See PyCharm help at https://www.jetbrains.com/help/pycharm/
import pybullet as p
import time
import pybullet_data
import math

# connect to physics engine
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
# render
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 1)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

# video
logging_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "./video/withdiffspeed.mp4")

# add resource path
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
# add gravity
p.setGravity(0, 0, -10)
# add urdf models and set initial position
# set the center of mass frame (loadURDF sets base link frame)
# startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
planeId = p.loadURDF("plane.urdf")
numofmodules = 3
listboxId = []
listcid = []
for i in range(numofmodules):
    print(f"i={i}")
    startPos = [i*0.25, 0, 0.1]
    startOrientation = p.getQuaternionFromEuler([0, 0, math.radians(180)])
    startOrientationf = p.getQuaternionFromEuler([0, 0, math.radians(180)])

    # boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
    # boxId = p.loadURDF("./3D_ver5/urdf/3D_ver5.urdf", startPos, startOrientation)
    # boxId = p.loadURDF("./segment_follow_ver1/urdf/segment_follow_ver1.urdf", startPos, startOrientation)
    if i == 0:
        boxId = p.loadURDF("./segl_ver3/urdf/segl_ver3.urdf", startPos, startOrientation)
    else:
        boxId = p.loadURDF("./segf_ver5/urdf/segf_ver5.urdf", startPos, startOrientationf)


    listboxId.append(boxId)
    if i > 0:
        cid = p.createConstraint(listboxId[i-1], -1, listboxId[i], 10, p.JOINT_FIXED, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                                 [0, 0, 0],
                                 [0, 0, 0])
        listcid.append(cid)

        pivot = [0, 0, 0]
        halfpi = math.pi / 2.0
        orn = p.getQuaternionFromEuler([-math.radians(33), 0, 0])
        p.changeConstraint(listcid[i-1], pivot, jointChildFrameOrientation=orn, maxForce=5)

# wheel index for lead
wheel_joints_indexes_l = [1, 3, 5, 13, 15, 17]
suspension_joints_indexes_l = [0, 2, 4, 12, 14, 16]
bend_joints_indexes_l = [8]
raise_joints_indexes_l = [9]
prismatic_joints_indexes_l = [7, 10]

# wheel index for follower
wheel_joints_indexes_f = [1, 3, 5]
suspension_joints_indexes_f = [0, 2, 4]
bend_joints_indexes_f = [8]
raise_joints_indexes_f = [9]
prismatic_joints_indexes_f = [7, 10]

# control parameter of the joints
target_v = 0  # wheel velocity set
max_force = 100  # wheel force
target_bend_v = 0
target_raise_v = 0
target_slide = 0

speed_step = 0.8
bend_step = 0.05
raise_step = 0.20

textColor = [1, 1, 0]

# blank text
text_word = "start"
debug_text_id = p.addUserDebugText(
    text=text_word,
    textPosition=[0, 0, 2],
    textColorRGB=textColor,
    textSize=2.5
)
# # set camera
# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
# p.resetDebugVisualizerCamera(
#     cameraDistance=1.0,
#     cameraYaw=60,
#     cameraPitch=-45,
#     cameraTargetPosition=cubePos
# )

# realtime simulation
p.setRealTimeSimulation(1)

bend_number = 0 
left_turn = False
bend_times = 0
current_bending = 0
reset = True

while True:
    p.stepSimulation()
    key_dict = p.getKeyboardEvents()

    if len(key_dict):
        reset = False
        if p.B3G_UP_ARROW in key_dict and p.B3G_LEFT_ARROW in key_dict:
            target_v += speed_step
            target_bend_v = -0.3
            print("turn KEY_WAS_TRIGGERED")
            text_word = "turn"
            left_turn = True
            bend_times += 1
            if bend_times >= 15:
                current_bending += 1
                if current_bending >= numofmodules:
                    current_bending = 0
                print("bending:")
                print(current_bending)

        elif p.B3G_UP_ARROW in key_dict:
            target_v += speed_step
            target_bend_v = 0
            print("up KEY_WAS_TRIGGERED")
            text_word = "forward"

        # elif p.B3G_DOWN_ARROW in key_dict:
        #     target_v -= speed_step
        #     target_bend_v = 0
        #     print("down KEY_WAS_TRIGGERED")
        #     text_word = "backward"

        # elif p.B3G_LEFT_ARROW in key_dict:
        #     target_bend_v -= bend_step
        #     print("left KEY_WAS_TRIGGERED")
        #     text_word = "L-bend"

        elif p.B3G_RIGHT_ARROW in key_dict:
            target_bend_v += bend_step
            print("right KEY_WAS_TRIGGERED")
            text_word = "R-bend"

        elif p.B3G_SPACE in key_dict:
            target_raise_v += raise_step
            print("space KEY_WAS_TRIGGERED")
            text_word = "raise"

        elif p.B3G_ALT in key_dict:
            target_raise_v -= raise_step
            print("alt KEY_WAS_TRIGGERED")
            text_word = "yield"

        elif p.B3G_RETURN in key_dict:
            print("abort KEY_WAS_TRIGGERED")
            text_word = "abort"
            break
        else:
            continue

    else:
        target_v = 0
        target_bend_v = 0
        text_word = ""
        left_turn = False
        bend_times = 0
        current_bending = 0
        reset = True
        # continue

    for i in range(numofmodules):
        # if len(key_dict) and i == 1:
        #     target_bend_v = 5
        #     max_force = 100
        # # elif len(key_dict) and i == 0:
        # #     target_bend_v = -1
        # else:
        #     target_bend_v = 0
        #     max_force = 0
        if i == 0:
            wheel_joints_indexes = wheel_joints_indexes_l
            suspension_joints_indexes = suspension_joints_indexes_l
            bend_joints_indexes = bend_joints_indexes_l
            raise_joints_indexes = raise_joints_indexes_l
            prismatic_joints_indexes = prismatic_joints_indexes_l
            wheel_joints_indexes_leftwheel = [3, 13]
            wheel_joints_indexes_rightwheel = [1, 11]

        else:
            wheel_joints_indexes = wheel_joints_indexes_f
            suspension_joints_indexes = suspension_joints_indexes_f
            bend_joints_indexes = bend_joints_indexes_f
            raise_joints_indexes = raise_joints_indexes_f
            prismatic_joints_indexes = prismatic_joints_indexes_f
            wheel_joints_indexes_leftwheel = [3]
            wheel_joints_indexes_rightwheel = [1]


        # if i == current_bending and left_turn:
        if left_turn==True and i == current_bending:
                p.setJointMotorControlArray(
                bodyUniqueId=listboxId[i],
                jointIndices=wheel_joints_indexes_leftwheel,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[target_v for _ in wheel_joints_indexes_leftwheel],
                # targetVelocities=[target_v, target_v],
                forces=[max_force for _ in wheel_joints_indexes_leftwheel]
                #     forces=[0 for _ in wheel_joints_indexes_leftwheel]
                )

                p.setJointMotorControlArray(
                bodyUniqueId=listboxId[i],
                jointIndices=wheel_joints_indexes_rightwheel,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[-1*target_v for _ in wheel_joints_indexes_rightwheel],
                # targetVelocities=[target_v, target_v],
                forces=[max_force for _ in wheel_joints_indexes_rightwheel]
                #     forces=[0 for _ in wheel_joints_indexes_rightwheel]
                )
                # p.setJointMotorControl2(
                # bodyUniqueId=listboxId[i],
                # jointIndices=wheel_joints_indexes,
                # controlMode=p.VELOCITY_CONTROL,
                # targetVelocities=[target_v for _ in wheel_joints_indexes_leftwheel],
                # # targetVelocities=[target_v, target_v],
                # forces=[max_force for _ in wheel_joints_indexes_leftwheel]
                # )

                # p.setJointMotorControl2(
                # bodyUniqueId=listboxId[i],
                # jointIndices=wheel_joints_indexes,
                # controlMode=p.VELOCITY_CONTROL,
                # targetVelocities=[-1*target_v for _ in wheel_joints_indexes_rightwheel],
                # # targetVelocities=[target_v, target_v],
                # forces=[max_force for _ in wheel_joints_indexes_rightwheel]
                # )




                p.setJointMotorControlArray(
                    bodyUniqueId=listboxId[i],
                    jointIndices=bend_joints_indexes,
                    controlMode=p.POSITION_CONTROL,
                    targetVelocities=[-5 for _ in bend_joints_indexes],
                    # targetPositions=30,forces=[max_force for _ in wheel_joints_indexes_rightwheel]
                    targetPositions=[3.14/4 for _ in bend_joints_indexes],
                    forces=[max_force for _ in bend_joints_indexes]
                )


        elif left_turn==True:
                p.setJointMotorControlArray(
                bodyUniqueId=listboxId[i],
                jointIndices=wheel_joints_indexes,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[target_v for _ in wheel_joints_indexes],
                # targetVelocities=[target_v, target_v],
                forces=[max_force for _ in wheel_joints_indexes]
                )
                p.setJointMotorControlArray(
                    bodyUniqueId=listboxId[i],
                    jointIndices=bend_joints_indexes,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocities=[0 for _ in bend_joints_indexes],
                    forces=[max_force for _ in bend_joints_indexes]
                )


        else:
            p.setJointMotorControlArray(
                bodyUniqueId=listboxId[i],
                jointIndices=wheel_joints_indexes,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[target_v for _ in wheel_joints_indexes],
                # targetVelocities=[target_v, target_v],
                forces=[max_force for _ in wheel_joints_indexes]
            )

            p.setJointMotorControlArray(
                bodyUniqueId=listboxId[i],
                jointIndices=bend_joints_indexes,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[target_bend_v for _ in bend_joints_indexes],
                forces=[max_force for _ in bend_joints_indexes]
            )

            p.setJointMotorControlArray(
                bodyUniqueId=listboxId[i],
                jointIndices=raise_joints_indexes,
                controlMode=p.POSITION_CONTROL,
                targetVelocities=[target_raise_v for _ in raise_joints_indexes],
                forces=[max_force for _ in raise_joints_indexes]
            )

            p.setJointMotorControlArray(
                bodyUniqueId=listboxId[i],
                jointIndices=prismatic_joints_indexes,
                controlMode=p.POSITION_CONTROL,
                targetVelocities=[target_slide for _ in prismatic_joints_indexes],
                forces=[max_force for _ in prismatic_joints_indexes]
            )
            p.setJointMotorControlArray(
                bodyUniqueId=listboxId[i],
                jointIndices=suspension_joints_indexes,
                controlMode=p.POSITION_CONTROL,
                targetVelocities=[target_slide for _ in suspension_joints_indexes],
                forces=[max_force for _ in suspension_joints_indexes]
            )

    # if reset == True:

    # # set camera
    # cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    # p.resetDebugVisualizerCamera(
    #     cameraDistance=0.5,
    #     cameraYaw=60,
    #     cameraPitch=-60,
    #     cameraTargetPosition=cubePos
    # )
    debug_text_id = p.addUserDebugText(
        text=text_word,
        textPosition=[0, 0, 1],
        # textPosition=cubePos,
        textColorRGB=textColor,
        textSize=2.5,
        replaceItemUniqueId=debug_text_id
    )

p.stopStateLogging(logging_id)

# # time delay step simulation
# for i in range(1000):
#     p.stepSimulation()
#
#
#
#     p.setJointMotorControlArray(
#         bodyUniqueId=boxId,
#         jointIndices=wheel_joints_indexes,
#         controlMode=p.VELOCITY_CONTROL,
#         targetVelocities=[target_v for _ in wheel_joints_indexes],
#         forces=[max_force for _ in wheel_joints_indexes]
#     )
#     p.setJointMotorControlArray(
#         bodyUniqueId=boxId,
#         jointIndices=bend_joints_indexes,
#         controlMode=p.VELOCITY_CONTROL,
#         targetVelocities=[0*math.sin(i) for _ in bend_joints_indexes],
#         forces=[max_force for _ in bend_joints_indexes]
#     )
#     # set camera
#     cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
#     p.resetDebugVisualizerCamera(
#         cameraDistance=0.5,
#         cameraYaw=60,
#         cameraPitch=-30,
#         cameraTargetPosition=cubePos
#     )
#
#     time.sleep(1./240.)

# # display joint info
# joint_num = p.getNumJoints(boxId)
# print(f"num of joint of r2d2: {joint_num}")
#
# print("joint info: ")
# for joint_index in range(joint_num):
#     info_tuple = p.getJointInfo(boxId,joint_index)
#     print(f"joint_index：{info_tuple[0]}\n\
#             joint_name：{info_tuple[1]}\n\
#             joint_type：{info_tuple[2]}\n\
#             机器人第一个位置的变量索引：{info_tuple[3]}\n\
#             机器人第一个速度的变量索引：{info_tuple[4]}\n\
#             保留参数：{info_tuple[5]}\n\
#             joint_damping：{info_tuple[6]}\n\
#             joint_friction_coefficient：{info_tuple[7]}\n\
#             slider和revolute(hinge)类型的位移最小值：{info_tuple[8]}\n\
#             slider和revolute(hinge)类型的位移最大值：{info_tuple[9]}\n\
#             关节驱动的最大值：{info_tuple[10]}\n\
#             关节的最大速度：{info_tuple[11]}\n\
#             node_name：{info_tuple[12]}\n\
#             局部框架中的关节轴系：{info_tuple[13]}\n\
#             父节点frame的关节位置：{info_tuple[14]}\n\
#             父节点frame的关节方向：{info_tuple[15]}\n\
#             父节点的索引，若是基座返回-1：{info_tuple[16]}\n\n")

# disconnect the engine
p.disconnect(physicsClient)
