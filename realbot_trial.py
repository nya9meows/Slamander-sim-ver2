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
logging_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "./video/keyboard3.mp4")

# add resource path
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
# add gravity
p.setGravity(0, 0, -10)
# add urdf models and set initial position
# set the center of mass frame (loadURDF sets base link frame)
# startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 0.2]
startOrientation = p.getQuaternionFromEuler([0, 0, math.radians(180)])
# boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
# boxId = p.loadURDF("./3D_ver5/urdf/3D_ver5.urdf",startPos, startOrientation)
boxId = p.loadURDF("./segl_ver3/urdf/segl_ver3.urdf", startPos, startOrientation)

# 获取轮子的关节索引
available_joints_indexes = [i for i in range(p.getNumJoints(boxId)) if p.getJointInfo(boxId, i)[2] != p.JOINT_FIXED]
# wheel_joints_indexes = [i for i in available_joints_indexes if "wheel" in str(p.getJointInfo(boxId, i)[1])]
wheel_joints_indexes = [1, 3, 5, 13, 15, 17]
suspension_joints_indexes = [0, 2, 4, 12, 14, 16]
# wheel_joints_indexes = [0, 1]
# bend_joints_indexes = [i for i in available_joints_indexes if "bend" in str(p.getJointInfo(boxId, i)[1])]
bend_joints_indexes = [8]
raise_joints_indexes = [9]
prismatic_joints_indexes = [7, 10]
# control parameter of the joints
target_v = 0  # wheel velocity set
max_force = 1000  # wheel force
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
# set camera
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
p.resetDebugVisualizerCamera(
    cameraDistance=0.1,
    cameraYaw=60,
    cameraPitch=-45,
    cameraTargetPosition=cubePos
)

# realtime simulation
p.setRealTimeSimulation(1)

while True:
    p.stepSimulation()
    key_dict = p.getKeyboardEvents()

    if len(key_dict):
        if p.B3G_UP_ARROW in key_dict:
            target_v += speed_step
            target_bend_v = 0
            print("up KEY_WAS_TRIGGERED")
            text_word = "forward"

        elif p.B3G_DOWN_ARROW in key_dict:
            target_v -= speed_step
            target_bend_v = 0
            print("down KEY_WAS_TRIGGERED")
            text_word = "backward"

        elif p.B3G_LEFT_ARROW in key_dict:
            target_bend_v -= bend_step
            print("left KEY_WAS_TRIGGERED")
            text_word = "L-bend"

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
        # continue

    p.setJointMotorControlArray(
        bodyUniqueId=boxId,
        jointIndices=wheel_joints_indexes,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=[target_v for _ in wheel_joints_indexes],
        # targetVelocities=[target_v, target_v],
        forces=[max_force for _ in wheel_joints_indexes]
    )

    p.setJointMotorControlArray(
        bodyUniqueId=boxId,
        jointIndices=bend_joints_indexes,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=[target_bend_v for _ in bend_joints_indexes],
        forces=[max_force for _ in bend_joints_indexes]
    )

    p.setJointMotorControlArray(
        bodyUniqueId=boxId,
        jointIndices=raise_joints_indexes,
        controlMode=p.POSITION_CONTROL,
        targetVelocities=[target_raise_v for _ in raise_joints_indexes],
        forces=[max_force for _ in raise_joints_indexes]
    )

    p.setJointMotorControlArray(
        bodyUniqueId=boxId,
        jointIndices=prismatic_joints_indexes,
        controlMode=p.POSITION_CONTROL,
        targetVelocities=[target_slide for _ in prismatic_joints_indexes],
        forces=[max_force for _ in prismatic_joints_indexes]
    )

    p.setJointMotorControlArray(
        bodyUniqueId=boxId,
        jointIndices=suspension_joints_indexes,
        controlMode=p.POSITION_CONTROL,
        targetVelocities=[target_slide for _ in suspension_joints_indexes],
        forces=[max_force for _ in suspension_joints_indexes]
    )
    # set camera
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    p.resetDebugVisualizerCamera(
        cameraDistance=0.5,
        cameraYaw=60,
        cameraPitch=-60,
        cameraTargetPosition=cubePos
    )
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
