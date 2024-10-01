import time
import rowan
import numpy as np
import robotic as ry
import manipulation as manip
from shelf import generate_shelf, generate_target_box

C = ry.Config()
# C.addFile("./g_files/scenarios/just_hand.g")
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
gripper = "l_gripper"
table = "table"

# Shelf and target
pos = np.array([-.7, 0., 1.])
generate_shelf(C, pos, openings_small=[1, 1], just_front=True)
generate_target_box(C, [.0, .3, .8])

shelf_inside_pos = C.getFrame("small_box_inside_0_0_0").getPosition()

obj_count = 5
frame_names = []
for i in range(obj_count):
    cyl = np.random.choice([0, 1])
    cyl = np.random.choice([0])
    if cyl:
        frame_name = f"cylinder_{i}"
    else:
        frame_name = f"box_{i}"
    obj_frame = C.addFrame(frame_name)
    frame_names.append(frame_name)

    # pos
    rel_pos = np.random.uniform(-.1, .1, size=(3,))
    # rel_pos += np.array([0., 0., .2])
    obj_frame.setPosition(shelf_inside_pos + rel_pos)

    # shape
    if cyl:
        dims = np.random.uniform(.03, .06, size=(2,))
        obj_frame.setShape(ry.ST.cylinder, dims)
    else:
        dims = np.random.uniform(.03, .06, size=(3,))
        obj_frame.setShape(ry.ST.ssBox, [*dims, .005])

    # color
    obj_frame.setColor(np.random.uniform(0., 1., size=(3,)))

    # rot
    quat = rowan.from_euler(*np.random.uniform(0. ,2*np.pi, size=(3,)))
    obj_frame.setQuaternion(quat)

    # physics
    mass = np.random.uniform(.05, .5)
    obj_frame.setMass(mass)
    obj_frame.setContact(1)

C.view(False)

bot = ry.BotOp(C, False)
for _ in range(10):
    bot.sync(C)
    time.sleep(.01)

objs_inside = []
for i, frame_name in enumerate(frame_names):
    evaluation = C.eval(ry.FS.negDistance, [f"small_box_inside_0_0_0", frame_name])[0][0]
    if evaluation > .0:
        objs_inside.append(frame_name)

if not len(objs_inside):
    print("No objects where inside the shelf!")
    del bot
    del C
    exit()

M = manip.ManipulationModelling(C, "shelf_task", helpers=[gripper])
M.setup_pick_and_place_sequence(gripper, objs_inside, accumulated_collisions=False)

for i, obj in enumerate(objs_inside):
    t = i*2+1
    if "box" in obj:
        grasp_orientation = np.random.choice(["x", "y", "z"])
        M.grasp_box(t, gripper, obj, "l_palm", grasp_orientation)
    elif "cylinder" in obj:
        pass
    else:
        print("Object shape not supported!")
    
    M.komo.addObjective([], ry.FS.negDistance, ["l_palm", "target_box_left"], ry.OT.sos, [-.5])
    M.komo.addObjective([], ry.FS.negDistance, ["l_palm", "target_box_right"], ry.OT.sos, [-.5])
    M.komo.addObjective([], ry.FS.negDistance, ["l_palm", "target_box_front"], ry.OT.sos, [-.5])
    M.komo.addObjective([], ry.FS.negDistance, ["l_palm", "target_box_back"], ry.OT.sos, [-.5])

    M.komo.addObjective([t+1], ry.FS.negDistance, ["target_box_inside", obj], ry.OT.ineq, [-1e1], 0)
    M.komo.addObjective([t+1], ry.FS.vectorZ, [gripper], ry.OT.eq, [1e1], [0, 0, 1])
    M.target_relative_xy_position(t+1, obj, "target_box_inside", [0, 0])
    M.keep_distance([], "l_palm", "target_box_left")
    M.keep_distance([], "l_palm", "target_box_right")
    M.keep_distance([], "l_palm", "target_box_front")
    M.keep_distance([], "l_palm", "target_box_back")


keypoints = M.solve()
M.komo.report(plotOverTime=True)

if M.feasible:
    for i, k in enumerate(keypoints):
        C.setJointState(k)
        if i%2:
            C.attach(table, objs_inside[i//2])
        else:
            C.attach(gripper, objs_inside[i//2])
        C.view(False, f'step {i}')
        time.sleep(1.)

C.view(False)
