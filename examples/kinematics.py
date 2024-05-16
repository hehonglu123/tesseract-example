from tesseract_robotics.tesseract_common import FilesystemPath, GeneralResourceLocator, Isometry3d, Translation3d, \
    TransformMap, Quaterniond
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_kinematics import KinGroupIKInput, KinGroupIKInputs
import numpy as np
from tesseract_env.gazebo_model_resource_locator import GazeboModelResourceLocator


locator = GeneralResourceLocator()
env = Environment()
with open("src/tesseract_env/config/cell.urdf",'r') as f:
    combined_urdf = f.read()
with open("src/tesseract_env/config/cell.srdf",'r') as f:
    combined_srdf = f.read()

assert env.init(combined_urdf, combined_srdf, GazeboModelResourceLocator())

robot_joint_names = ["MA2010_joint_1_s","MA2010_joint_2_l","MA2010_joint_3_u","MA2010_joint_4_r","MA2010_joint_5_b","MA2010_joint_6_t"]

# Get the kinematics solver. The name "manipulator" is specified in the SRDF file
kin_group = env.getKinematicGroup("MA2010")

# Solve forward kinematics at a specific joint position
robot_joint_pos = np.deg2rad(np.array([10, 20,-5, 70, 30, 90], dtype=np.float64))
fwdkin_result = kin_group.calcFwdKin(robot_joint_pos)
#fwdkin_result is a TransformMap, which is a dictionary of link names to Isometry3d. For this robot, we are
#interested in the transform of the "tool0" link
tool0_transform = fwdkin_result["MA2010_tool0"]
# Print the transform as a translation and quaternion
print("Tool0 transform at joint position " + str(robot_joint_pos) + " is: ")
q = Quaterniond(tool0_transform.rotation())
print("Translation: " + str(tool0_transform.translation().flatten()))
print(f"Rotation: {q.w()} {q.x()} {q.y()} {q.z()}")

# Solve inverse kinematics at a specific tool0 pose
tool0_transform2 = Isometry3d.Identity() * Translation3d(0.7, -0.1, 1) * Quaterniond(0.70711, 0, 0.7171, 0)

# Create a KinGroupIKInput and KinGroupIKInputs object. The KinGroupIKInputs object is a list of KinGroupIKInput
ik = KinGroupIKInput()
ik.pose = tool0_transform2
ik.tip_link_name = "MA2010_tool0"
ik.working_frame = "MA2010_base_link"
iks = KinGroupIKInputs()
iks.append(ik)
# Solve IK
ik_result = kin_group.calcInvKin(iks, robot_joint_pos)
# Print the result
print(f"Found {len(ik_result)} solutions")
for i in range(len(ik_result)):
    print("Solution " + str(i) + ": " + str(ik_result[i].flatten()))




   