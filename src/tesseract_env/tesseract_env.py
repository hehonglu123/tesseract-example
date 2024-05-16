#!/usr/bin/env python3
from tesseract_robotics.tesseract_environment import Environment, ChangeJointOriginCommand, MoveJointCommand
from tesseract_robotics import tesseract_geometry
from tesseract_robotics.tesseract_common import Isometry3d, CollisionMarginData, Translation3d, Quaterniond, \
	ManipulatorInfo
from tesseract_robotics.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond, \
    ManipulatorInfo, AnyPoly, AnyPoly_wrap_double
from tesseract_robotics.tesseract_command_language import CartesianWaypoint, WaypointPoly, \
    MoveInstructionType_FREESPACE, MoveInstruction, InstructionPoly, StateWaypoint, StateWaypointPoly, \
    CompositeInstruction, MoveInstructionPoly, CartesianWaypointPoly, ProfileDictionary, \
        AnyPoly_as_CompositeInstruction, CompositeInstructionOrder_ORDERED, DEFAULT_PROFILE_KEY, \
        AnyPoly_wrap_CompositeInstruction, DEFAULT_PROFILE_KEY, JointWaypoint, JointWaypointPoly, \
        InstructionPoly_as_MoveInstructionPoly, WaypointPoly_as_StateWaypointPoly, \
        MoveInstructionPoly_wrap_MoveInstruction, StateWaypointPoly_wrap_StateWaypoint, \
        CartesianWaypointPoly_wrap_CartesianWaypoint, JointWaypointPoly_wrap_JointWaypoint
from tesseract_robotics.tesseract_task_composer import TaskComposerPluginFactory, PlanningTaskComposerProblem, \
    TaskComposerDataStorage, TaskComposerContext

from tesseract_robotics import tesseract_collision
from tesseract_robotics_viewer import TesseractViewer

import json, pkg_resources
import numpy as np

from tesseract_env.gazebo_model_resource_locator import GazeboModelResourceLocator

class Tess_Env(object):
	def __init__(self,config_path,robot_linkname,robot_jointname):
		self.robot_linkname=robot_linkname
		self.robot_jointname=robot_jointname

		self.config_path = pkg_resources.resource_filename('tesseract_env', config_path)		#path to urdf and srdf files
		######tesseract environment setup:
		with open(self.config_path+'cell.urdf','r') as f:
			combined_urdf = f.read()
		with open(self.config_path+'cell.srdf','r') as f:
			combined_srdf = f.read()

		self.t_env= Environment()
		self.t_env.init(combined_urdf, combined_srdf, GazeboModelResourceLocator())
		self.scene_graph=self.t_env.getSceneGraph()


		##############################################Tesseract reports all GJK/EPA distance within contact_distance threshold##############################################
		contact_distance=0.1
		monitored_link_names = self.t_env.getLinkNames()
		self.manager = self.t_env.getDiscreteContactManager()
		self.manager.setActiveCollisionObjects(monitored_link_names)
		self.manager.setCollisionMarginData(CollisionMarginData(contact_distance))

		##############################################Tesseract planner setup##############################################
		print(self.config_path+'task_composer_plugins.yaml')
		self.factory = TaskComposerPluginFactory(FilesystemPath(self.config_path+'task_composer_plugins.yaml'))
		# Create the task composer node. In this case the FreespacePipeline is used. Many other are available.
		self.task = self.factory.createTaskComposerNode("FreespacePipeline")

		# Get the output keys for the task
		self.output_key = self.task.getOutputKeys()[0]

		# Create a profile dictionary. Profiles can be customized by adding to this dictionary and setting the profiles
		# in the instructions.
		self.profiles = ProfileDictionary()

		#######viewer setup, for URDF setup verification in browser @ localhost:8000/#########################
		self.viewer = TesseractViewer()

		self.viewer.update_environment(self.t_env, [0,0,0])

		self.viewer.start_serve_background()

	def update_pose(self,model_name,H):
		###update model pose in tesseract environment
		cmd = ChangeJointOriginCommand(model_name+'_pose', Isometry3d(H))
		self.t_env.applyCommand(cmd)
		#refresh
		self.viewer.update_environment(self.t_env)

	def attach_part(self,model_name,link_name,H=np.eye(4)):
		cmd = MoveJointCommand(model_name+'_pose', link_name)
		self.t_env.applyCommand(cmd)
		cmd = ChangeJointOriginCommand(model_name+'_pose', H)
		self.t_env.applyCommand(cmd)
		#refresh
		self.viewer.update_environment(self.t_env)

	def check_collision(self,robot_names,q):
		###check collision for a single robot, including self collision and collision with part
		for i,robot_name in enumerate(robot_names):
			self.t_env.setState(self.robot_jointname[robot_name], q[i])
		env_state = self.t_env.getState()
		self.manager.setCollisionObjectsTransform(env_state.link_transforms)

		result = tesseract_collision.ContactResultMap()
		self.manager.contactTest(result, tesseract_collision.ContactRequest(tesseract_collision.ContactTestType_ALL))
		result_vector = tesseract_collision.ContactResultVector()
		result.flattenMoveResults(result_vector)

		###iterate all collision instances
		for robot_name in robot_names:
			for c in result_vector: 
				if (c.link_names[0] in self.robot_linkname[robot_name] or c.link_names[1] in self.robot_linkname[robot_name]):
					print(c.link_names[0], c.link_names[1], c.distance)
					return True

		return False

	def plan_js(self,robot_name,q_cur,q_goal):
		###plan a trajectory for a single robot
		manip_info = ManipulatorInfo()
		manip_info.tcp_frame = "MA2010_tool0"
		manip_info.manipulator = "MA2010"
		manip_info.working_frame = "MA2010_base_link"

		# Set the initial state of the robot
		joint_names = self.robot_jointname[robot_name]
		self.viewer.update_joint_positions(joint_names, q_cur)

		# Set the initial state of the robot
		self.t_env.setState(joint_names, q_cur)

		# Create the input command program waypoints
		wp1 = JointWaypoint(joint_names,q_cur)
		wp2 = JointWaypoint(joint_names,q_goal)
		# wp3 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8,0.5,1.455) * Quaterniond(0.70710678,0,0.70710678,0))

		# Create the input command program instructions. Note the use of explicit construction of the CartesianWaypointPoly
		# using the *_wrap_CartesianWaypoint functions. This is required because the Python bindings do not support implicit
		# conversion from the CartesianWaypoint to the CartesianWaypointPoly.
		start_instruction = MoveInstruction(JointWaypointPoly_wrap_JointWaypoint(wp1), MoveInstructionType_FREESPACE, "DEFAULT")
		plan_f1 = MoveInstruction(JointWaypointPoly_wrap_JointWaypoint(wp2), MoveInstructionType_FREESPACE, "DEFAULT")
		# plan_f2 = MoveInstruction(CartesianWaypointPoly_wrap_CartesianWaypoint(wp3), MoveInstructionType_FREESPACE, "DEFAULT")

		# Create the input command program. Note the use of *_wrap_MoveInstruction functions. This is required because the
		# Python bindings do not support implicit conversion from the MoveInstruction to the MoveInstructionPoly.
		program = CompositeInstruction("DEFAULT")
		program.setManipulatorInfo(manip_info)
		program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
		program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f1))
		# program.appendMoveInstruction(MoveInstructionPoly(plan_f2))


		# Create an AnyPoly containing the program. This explicit step is required because the Python bindings do not
		# support implicit conversion from the CompositeInstruction to the AnyPoly.
		program_anypoly = AnyPoly_wrap_CompositeInstruction(program)

		# Create the task problem and input
		task_planning_problem = PlanningTaskComposerProblem(self.t_env, self.profiles)
		task_planning_problem.input = program_anypoly

		# Create an executor to run the task
		task_executor = self.factory.createTaskComposerExecutor("TaskflowExecutor")

		# Run the task and wait for completion
		future = task_executor.run(self.task.get(), task_planning_problem)
		future.wait()

		# Retrieve the output, converting the AnyPoly back to a CompositeInstruction
		results = AnyPoly_as_CompositeInstruction(future.context.data_storage.getData(self.output_key))

		# Display the output
		# Print out the resulting waypoints
		for instr in results:
			assert instr.isMoveInstruction()
			move_instr1 = InstructionPoly_as_MoveInstructionPoly(instr)
			wp1 = move_instr1.getWaypoint()
			assert wp1.isStateWaypoint()
		
		self.viewer.update_trajectory(results)
		self.viewer.plot_trajectory(results, manip_info)

	#######################################update joint angles in Tesseract Viewer###########################################
	def viewer_joints_update(self,robot_names,joints):
		joint_names=[]
		for robot_name in robot_names:
			joint_names.extend(self.robot_jointname[robot_name])

		self.viewer.update_joint_positions(joint_names, np.array(joints))

	def viewer_trajectory(self,robot_names,curve_js,timestamp=[]):
		joint_names=[]
		for robot_name in robot_names:
			joint_names.extend(self.robot_jointname[robot_name])
		
		#default time interpolation 5s
		if len(timestamp)==0:
			timestamp=np.linspace(0,5,len(curve_js))
		self.viewer.update_trajectory_list(joint_names, np.hstack((curve_js,timestamp[:,np.newaxis])).tolist())











