#!/usr/bin/env python3
from tesseract_robotics.tesseract_environment import Environment, ChangeJointOriginCommand, MoveJointCommand
from tesseract_robotics import tesseract_geometry
from tesseract_robotics.tesseract_common import Isometry3d, CollisionMarginData, Translation3d, Quaterniond, \
	ManipulatorInfo
from tesseract_robotics import tesseract_collision
from tesseract_robotics_viewer import TesseractViewer

import json, pkg_resources
import numpy as np

from tesseract_env.gazebo_model_resource_locator import GazeboModelResourceLocator

class Tess_Env(object):
	def __init__(self,urdf_path,robot_linkname,robot_jointname):
		self.robot_linkname=robot_linkname
		self.robot_jointname=robot_jointname

		urdf_path = pkg_resources.resource_filename('tesseract_env', urdf_path)		#path to urdf and srdf files
		######tesseract environment setup:
		with open(urdf_path+'.urdf','r') as f:
			combined_urdf = f.read()
		with open(urdf_path+'.srdf','r') as f:
			combined_srdf = f.read()

		self.t_env= Environment()
		self.t_env.init(combined_urdf, combined_srdf, GazeboModelResourceLocator())
		self.scene_graph=self.t_env.getSceneGraph()


		#Tesseract reports all GJK/EPA distance within contact_distance threshold
		contact_distance=0.1
		monitored_link_names = self.t_env.getLinkNames()
		self.manager = self.t_env.getDiscreteContactManager()
		self.manager.setActiveCollisionObjects(monitored_link_names)
		self.manager.setCollisionMarginData(CollisionMarginData(contact_distance))


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

	def check_collision_single(self,robot_name,q):
		###check collision for a single robot, including self collision and collision with part

		self.t_env.setState(self.robot_jointname[robot_name], q)
		env_state = self.t_env.getState()
		self.manager.setCollisionObjectsTransform(env_state.link_transforms)

		result = tesseract_collision.ContactResultMap()
		contacts = self.manager.contactTest(result,tesseract_collision.ContactRequest(tesseract_collision.ContactTestType_ALL))
		result_vector = tesseract_collision.ContactResultVector()
		tesseract_collision.flattenResults(result,result_vector)
		###iterate all collision instances
		for c in result_vector: 
			if (c.link_names[0] in self.robot_linkname[robot_name] or c.link_names[1] in self.robot_linkname[robot_name]):
				print(c.link_names[0], c.link_names[1], c.distance)
				return True

		return False

	def plan(self,robot_name,q_cur,q_goal):
		###plan a trajectory for a single robot
		manip_info = ManipulatorInfo()
		manip_info.tcp_frame = "tool0"
		manip_info.manipulator = "manipulator"
		manip_info.working_frame = "base_link"

		# Create a viewer and set the environment so the results can be displayed later
		viewer = TesseractViewer()
		viewer.update_environment(t_env, [0,0,0])

		# Set the initial state of the robot
		joint_names = ["joint_%d" % (i+1) for i in range(6)]
		viewer.update_joint_positions(joint_names, np.array([1,-.2,.01,.3,-.5,1]))

		# Start the viewer
		viewer.start_serve_background()

		# Set the initial state of the robot
		t_env.setState(joint_names, np.ones(6)*0.1)

		# Create the input command program waypoints
		wp1 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8,-0.3,1.455) * Quaterniond(0.70710678,0,0.70710678,0))
		wp2 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8,0.3,1.455) * Quaterniond(0.70710678,0,0.70710678,0))
		wp3 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8,0.5,1.455) * Quaterniond(0.70710678,0,0.70710678,0))

		# Create the input command program instructions. Note the use of explicit construction of the CartesianWaypointPoly
		# using the *_wrap_CartesianWaypoint functions. This is required because the Python bindings do not support implicit
		# conversion from the CartesianWaypoint to the CartesianWaypointPoly.
		start_instruction = MoveInstruction(CartesianWaypointPoly_wrap_CartesianWaypoint(wp1), MoveInstructionType_FREESPACE, "DEFAULT")
		plan_f1 = MoveInstruction(CartesianWaypointPoly_wrap_CartesianWaypoint(wp2), MoveInstructionType_FREESPACE, "DEFAULT")
		plan_f2 = MoveInstruction(CartesianWaypointPoly_wrap_CartesianWaypoint(wp3), MoveInstructionType_FREESPACE, "DEFAULT")

		# Create the input command program. Note the use of *_wrap_MoveInstruction functions. This is required because the
		# Python bindings do not support implicit conversion from the MoveInstruction to the MoveInstructionPoly.
		program = CompositeInstruction("DEFAULT")
		program.setManipulatorInfo(manip_info)
		program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
		program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f1))
		# program.appendMoveInstruction(MoveInstructionPoly(plan_f2))

		# Create the task composer plugin factory and load the plugins
		config_path = FilesystemPath(task_composer_filename)
		factory = TaskComposerPluginFactory(config_path)

		# Create the task composer node. In this case the FreespacePipeline is used. Many other are available.
		task = factory.createTaskComposerNode("FreespacePipeline")

		# Get the output keys for the task
		output_key = task.getOutputKeys()[0]

		# Create a profile dictionary. Profiles can be customized by adding to this dictionary and setting the profiles
		# in the instructions.
		profiles = ProfileDictionary()

		# Create an AnyPoly containing the program. This explicit step is required because the Python bindings do not
		# support implicit conversion from the CompositeInstruction to the AnyPoly.
		program_anypoly = AnyPoly_wrap_CompositeInstruction(program)

		# Create the task problem and input
		task_planning_problem = PlanningTaskComposerProblem(t_env, profiles)
		task_planning_problem.input = program_anypoly

		# Create an executor to run the task
		task_executor = factory.createTaskComposerExecutor("TaskflowExecutor")

		# Run the task and wait for completion
		future = task_executor.run(task.get(), task_planning_problem)
		future.wait()

		# Retrieve the output, converting the AnyPoly back to a CompositeInstruction
		results = AnyPoly_as_CompositeInstruction(future.context.data_storage.getData(output_key))

		# Display the output
		# Print out the resulting waypoints
		for instr in results:
			assert instr.isMoveInstruction()
			move_instr1 = InstructionPoly_as_MoveInstructionPoly(instr)
			wp1 = move_instr1.getWaypoint()
			assert wp1.isStateWaypoint()
			wp = WaypointPoly_as_StateWaypointPoly(wp1)
			print(f"Joint Positions: {wp.getPosition().flatten()} time: {wp.getTime()}")

	#######################################update joint angles in Tesseract Viewer###########################################
	def viewer_joints_update(self,robot_name,joints):
		self.viewer.update_joint_positions(self.robot_jointname[robot_name], np.array(joints))

	def viewer_trajectory(self,robot_names,curve_js):
		print(curve_js)
		try:
			trajectory_json = dict()
			trajectory_json["use_time"] = True
			trajectory_json["loop_time"] = 20
			joint_names=[]
			for robot_name in robot_names:
				joint_names.extend(self.robot_jointname[robot_name])
			trajectory_json["joint_names"] = joint_names
			trajectory2 = np.hstack((curve_js,np.linspace(0,10,num=len(curve_js))[np.newaxis].T))
			trajectory_json["trajectory"] = trajectory2.tolist()
			self.viewer.trajectory_json=json.dumps(trajectory_json)
			print('updated traj')
		except Exception as e:
			print(e)

def collision_test_abb():
	#link and joint names in urdf
	ABB_6640_180_255_joint_names=["ABB_6640_180_255_joint_1","ABB_6640_180_255_joint_2","ABB_6640_180_255_joint_3","ABB_6640_180_255_joint_4","ABB_6640_180_255_joint_5","ABB_6640_180_255_joint_6"]
	ABB_6640_180_255_link_names=["ABB_6640_180_255_link_1","ABB_6640_180_255_link_2","ABB_6640_180_255_link_3","ABB_6640_180_255_link_4","ABB_6640_180_255_link_5","ABB_6640_180_255_link_6"]
	
	#Robot dictionaries, all reference by name
	robot_linkname={'ABB_6640_180_255':ABB_6640_180_255_link_names}
	robot_jointname={'ABB_6640_180_255':ABB_6640_180_255_joint_names}
	
	t=Tess_Env('config/urdf/abb_cell',robot_linkname,robot_jointname)				#create obj


	q=np.array([1.1,0.5,0.7,1,1,1.])
	t.viewer_joints_update('ABB_6640_180_255',q)
	print(t.check_collision_single('ABB_6640_180_255',q))

	input("Press enter to quit")











