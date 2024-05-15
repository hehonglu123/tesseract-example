from tesseract_env import Tess_Env
import numpy as np

def main():
	#link and joint names in urdf
	MA2010_link_names=["MA2010_base_link","MA2010_link_1_s","MA2010_link_2_l","MA2010_link_3_u","MA2010_link_4_r","MA2010_link_5_b","MA2010_link_6_t"]
	MA2010_joint_names=["MA2010_joint_1_s","MA2010_joint_2_l","MA2010_joint_3_u","MA2010_joint_4_r","MA2010_joint_5_b","MA2010_joint_6_t"]

	MA1440_link_names=["MA1440_base_link","MA1440_link_1_s","MA1440_link_2_l","MA1440_link_3_u","MA1440_link_4_r","MA1440_link_5_b","MA1440_link_6_t"]
	MA1440_joint_names=["MA1440_joint_1_s","MA1440_joint_2_l","MA1440_joint_3_u","MA1440_joint_4_r","MA1440_joint_5_b","MA1440_joint_6_t"]

	D500B_joint_names=["D500B_joint_1","D500B_joint_2"]
	D500B_link_names=["D500B_base_link","D500B_link_1","D500B_link_2"]

	#Robot dictionaries, all reference by name
	robot_linkname={'MA2010_A0':MA2010_link_names,'MA1440_A0':MA1440_link_names,'D500B':D500B_link_names}
	robot_jointname={'MA2010_A0':MA2010_joint_names,'MA1440_A0':MA1440_joint_names,'D500B':D500B_joint_names}
	
	t=Tess_Env('config/motoman_cell',robot_linkname,robot_jointname)				#create obj
	t.viewer_trajectory(['MA2010_A0'], np.linspace(np.zeros(6),np.ones(6),100))	#visualize trajectory
	# t.viewer_joints_update('MA2010_A0',np.ones(6))	#update joint angles
	input("Press enter to quit")

if __name__ == '__main__':
	main()
