from tesseract_env import Tess_Env
from general_robotics_toolbox import rot
import numpy as np

def main():
	#link and joint names in urdf
	MA2010_link_names=["MA2010_base_link","MA2010_link_1_s","MA2010_link_2_l","MA2010_link_3_u","MA2010_link_4_r","MA2010_link_5_b","MA2010_link_6_t","MA2010_tool0"]
	MA2010_joint_names=["MA2010_joint_1_s","MA2010_joint_2_l","MA2010_joint_3_u","MA2010_joint_4_r","MA2010_joint_5_b","MA2010_joint_6_t"]

	MA1440_link_names=["MA1440_base_link","MA1440_link_1_s","MA1440_link_2_l","MA1440_link_3_u","MA1440_link_4_r","MA1440_link_5_b","MA1440_link_6_t","MA1440_tool0"]
	MA1440_joint_names=["MA1440_joint_1_s","MA1440_joint_2_l","MA1440_joint_3_u","MA1440_joint_4_r","MA1440_joint_5_b","MA1440_joint_6_t"]

	D500B_joint_names=["D500B_joint_1","D500B_joint_2"]
	D500B_link_names=["D500B_base_link","D500B_link_1","D500B_link_2"]

	#Robot dictionaries, all reference by name
	robot_linkname={'MA2010':MA2010_link_names,'MA1440':MA1440_link_names,'D500B':D500B_link_names}
	robot_jointname={'MA2010':MA2010_joint_names,'MA1440':MA1440_joint_names,'D500B':D500B_joint_names}
	
	t=Tess_Env('config/','motoman_cell',robot_linkname,robot_jointname)				#create obj
	t.plan_p2p_cs('MA2010',np.zeros(6),np.array([500., -100., 0.]),rot(np.array([0,1,0]),np.pi/2))

	
	input("Press enter to quit")



if __name__ == '__main__':
	main()
