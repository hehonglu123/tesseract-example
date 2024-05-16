from tesseract_env import Tess_Env
import numpy as np

def main():

	ABB_6640_180_255_joint_names=["ABB_6640_180_255_joint_1","ABB_6640_180_255_joint_2","ABB_6640_180_255_joint_3","ABB_6640_180_255_joint_4","ABB_6640_180_255_joint_5","ABB_6640_180_255_joint_6"]
	ABB_6640_180_255_link_names=["ABB_6640_180_255_link_1","ABB_6640_180_255_link_2","ABB_6640_180_255_link_3","ABB_6640_180_255_link_4","ABB_6640_180_255_link_5","ABB_6640_180_255_link_6","ABB_6640_180_255_tool0"]
	ABB_1200_5_90_joint_names=['ABB_1200_5_90_joint_1','ABB_1200_5_90_joint_2','ABB_1200_5_90_joint_3','ABB_1200_5_90_joint_4','ABB_1200_5_90_joint_5','ABB_1200_5_90_joint_6']
	ABB_1200_5_90_link_names=['ABB_1200_5_90_link_1','ABB_1200_5_90_link_2','ABB_1200_5_90_link_3','ABB_1200_5_90_link_4','ABB_1200_5_90_link_5','ABB_1200_5_90_link_6','ABB_1200_5_90_tool0']

	#Robot dictionaries, all reference by name
	robot_linkname={'ABB_6640_180_255':ABB_6640_180_255_link_names,'ABB_1200_5_90':ABB_1200_5_90_link_names}
	robot_jointname={'ABB_6640_180_255':ABB_6640_180_255_joint_names,'ABB_1200_5_90':ABB_1200_5_90_joint_names}
	
	t=Tess_Env('config/','abb_cell',robot_linkname,robot_jointname)				#create obj
	H=np.eye(4)
	H[0,3]=1000
	t.add_sphere(100,H)		#add sphere

	
	input("Press enter to quit")



if __name__ == '__main__':
	main()
