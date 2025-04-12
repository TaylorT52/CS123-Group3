import numpy as np

def forward_kinematics(theta1, theta2, theta3):

        def rotation_x(angle):
            # rotation about the x-axis implemented for you
            return np.array([
                [1, 0, 0, 0],
                [0, np.cos(angle), -np.sin(angle), 0],
                [0, np.sin(angle), np.cos(angle), 0],
                [0, 0, 0, 1]
            ])

        def rotation_y(angle):
            return np.array([
                [np.cos(angle), 0, np.sin(angle), 0],
                [0, 1, 0, 0],
                [-np.sin(angle), 0, np.cos(angle), 0], 
                [0, 0, 0, 1]
            ])
        
        def rotation_z(angle):
            return np.array([
                [np.cos(angle), -np.sin(angle), 0, 0], 
                [np.sin(angle), np.cos(angle), 0, 0], 
                [0, 0, 1, 0], 
                [0, 0, 0, 1]
            ])

        def translation(x, y, z):
            return np.array([
                [1, 0, 0, x],
                [0, 1, 0 ,y],
                [0, 0, 1, z],
                [0, 0, 0, 1]
            ])


        #translation matrices 

        # T_0_1 (base_link to leg_front_r_1)
        T_0_1 = translation(0.07500, -0.0445, 0) @ rotation_x(1.57080) @ rotation_z(theta1)

        # T_1_2 (leg_front_r_1 to leg_front_r_2)
        ## TODO: Implement the transformation matrix from leg_front_r_1 to leg_front_r_2
        T_1_2 = translation(0, 0, 0.039-0.008) @ rotation_y(-np.pi / 2) @ rotation_z(theta2)

        # T_2_3 (leg_front_r_2 to leg_front_r_3)
        ## TODO: Implement the transformation matrix from leg_front_r_2 to leg_front_r_3
        T_2_3 = translation(0, -0.0494, 0.0685) @ rotation_y(np.pi / 2) @ rotation_z(theta3)

        # T_3_ee (leg_front_r_3 to end-effector)
        T_3_ee = translation(0.06231, -0.06216, 0.018)

        # TODO: Compute the final transformation. T_0_ee is a concatenation of the previous transformation matrices
        T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee

        # TODO: Extract the end-effector position.
        end_effector_position = T_0_ee[:3, 3] 

        print(end_effector_position)


        return end_effector_position

forward_kinematics(0,0,0 )

forward_kinematics(np.pi/2,np.pi/2,np.pi/2)