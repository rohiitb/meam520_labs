import numpy as np
<<<<<<< HEAD
from numpy import cos,sin
from math import pi


=======
from math import pi

>>>>>>> c6e6f6694c1f8d9e4303209810796f1892356d2e
class FK():

    def __init__(self):

        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout

<<<<<<< HEAD
        self.dh_parameters = np.array([[0.     , 0.   , 0.    ],
                                       [0.     , -pi/2, 0.333 ],
                                       [0.     ,  pi/2, 0.    ],
                                       [0.0825 ,  pi/2, 0.316 ],
                                       [-0.0825, -pi/2, 0.    ],
                                       [0.     ,  pi/2, 0.384 ],
                                       [0.088  ,  pi/2, 0.    ],
                                       [0.     , 0.   , 0.21  ]])   

        self.init_offsets()                                   

    def init_offsets(self):
        self.offsets = np.array([[0., 0., 0.141 ,1],
                                [0., 0., 0.,1],
                                [0., 0., .195,1],
                                [0., 0., 0.,1],
                                [0., 0., 0.125,1],
                                [0., 0., -0.015,1],
                                [0., 0., 0.051,1],
                                [0.,0.,0.,1]]) 


    def dh_transformation(self, theta, a, alpha, d):
        """
        Constructs the transformation matrix based on the dh parameters
        """
        A_i = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                        [0.           ,  np.sin(alpha)              ,  np.cos(alpha)              , d              ],
                        [0.           ,  0.                         ,  0.                         , 1.             ]])

        return A_i
=======
        pass
>>>>>>> c6e6f6694c1f8d9e4303209810796f1892356d2e

    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions -8 x 3 matrix, where each row corresponds to a rotational joint of the robot or end effector
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Your Lab 1 code starts here

        jointPositions = np.zeros((8,3))
<<<<<<< HEAD
        q = np.hstack((np.zeros(1), q)).reshape(-1)

        T0e = np.identity(4)

        for i in range(8):
            # print(f"For {i}th iteration : ", self.dh_transformation(q[i], self.dh_parameters[i,0], self.dh_parameters[i,1], self.dh_parameters[i,2]))
            T0e = T0e @ self.dh_transformation(q[i], self.dh_parameters[i,0], self.dh_parameters[i,1], self.dh_parameters[i,2])
            jointPositions[i] = (T0e @ self.offsets[i])[:3].reshape(-1)

        T0e = T0e @ self.dh_transformation(-pi/4, 0, 0, 0)
=======
        T0e = np.identity(4)

>>>>>>> c6e6f6694c1f8d9e4303209810796f1892356d2e
        # Your code ends here

        return jointPositions, T0e

    # feel free to define additional helper methods to modularize your solution for lab 1
<<<<<<< HEAD
=======

>>>>>>> c6e6f6694c1f8d9e4303209810796f1892356d2e
    
    # This code is for Lab 2, you can ignore it ofr Lab 1
    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
                                 world frame

        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()
    
    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()
    
if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
<<<<<<< HEAD
    # q = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
    q = np.array([0,0,0,0,0,0,0])

=======
    q = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
>>>>>>> c6e6f6694c1f8d9e4303209810796f1892356d2e

    joint_positions, T0e = fk.forward(q)
    
    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)
<<<<<<< HEAD



=======
>>>>>>> c6e6f6694c1f8d9e4303209810796f1892356d2e
