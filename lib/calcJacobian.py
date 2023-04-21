import numpy as np
from numpy import cos,sin
from math import pi

from lib.calculateFK import FK
# from calculateFK import FK

class FK():

    def __init__(self):

        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout

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
        q = np.hstack((np.zeros(1), q)).reshape(-1)

        T0e = np.identity(4)

        for i in range(8):
            # print(f"For {i}th iteration : ", self.dh_transformation(q[i], self.dh_parameters[i,0], self.dh_parameters[i,1], self.dh_parameters[i,2]))
            T0e = T0e @ self.dh_transformation(q[i], self.dh_parameters[i,0], self.dh_parameters[i,1], self.dh_parameters[i,2])
            jointPositions[i] = (T0e @ self.offsets[i])[:3].reshape(-1)

        T0e = T0e @ self.dh_transformation(-pi/4, 0, 0, 0)
        # Your code ends here

        return jointPositions, T0e

def calcJacobian(q_in):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """

    J = np.zeros((6, 7))

    ## STUDENT CODE GOES HERE
    fk = FK()
    joint_positions, T0e = fk.forward(q_in)

    o_n = (T0e @ np.array([0,0,0,1]))[:3]
    o_i = o_n.T - joint_positions

    T0e_here = np.eye(4)
    q = np.hstack((np.zeros(1), q_in)).reshape(-1)
    
    z_i = []
    for i in range(1,8):
        z = T0e_here[:3,:3] @ np.array([0,0,1])
        z = z/np.linalg.norm(z)
        z_i.append(z)
        T0e_here = T0e_here @ fk.dh_transformation(q[i], fk.dh_parameters[i,0], fk.dh_parameters[i,1], fk.dh_parameters[i,2])
        J[:3,i-1] = np.cross(z, o_i[i-1])
        J[3:,i-1] = z

    return J

if __name__ == '__main__':
    # q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    q= np.array([0, 0, 0, 0, 0, 0, 0])

    print(np.round(calcJacobian(q),3))
