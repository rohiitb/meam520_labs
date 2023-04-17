import numpy as np 
from lib.calcJacobian import calcJacobian
<<<<<<< HEAD
# from calcJacobian import calcJacobian

=======
>>>>>>> c6e6f6694c1f8d9e4303209810796f1892356d2e

def FK_velocity(q_in, dq):
    """
    :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    :param dq: 1 x 7 vector corresponding to the joint velocities.
    :return:
    velocity - 6 x 1 vector corresponding to the end effector velocities.    
    """

    ## STUDENT CODE GOES HERE

    velocity = np.zeros((6, 1))
<<<<<<< HEAD
    J = calcJacobian(q_in)

    velocity = J @ dq

    return velocity.reshape(-1,1)

if __name__ == '__main__':
    # q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    q= np.array([0, 0, 0, 0, 0, 0, 0])
    dq = np.array([0.1,0,0.1,0,0.1,0,0])
    vel = FK_velocity(q, dq)
    print(vel)
=======


    return velocity
>>>>>>> c6e6f6694c1f8d9e4303209810796f1892356d2e
