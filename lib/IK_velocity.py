import numpy as np 
from lib.calcJacobian import calcJacobian
# from calcJacobian import calcJacobian


def IK_velocity(q_in, v_in, omega_in):
     """
     :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
     :param v_in: The desired linear velocity in the world frame. If any element is
     Nan, then that velocity can be anything
     :param omega_in: The desired angular velocity in the world frame. If any
     element is Nan, then that velocity is unconstrained i.e. it can be anything
     :return:
     dq - 1 x 7 vector corresponding to the joint velocities. If v_in and omega_in
          are infeasible, then dq should minimize the least squares error. If v_in
          and omega_in have multiple solutions, then you should select the solution
          that minimizes the l2 norm of dq
     """

     ## STUDENT CODE GOES HERE

     dq = np.zeros((1, 7))

     v_in = v_in.reshape((3,1))
     omega_in = omega_in.reshape((3,1))
     J = calcJacobian(q_in)
     vels = np.vstack((v_in, omega_in))
     nan_idx = np.where(~np.isnan(vels))
     if len(nan_idx) == 7:
          return dq
     vels_new = vels[nan_idx]
     J = J[nan_idx[0]]
     dq = np.linalg.lstsq(J,vels_new,rcond=None)[0]
     
   
     return dq

if __name__ == '__main__':
     # q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
     q= np.array([0, 0, 0, 0, 0, 0, 0])

     # vels = np.array([[7.71527483e-18],
     #                  [2.64000000e-02],
     #                  [0.00000000e+00],
     #                  [0.00000000e+00],
     #                  [0.00000000e+00],
     #                  [3.00000000e-01]])
     vels = np.array([[7.71527483e-18],
                    [np.nan],
                    [0.00000000e+00],
                    [np.nan],
                    [0.653],
                    [0.3]])

     v_in = vels[:3]
     w_in = vels[3:]
     dq = IK_velocity(q, v_in, w_in)


