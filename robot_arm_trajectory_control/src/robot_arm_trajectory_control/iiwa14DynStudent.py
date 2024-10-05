#!/usr/bin/env python

import numpy as np
from iiwa14DynBase import Iiwa14DynamicBase


class Iiwa14DynamicRef(Iiwa14DynamicBase):
    def __init__(self):
        super(Iiwa14DynamicRef, self).__init__(tf_suffix='ref')
        # Loading iiwa libraries
        self.base_iiwa = Iiwa14DynamicBase()

    def forward_kinematics(self, joints_readings, up_to_joint=7):
        """This function solve forward kinematics by multiplying frame transformation up until a specified
        joint. Reference Lecture 9 slide 13.
        Args:
            joints_readings (list): the state of the robot joints.
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematics.
                Defaults to 7.
        Returns:
            np.ndarray The output is a numpy 4*4 matrix describing the transformation from the 'iiwa_link_0' frame to
            the selected joint frame.
        """

        assert isinstance(joints_readings, list), "joint readings of type " + str(type(joints_readings))
        assert isinstance(up_to_joint, int)

        T = np.identity(4)
        # iiwa base offset
        T[2, 3] = 0.1575

        # 1. Recall the order from lectures. T_rot_z * T_trans * T_rot_x * T_rot_y. You are given the location of each
        # joint with translation_vec, X_alpha, Y_alpha, Z_alpha. Also available are function T_rotationX, T_rotation_Y,
        # T_rotation_Z, T_translation for rotation and translation matrices.
        # 2. Use a for loop to compute the final transformation.
        for i in range(0, up_to_joint):
            T = T.dot(self.T_rotationZ(joints_readings[i]))
            T = T.dot(self.T_translation(self.translation_vec[i, :]))
            T = T.dot(self.T_rotationX(self.X_alpha[i]))
            T = T.dot(self.T_rotationY(self.Y_alpha[i]))

        assert isinstance(T, np.ndarray), "Output wasn't of type ndarray"
        assert T.shape == (4, 4), "Output had wrong dimensions"

        return T

    def get_jacobian_centre_of_mass(self, joint_readings, up_to_joint=7):
        """Given the joint values of the robot, compute the Jacobian matrix at the centre of mass of the link.
        Reference - Lecture 9 slide 14.

        Args:
            joint_readings (list): the state of the robot joints.
            up_to_joint (int, optional): Specify up to what frame you want to compute the Jacobian.
            Defaults to 7.

        Returns:
            jacobian (numpy.ndarray): The output is a numpy 6*7 matrix describing the Jacobian matrix defining at the
            centre of mass of a link.
        """
        assert isinstance(joint_readings, list)
        assert len(joint_readings) == 7

        # Your code starts here ----------------------------

        #Initialising the jacobian
        jacobian = np.zeros((6,7))
        # Finding the transformation matrix for 0T_Ge as this gives p_li
        T_Gi = self.forward_kinematics_centre_of_mass(joint_readings, up_to_joint)
        # Finding p_li which is the first three elements of the fourth column of T_Ge
        p_li = T_Gi[0:3,3]

        # Looping through each column of the jacobian:
        for j in range(up_to_joint):
            Jj = np.array([0.0,0.0,0.0,0.0,0.0,0.0]).T
            # Finding 0Tj-1 as this is required to find the rotation matrix 0pj-1 and 0Rj-1 and therefore zj-1
            T_Gj_1 = self.forward_kinematics(joint_readings, j) #try +1
            # Finding zj-1 which is given by the first three elements of the third column of 0Tj-1
            zj_1 = T_Gj_1[0:3,2]
            if j == 0:
                zj_1 = np.array([0, 0, 1]).T
            # Finding pi-1 which is given by the first three elements of the fourth column of 0Tj-1
            pj_1 = T_Gj_1[0:3,3]
            # Computing the equations for JPjand JOj
            Jj[0:3] = np.cross(zj_1,np.subtract(p_li,pj_1))
            Jj[3:6] = zj_1
            jacobian[:,j] = Jj

        # Your code ends here ------------------------------

        assert jacobian.shape == (6, 7)
        return jacobian

    def forward_kinematics_centre_of_mass(self, joints_readings, up_to_joint=7):
        """This function computes the forward kinematics up to the centre of mass for the given joint frame.
        Reference - Lecture 9 slide 14.
        Args:
            joints_readings (list): the state of the robot joints.
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematicks.
                Defaults to 5.
        Returns:
            np.ndarray: A 4x4 homogeneous transformation matrix describing the pose of frame_{up_to_joint} for the
            centre of mass w.r.t the base of the robot.
        """
        T= np.identity(4)
        T[2, 3] = 0.1575

        T = self.forward_kinematics(joints_readings, up_to_joint-1)
        T = T.dot(self.T_rotationZ(joints_readings[up_to_joint-1]))
        T = T.dot(self.T_translation(self.link_cm[up_to_joint-1, :]))

        return T

    def get_B(self, joint_readings):
        """Given the joint positions of the robot, compute inertia matrix B.
        Args:
            joint_readings (list): The positions of the robot joints.

        Returns:
            B (numpy.ndarray): The output is a numpy 7*7 matrix describing the inertia matrix B.
        """
        
	    # Your code starts here ------------------------------

        #1. Define B as a n*n matrix
        n = len(joint_readings)
        B = np.zeros((n, n))

        ## Define Inertia matrix here of shape 3*3
        # Each row is (Ixx, Iyy, Izz) and Ixy = Ixz = Iyz = 0.
        Ili = np.zeros((3,3))

        for i in range(len(joint_readings)):

            #2. compute and store the FK at CoM for joint i
            T = self.forward_kinematics_centre_of_mass(joint_readings, i+1)

            # Mass of link i
            mli = self.base_iiwa.mass[i]

            #Finding the rotation matrix for the centre of mass
            RGi = T[0:3,0:3]
            # Moment on inertia of current link, defined at the centre of mass.
            Ili[0,0] = self.base_iiwa.Ixyz[i,0]
            Ili[1,1] = self.base_iiwa.Ixyz[i,1]
            Ili[2,2] = self.base_iiwa.Ixyz[i,2]
            #Finding the inertia tensor at the centre of mass:
            # Each row is (Ixx, Iyy, Izz) and Ixy = Ixz = Iyz = 0.
            inertia_tensor = np.dot(np.dot(RGi,Ili),RGi.T)

            #3. compute and store the Jacobian at CoM for joint i
            J = self.get_jacobian_centre_of_mass(joint_readings, i+1)
            Jp = J[0:3,:]
            Jo = J[3:6,:]
            
            #4. Compute B following the formula
            B = B + (mli*np.dot(Jp.T,Jp)) + (np.dot(np.dot(Jo.T,inertia_tensor),Jo))
            

        # Your code ends here ------------------------------

        return B

    def get_C_times_qdot(self, joint_readings, joint_velocities):
        """Given the joint positions and velocities of the robot, compute Coriolis terms C.
        Args:
            joint_readings (list): The positions of the robot joints.
            joint_velocities (list): The velocities of the robot joints.

        Returns:
            C (numpy.ndarray): The output is a numpy 7*1 matrix describing the Coriolis terms C times joint velocities.
        """
        assert isinstance(joint_readings, list)
        assert len(joint_readings) == 7
        assert isinstance(joint_velocities, list)
        assert len(joint_velocities) == 7

        # Your code starts here ------------------------------
        

        #1. Define h_ijk and C as a nxn and a nxnxn matrices respectively
        n = len(joint_readings)
        h_ijk = np.ndarray((n,n,n))
        C = np.zeros((n,n))

        #2. fill h_ijk following the above formula
        for i in range(n):

            for j in range(n):

                for k in range(n):

                    #finding B without any change in joint readings, we use this to find a derivative as the derative is with respect to q.
                    B = self.get_B(joint_readings)

                    # To compute the derivatives we can use an approximation by finding (f(x+h)-f(x))/h as h tends to 0.
                    h = 0.0000001 #this is the deviation in the joint values that we need to concider as the diff is with respect to the joint values.

                    #Computing the first derivative element with respect to the kth joint:
                    joint_readings_kd = np.array(joint_readings).copy()
                    joint_readings_kd[k] = joint_readings_kd[k] + h
                    Bd = self.get_B(list(joint_readings_kd))
                    # First derivative term
                    bij_dqk = (Bd[i,j] - B[i,j])/h

                    #Computing the second derivative element with respect to the ith joint:
                    joint_readings_id = np.array(joint_readings).copy()
                    joint_readings_id[i] = joint_readings_id[i] + h
                    Bd = self.get_B(list(joint_readings_id))
                    # Second derivative term
                    bjk_dqi = (Bd[j,k] - B[j,k])/h

                    #Finding h_ijk
                    h_ijk[i,j,k] = bij_dqk - 0.5*(bjk_dqi)

                    #3. Filling in C
                    C[i,j] = C[i,j] + np.dot(h_ijk[i,j,k],joint_velocities[k])

        # We multiply the velocity with C to get 7*1
        C = np.dot(C,joint_velocities)

        # Your code ends here ------------------------------

        assert isinstance(C, np.ndarray)
        assert C.shape == (7,)
        return C

    def get_G(self, joint_readings):
        """Given the joint positions of the robot, compute the gravity matrix g.
        Args:
            joint_readings (list): The positions of the robot joints.

        Returns:
            G (numpy.ndarray): The output is a numpy 7*1 numpy array describing the gravity matrix g.
        """
        assert isinstance(joint_readings, list)
        assert len(joint_readings) == 7

        # Your code starts here ------------------------------

        #1. Define g as a nx1 matrix
        n = len(joint_readings)
        g = np.zeros((n,1))


        #2. Compute P(q) using the formula

        # goT = [0 0 -gravity]
        goT = np.array([0,0,-self.base_iiwa.g])

        # Initialise P(q)
        Pq = 0
        for i in range(n):
            
            # Mass of link i
            mli = self.base_iiwa.mass[i]

            # compute the FK at CoM for joint i and extract its position
            T = self.forward_kinematics_centre_of_mass(joint_readings, i+1) 
            pli = T[0:3,3]

            # Calculate P(q) using the formula
            Pq = Pq - mli*np.dot(goT,pli)
            

            
        #3. Filling g using the formula, and again using the approximation of the derivative
        
        # To compute the derivatives we can use an approximation by finding (f(x+h)-f(x))/h as h tends to 0.
        h = 0.0000001 #this is the deviation in the joint values that we need to concider as the diff is with respect to the joint values.

        # To find the derivative with respect to q, I first need to find Pq with a small change  in q for each joint:
        # Initialising P(q) with small change in q for each joint.
        Pqd = np.zeros((n,1))

        # Need an extra loop as we want to find new joint readings with small step change when finding Pqd
        for i in range(n):
            
            # Joint readings with small deviation in readings of joint i
            joint_readings_id = np.array(joint_readings).copy()
            joint_readings_id[i] = joint_readings_id[i]+h

            for j in range(n):
                
                # Mass of link j
                mlj = self.base_iiwa.mass[j]
                
                # compute the FK at CoM for joint j and extract its position
                T = self.forward_kinematics_centre_of_mass(list(joint_readings_id), j+1)
                plj = T[0:3,3]

                # Calculate P(q) using the formula
                Pqd[i] = Pqd[i] - mlj*np.dot(goT,plj)

        #Now computing the derivative and filling in G:
        for i in range(n):
            g[i] = (Pqd[i] - Pq)/h
        g = g.ravel()


        # Your code ends here ------------------------------

        assert isinstance(g, np.ndarray)
        assert g.shape == (7,)
        return g