#!/usr/bin/env python
import numpy as np
import rospy
import rosbag
import rospkg
from trajectory_msgs.msg import JointTrajectory
from robot_arm_trajectory_control.iiwa14DynKDL import Iiwa14DynamicKDL
from robot_arm_trajectory_control.iiwa14DynBase import Iiwa14DynamicBase
from robot_arm_trajectory_control.iiwa14DynStudent import Iiwa14DynamicRef
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState


class ComputingJointAccelerations:
    def __init__(self):
        # Loading iiwa libraries to perform kinematics calculations for the robot.
        self.kdl_iiwa = Iiwa14DynamicKDL()
        self.base_iiwa = Iiwa14DynamicBase()
        self.ref_iiwa = Iiwa14DynamicRef()

        #Initialising arrays to be used for plotting accelerations
        self.time = []
        self.joint_0_accelerations = []
        self.joint_1_accelerations = []
        self.joint_2_accelerations = []
        self.joint_3_accelerations = []
        self.joint_4_accelerations = []
        self.joint_5_accelerations = []
        self.joint_6_accelerations = []

        # Defining plot for time vs accelerations
        ax = plt.subplot()
        
        # Calling a subscriber that will subscribe joint_states topic and will compute forward kinematics and forward
        # kinematics at centre of mass.
        self.sub_fk = rospy.Subscriber('/iiwa/joint_states', JointState, self.joint_state_callback, queue_size=5)
        #Defining a trajectory publisher to publish messages from the bagfile
        self.traj_publisher = rospy.Publisher('/iiwa/EffortJointInterface_trajectory_controller/command', JointTrajectory,
                                              queue_size=5)
    

    def run(self):
        """This function is the main run function of the class. When called, it runs question 5 by calling the q5()
        function to make the robot move and plot its joint accelerations.
        """
        print("run q5")
        rospy.loginfo("Waiting 5 seconds for everything to load up.")
        rospy.sleep(5.0)
        self.q5()

    def q5(self):
        """ This is the main q5 function. Here, other methods are called to ..................
        Returns:
            accelerations: 
        """
        # Steps to solving Q5.
        # a. Load the bag from the bagfile
        # b. Is this a problem of forward or inverse dynamics?
        # c. Publish the trajectory to the appropriate topic to see the robot moving in simulation.
        # d. Subscribe to the appropriate topic, and calculate the joint accelerations throughout the trajectory using dynamic components.
        # e. Plot the joint accelerations as a function of time in your python script.


        # a. Load the bag from the bagfile
        bag = self.load_bag()

        # b. --> this is a problem of forward kinematics as we have torque and need to find the acceleration.

        # c. Publish the trajectory to the appropriate topic to see the robot moving in simulation.
        self.publish_trajectory(bag)

        # d. Subscribe to the appropriate topic, and calculate the joint accelerations throughout the trajectory using dynamic components.
        # implemented in callback function

        # # e. Plot the joint accelerations as a function of time in your python script.
        self.current_time = 0.0
        ##While loop to keep the script live until rospy is shutdown, so that we can plot live accelerations
        while not rospy.is_shutdown():
            #Plotting after 40 seconds to make sure the trajectory has completed before plotting accelerations
            if(self.current_time > 40):
                # Calling function that plots the results
                self.plot_results()



    def load_bag(self):
        """This function loads the bag from the bagfile.
        """
        # Defining ros package path
        rospack = rospkg.RosPack()
        path = rospack.get_path('force_torque_simulation_kuka')

        # Load path for selected question
        bag = rosbag.Bag(path + '/bag/force_torque_simulation_kuka.bag')
        
        return bag
    
    def publish_trajectory(self,bag):
        """ Publish Trajectory function to publish the trajectory stored in the bagfile, to make the robot move.
        Args:
            bag: bag
        """
        #extract the message from the bagfile
        for topic, msg, t in bag.read_messages(topics=['/iiwa/EffortJointInterface_trajectory_controller/command']):
            #print(len(msg.points)) #3
            #publish the message read from the corresponding topic
            self.traj_publisher.publish(msg)
        #close the bagfile
        bag.close()


    def joint_state_callback(self, msg):
        """ ROS callback function for joint states of the robot. Compute the accelerations at each joint of the robot
        and append them to corresponding lists, to be used for plotting.
        Args:
            msg (JointState): Joint state message containing current robot joint position, velocity and effort:
                Header header
                string[] name
                float64[] position
                float64[] velocity
                float64[] effort
        """

        # Grab the positions, velocity and effort from the JointState message and set to current_joint_position
        current_joint_position = list(msg.position)
        current_joint_velocity = list(msg.velocity)
        current_joint_effort = list(msg.effort)


        # d. Subscribe to the appropriate topic, and calculate the joint accelerations throughout the trajectory using dynamic components.
        # Calling a subscriber that will subscribe joint_states topic and will compute forward kinematics and forward
        # kinematics at centre of mass.
        self.current_joint_acceleration = self.calculate_joint_accelerations(current_joint_position,current_joint_velocity,current_joint_effort)

        #Append the current simulation time and joint accelerations to the appropriate list to plot them later in the plot function
        self.current_time = float(rospy.Time.now().to_sec())
        self.time.append(self.current_time)
        self.joint_0_accelerations.append(self.current_joint_acceleration[:,0])
        self.joint_1_accelerations.append(self.current_joint_acceleration[:,1])
        self.joint_2_accelerations.append(self.current_joint_acceleration[:,2])
        self.joint_3_accelerations.append(self.current_joint_acceleration[:,3])
        self.joint_4_accelerations.append(self.current_joint_acceleration[:,4])
        self.joint_5_accelerations.append(self.current_joint_acceleration[:,5])
        self.joint_6_accelerations.append(self.current_joint_acceleration[:,6])
        


    def calculate_joint_accelerations(self,current_joint_position,current_joint_velocity,current_joint_effort):
        """ Calculate Joint Accelerations function for calculating the accelerations at each joint of the robot
        Args:
            current_joint_position: current position values for each joint
            current_joint_velocity: current velocity values for each joint
            current_joint_effort: current torque values for each joint
        """
        #Init array holding values of all current joint accelerations
        current_joint_acceleration = np.zeros((1, 7))
        # Get B using KDL
        B = self.kdl_iiwa.get_B(current_joint_position)
        #print("ref_iiwa B diff: ",self.ref_iiwa.get_B(current_joint_position)-self.kdl_iiwa.get_B(current_joint_position))
        # Get C times qdot using KDL
        C_times_qdot = self.kdl_iiwa.get_C_times_qdot(current_joint_position, current_joint_velocity)
        #print("ref_iiwa C diff: ",self.ref_iiwa.get_C_times_qdot(current_joint_position, current_joint_velocity)-self.kdl_iiwa.get_C_times_qdot(current_joint_position, current_joint_velocity))
        # Get G using KDL
        G = self.kdl_iiwa.get_G(current_joint_position)
        #print("ref_iiwa G diff: ",self.ref_iiwa.get_G(current_joint_position)-np.array(self.kdl_iiwa.get_G(current_joint_position)))
        # Calculate the current joint acceleration using the formula
        current_joint_acceleration = np.dot(np.linalg.inv(B),(np.array(current_joint_effort) - np.array(C_times_qdot) - np.array(G)))

        
        return current_joint_acceleration

    def plot_results(self):
        """ Plot results function to use the live joint acceleration arrays, to plot the accelerations at each joint using Matplotlib
        """
        # Plotting the joint accelerations for each joint seperately, against time
        plt.plot(self.time, np.squeeze(np.array(self.joint_0_accelerations)), label="Joint 0")
        plt.plot(self.time, np.squeeze(np.array(self.joint_1_accelerations)), label="Joint 1")
        plt.plot(self.time, np.squeeze(np.array(self.joint_2_accelerations)), label="Joint 2")
        plt.plot(self.time, np.squeeze(np.array(self.joint_3_accelerations)), label="Joint 3")
        plt.plot(self.time, np.squeeze(np.array(self.joint_4_accelerations)), label="Joint 4")
        plt.plot(self.time, np.squeeze(np.array(self.joint_5_accelerations)), label="Joint 5")
        plt.plot(self.time, np.squeeze(np.array(self.joint_6_accelerations)), label="Joint 6")
        # Adding Legend to plot
        plt.legend()
        # Adding title to plot
        plt.title('iiwa Joint Accelerations')
        # Adding x axis label to plot
        plt.xlabel('Time (s)')
        # Adding y axis label to plot
        plt.ylabel('Acceleration (m/s^2)')
        # Displaying plot
        plt.show()



if __name__ == '__main__':
    # Initialize node
    try:
        rospy.init_node('force_torque_simulation_kuka', anonymous=True)

        iiwa14 = ComputingJointAccelerations()
        iiwa14.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass