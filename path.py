from numpy import *
from robot import kinematics
import sys
import threading as t
import time

#Generate the path to be followed by the robot arm
class trajectory_planner():
    #Initializing the start and end position
    def __init__(self,x1,y1,z1,x2,y2,z2):
        self.x1 = x1
        self.y1 = y1
        self.z1 = z1
        self.x2 = x2
        self.y2 = y2
        self.z2 = z2
    #Equation that returns the respective angles of each joint
    #Get the first angle and the last angle for the joints
    def getAngles(self):
        qo,qf = [],[]
        #Returns the joint angles
        th1,th2,th3 = my_robot.ikine(self.x1,self.y1,self.z1) #Returns theta1,theta2,theta3
        qo.append(th1),qo.append(th2),qo.append(th3)

        th1,th2,th3 = my_robot.ikine(self.x2,self.y2,self.z2) #Returns theta1,theta2,theta3
        qf.append(th1),qf.append(th2),qf.append(th3),
        return qo,qf

    #Command for the gripper to either grasp or release the object
    def drill(self, process,flag=0): #0 > gripper, 1 > Grab
        for p in process:
            p.join() #Wait for all the threads to finish
        time.sleep(2)
        if flag:
            print("Drilling....")

my_robot = kinematics(a1=13.8,a2=0,a3=15.5,a4=8) #Arguments are link1,link2,link3,link4
def goTo(HOME_POS,TARGET):
    main = trajectory_planner(HOME_POS[0],HOME_POS[1],HOME_POS[2],
                      TARGET[0],TARGET[1],TARGET[2]) #arguments are the intial position and the endeffector position

    qo,qf = main.getAngles()
    return qo,qf
if __name__ == '__main__':
    HOME_POS = [4,0,1]
    TARGET = [3,0,1]
    tf = 3
    goTo(HOME_POS,TARGET)