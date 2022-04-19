import os
import time
import robot
import threading
import matplotlib.pyplot as plt
from path import goTo
from time import sleep
from numpy import *
import sys
os.chdir('/home/pi/servoblaster/')
os.system('echo sudo ./servod --pcm &')
print("-----------------------------------------------------------------")
class Move(threading.Thread):
    def __init__(self,name,qo,qf,servo_pin,tf):
        threading.Thread.__init__(self)
        self.name = name
        self.servo_pin = servo_pin
        self.qo = qo
        self.qf = qf
        self.tf = tf

    def run(self):
        qdd_p = 4*((self.qf-self.qo))/pow(self.tf,2)
        x = round((pow(qdd_p,2)*pow(self.tf,2)),3)
        y = round((4*qdd_p*(self.qf-self.qo)),3)
        num = round(sqrt(x-y),3)
        tb = (0.5*self.tf) - (num)/(2*qdd_p)
        #print(tb)
        qb_p = self.qo + (0.5*qdd_p*pow(tb,2))

        sec = linspace(0,self.tf,200)

        delay = self.tf/(len(sec))*1

        start = time.time()

        th = []
        tm = []

        velocity = []
        accele = []
        #calculating each joint's velocity and acceleration at each instance of time
        for t in sec:
            #print(t)
            if t < tb and t >= 0:
                q = self.qo + 0.5*qdd_p*pow(t,2)
                vel = qdd_p*t
                acc = qdd_p
            if t < (self.tf-tb) and t >= (tb):
                q = qb_p + qdd_p*tb*(t-tb)
                vel = t
                acc = 0
                #print(acc)
            if t <= self.tf and t >= (self.tf-tb):
                q = self.qf - 0.5*qdd_p*pow((t-self.tf),2)
                vel = qdd_p*(t-self.tf)
                acc = qdd_p
            #print(acc)
            q = round(q,2)
            t = round(t,2)
            vel = round(vel,2)
            acc = round(acc,2)

            th.append(q)
            tm.append(t)
            velocity.append(vel)
            accele.append(acc)
            def case1(q):
                if self.qf > 0:
                    q = linspace(self.qo,self.qf,100)
                else:
                    q = linspace(self.qf,self.qo,100)

                for th in q:
                    if th < -90 or th > 90:
                        print("Out of work envelope")
                    else:
                        print("-----------------------------------------------------------------")
                        theta = th + 90
                        pwm = theta + 60
                        #cmd = "echo 1="+str(pwm)+ "> /dev/servoblaster"
                        #cmd = "echo 1="+str(pwm)+ "> /dev/servoblaster"
                        cmd = "echo "+str(self.servo_pin)+"="+str(pwm)+ "> /dev/servoblaster"
                        os.system(cmd)
                        sleep(0.01)
            def case2(q):
                if q < -90 or q > 90:
                    print("***WARNING DURING EXECUTION***")
                    print("Target Out of work envelope. Joints may not move as expected")

                else:
                    q = abs(q)
                    theta = q + 90
                    pwm = theta + 60
                    #cmd = "echo 1="+str(pwm)+ "> /dev/servoblaster"
                    #cmd = "echo 1="+str(pwm)+ "> /dev/servoblaster"
                    cmd = "echo "+str(self.servo_pin)+"="+str(pwm)+ "> /dev/servoblaster"
                    os.system(cmd)
                    time.sleep(delay)

            case2(q)
        #print(q)
        stop = time.time()
        exe_t = stop-start

        return tm,th,velocity,accele

        #for theta in range(s,e):
            #pwm = theta + 60
            #cmd = "echo "+str(self.servo_pin)+"="+str(pwm)+ "> /dev/servoblaster"
            #os.system(cmd)
            #sleep(1)

def graph(tm,th,velo,accele):
    print(th)
    plt.xlabel('Time (s)')
    plt.title('Graph Showing Acceleration & Joint Angles against time')
    plt.plot(tm,th, label='Angle')
    #plt.plot(tm,velo, label='Velocity')
    #plt.plot(tm,accele, linestyle='dashed',label='Acceleration')
    plt.legend()
    plt.show()

def moveJoints(qo,qf,tf,plot=True):
    print("Starting Angles Theta1={} Theta2={} Theta3={}".format(qo[0],qo[1],qo[2]))
    print("Target Angles Theta1={} Theta2={} Theta3={}".format(qf[0],qf[1],qf[2]))
    try:
        print("Starting")
        process = []
        if qf[0]-qo[0] != 0:
            joint1 = Move('joint1',qo[0],qf[0],servo_pin=0,tf=tf) #1>GPIO-14
            joint1.start(),process.append(joint1)
            #tm,th,velo,accele = joint1.run()
            #print("Joint1 moving")
        if qf[1]-qo[1] != 0:
            joint2 = Move('joint2',qo[1],qf[1],servo_pin=1,tf=tf) #2>GPIO-17
            joint2.start(),process.append(joint2)
            #tm,th,velo,accele = joint2.run()
            #print("Joint2 moving")
        if qf[2]-qo[2] != 0:
            joint3 = Move('joint3',qo[2],qf[2],servo_pin=2,tf=tf) #3>GPIO-18
            joint3.start(),process.append(joint3)
            tm,th,velo,accele = joint3.run()
            if plot:
                graph(tm,th,velo,accele)
            #print("Joint3 moving")

        return process
    except:
        print("Error Moving Joints")

def check_target(process, TARGET):
    global NEW_TARGET
    def loop():
        global drillDista
        print("-----------------------------------------------------------------")
        drillDista = float(input("Enter Drilling Distance in (cm): "))
        print("Drilling Distance set >> {}".format(drillDista))
        if TARGET[0]+drillDista > 23.5:
            print("Too long drilling distance!!!")
            loop()
            sleep(1)
        #return drillDista

    for p in process:
        p.join() #Wait for all the threads to finish

    print("Reached Set Target x={},y={},z={}".format(TARGET[0],TARGET[1],TARGET[2]))
    sleep(2)
    loop()

    TARGET_1 = TARGET
    NEW_TARGET = [TARGET[0]+drillDista,TARGET[1],TARGET[2]]
    drill(TARGET_1,NEW_TARGET)

def drill(TARGET_1,NEW_TARGET):
    try:
        qo,qf = goTo(TARGET_1,NEW_TARGET)
        print("TARGET_1",TARGET_1)
        print("NEW_TARGET",NEW_TARGET)

        def prompt():
            global exe
            exe = int(input("Enter Drilling Time (s): "))

        prompt()
        for dely in range(0,3):
            print("Starting in...{}".format(dely+1))
            sleep(1)
        print("Drilling Started")
        process = moveJoints(qo,qf,exe,plot=False)
        print("Drilling...")
        check_finished(process)
    except Exception as e:
        print(e)
        print("Error During Drilling")

def check_finished(process):
    for p in process:
        p.join() #Wait for all the threads to finish
    print("Finished Drilling Stage")
    print("-----------------------------------------------------------------")
    sleep(1.7)

    print("Homing...")
    home(NEW_TARGET,HOME_POS)

def home(NEW_TARGET,HOME_POS):
    qo,qf = goTo(NEW_TARGET,HOME_POS)

    process = moveJoints(qo,qf,tf,plot=False)
    check_home(process)

def check_home(process):
    for p in process:
        p.join() #Wait for all the threads to finish
    print("Robot HOMED!")
    print("-----------------------------------------------------------------")
    print("System exit...")



def main():
    try:
        global HOME_POS
        global tf

        HOME_POS = [20,0,  12.0]
        TARGET   = [18,0,12,0]
        tf = 3 #Time taken to reach the target postion in seconds

        qo,qf = goTo(HOME_POS,TARGET)
        #sys.exit()
        print("Moving to target...")
        process = moveJoints(qo,qf,tf=tf,plot=True)
        check_target(process, TARGET)
    except:
        print("Error Starting Program!!!")

if __name__ == "__main__":
    main()