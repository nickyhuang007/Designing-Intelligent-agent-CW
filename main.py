import tkinter as tk
import random
import math
import numpy as np
import sympy as sp
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import time

class Brain():

    def __init__(self,botp,directControlp,PIDControlp,fuzzyControlp,controlTypep='fuzzyControl'):
        self.bot = botp

        self.directControl = directControlp
        self.fuzzyControl = fuzzyControlp
        self.PIDControl = PIDControlp

        self.turningCount = 0
        self.movingCount = random.randrange(50,100)
        self.currentlyTurning = False
        self.map = self.bot.map()
        self.controlType = controlTypep

        self.flagSearch = 0
        self.stopFlag = 0
        # print(self.map)

    # modify this to change the robot's behaviour
    def thinkAndAct(self, x, y, sl, sr, count):
        newX = None
        newY = None

        speedLeft = 0
        speedRight = 0

        # print('output:',self.bot.leftSensor1Output())

        # sensor1 = self.bot.leftSensor1Output()
        # sensor2 = self.bot.leftSensor2Output()
        # sensor3 = self.bot.frontSensorOutput()

        sensor1 = self.bot.leftSensor1Output() + random.gauss(0.01 * self.bot.leftSensor1Output(), 1)
        sensor2 = self.bot.leftSensor2Output() + random.gauss(0.01 * self.bot.leftSensor2Output(), 1)
        sensor3 = self.bot.frontSensorOutput() + random.gauss(0.01 * self.bot.frontSensorOutput(), 1)

        # print(sensor3)

        if self.bot.x > 1500 :

            if self.stopFlag == 0:
                self.bot.runTime = time.time() - self.bot.startTime
                self.stopFlag = 1

            speedLeft = 0
            speedRight = 0            

        else :
            

            if self.bot.theta < math.pi and self.flagSearch == 0 :
                speedLeft = -0.5
                speedRight = 0.5
            else :
                if min(sensor1, sensor2) > 100 and self.flagSearch == 0:
                    print('search')
                    if self.bot.theta > (math.pi / 6) * 10 and self.bot.theta < (math.pi / 6) * 11 :
                        speedLeft = 2
                        speedRight = 2
                    else :
                        speedLeft = -0.5
                        speedRight = 0.5                                 
                else :

                    self.flagSearch = 1
                    # print('run')
                    

                    if sensor3 < (self.bot.ll / 2)*np.sqrt(2) + 40 :
                        speedLeft = 1
                        speedRight = -1                    

                    else :

                        if self.controlType == 'direct' :
                            speedLeft, speedRight = self.directControl.directControlWallFollowing(sensor1, sensor2)

                        elif self.controlType == 'PID' :
                            speedLeft, speedRight = self.PIDControl.PIDWallFollowing(sensor1, sensor2)

                        elif self.controlType == 'fuzzy' :
                            # print('run1')
                            speedLeft, speedRight = self.fuzzyControl.fuzzyWallFollowing(sensor1, sensor2)
                            # speedLeft = 3 
                            # speedRight = 3
                            # print('output:',self.bot.leftSensor1Output(), self.bot.leftSensor2Output())
                            # print(self.fuzzyControl.fuzzyWallFollowing(self.bot.leftSensor1Output(), self.bot.leftSensor2Output()))

        #toroidal geometry
        if x>1700:
            newX = 0
        if x<0:
            newX = 1700
        if y>1000:
            newY = 0
        if y<0:
            newY = 1000

        return speedLeft, speedRight, newX, newY

class Bot():

    def __init__(self,namep,thetap,passiveObjectsp,counterp):
        self.name = namep
        # print()
        self.x = passiveObjectsp[0].getLocation()[0] + 50
        self.y = passiveObjectsp[0].getLocation()[1] + 50

        self.pathPoints = [(self.x,self.y)]
        self.pathLength = 0

        self.startTime = time.time()
        self.runTime = time.time()

        # self.x = random.randint(100,900)
        # self.y = random.randint(100,900)
        # self.theta = random.uniform(0.0,2.0*math.pi)
        self.theta = thetap
        #self.theta = 0
        # self.ll = 60 #axle width
        self.ll = 30 #axle width
        self.sl = 0.0
        self.sr = 0.0
        self.passiveObjects = passiveObjectsp
        self.counter = counterp

    def thinkAndAct(self, agents, passiveObjects):
        self.sl, self.sr, xx, yy = self.brain.thinkAndAct\
            (self.x, self.y, self.sl, self.sr, self.counter.dirtCollected)
        if xx != None:
            self.x = xx
        if yy != None:
            self.y = yy
        
    def setBrain(self,brainp):
        self.brain = brainp

    #returns the result from the ceiling-mounted dirt camera
    def map(self):
        map = np.zeros((10,10),dtype=np.int16)
        for p in self.passiveObjects:
            if isinstance(p,Dirt):
                xx = int(math.floor(p.centreX/100.0))
                yy = int(math.floor(p.centreY/100.0))
                map[xx][yy] += 1
        return map

    def distanceTo(self,obj):
        xx,yy = obj.getLocation()
        return math.sqrt( math.pow(self.x-xx,2) + math.pow(self.y-yy,2) )

    def distanceToRightSensor(self,lx,ly):
        return math.sqrt( (lx-self.sensorPositions[0])*(lx-self.sensorPositions[0]) + \
                          (ly-self.sensorPositions[1])*(ly-self.sensorPositions[1]) )

    def distanceToLeftSensor(self,lx,ly):
        return math.sqrt( (lx-self.sensorPositions[2])*(lx-self.sensorPositions[2]) + \
                            (ly-self.sensorPositions[3])*(ly-self.sensorPositions[3]) )

    # calculate left sensor1 output
    def leftSensor1Output(self) :

        reverseTheta = (2 * math.pi - self.theta) % (2 * math.pi)
        reverseTheta = (reverseTheta + math.pi / 2) % (math.pi)
        reverseTheta = -reverseTheta

        symbol_x = sp.Symbol('x')
        sensorFunction = math.tan(reverseTheta) * (symbol_x - self.sensorPositions[0]) + self.sensorPositions[1]
        # print(self.sensorPositions[0],self.sensorPositions[1])
        # print(math.tan(reverseTheta))
        # print(self.sensorPositions[0])
        # print(sensorFunction)
        wallFunction = self.passiveObjects[0].f_symbol

        interSection_x_list = sp.solve(sensorFunction - wallFunction, symbol_x)

        # print('List',interSection_x_list)

        # interSection_x = min([index for index in interSection_x_list if index > 0])
        # interSection_x = min([sp.re(index) for index in interSection_x_list if sp.re(index) > 0 and np.abs(sp.im(index)) < 1e-6])
        interSection_x = [sp.re(index) for index in interSection_x_list if sp.re(index) > 0 and np.abs(sp.im(index)) < 1e-6]
        # interSection_x = float(interSection_x)
        # print('x:',interSection_x)

        f_wallFunction = sp.lambdify(symbol_x, wallFunction, modules=['numpy'])

        temp = np.inf
        for index_x in interSection_x :
            x = np.double(index_x)
            y = np.double(f_wallFunction(x))

            dx = x - self.sensorPositions[0]
            dy = y - self.sensorPositions[1]

            if dx * math.cos(reverseTheta) + dy * math.sin(reverseTheta) > 0:

                if np.sqrt((x - self.sensorPositions[0]) ** 2 + (y - self.sensorPositions[1]) ** 2) < temp :
                    temp = np.sqrt((x - self.sensorPositions[0]) ** 2 + (y - self.sensorPositions[1]) ** 2)

        sensorOutput = temp

        # interSection_y = float(f_wallFunction(interSection_x))

        # sensorOutput = np.sqrt((interSection_x - self.sensorPositions[0]) ** 2 + \
        #                        (interSection_y - self.sensorPositions[1]) ** 2)
        # print('Output:',sensorOutput)
        
        return sensorOutput

    # calculate left sensor2 output
    def leftSensor2Output(self) :

        reverseTheta = (2 * math.pi - self.theta) % (2 * math.pi)
        reverseTheta = (reverseTheta + math.pi / 2) % (math.pi)
        reverseTheta = -reverseTheta

        symbol_x = sp.Symbol('x')
        sensorFunction = math.tan(reverseTheta) * (symbol_x - self.sensorPositions[2]) + self.sensorPositions[3]
        # print(self.sensorPositions[0],self.sensorPositions[1])
        # print(math.tan(reverseTheta))
        # print(self.sensorPositions[0])
        # print(sensorFunction)
        wallFunction = self.passiveObjects[0].f_symbol

        interSection_x_list = sp.solve(sensorFunction - wallFunction, symbol_x)

        # print('List',interSection_x_list)

        # interSection_x = min([index for index in interSection_x_list if index > 0])
        interSection_x = [sp.re(index) for index in interSection_x_list if sp.re(index) > 0 and np.abs(sp.im(index)) < 1e-6]

        # print(interSection_x)

        f_wallFunction = sp.lambdify(symbol_x, wallFunction, modules=['numpy'])

        temp = np.inf
        for index_x in interSection_x :
            x = np.double(index_x)
            y = np.double(f_wallFunction(x))

            dx = x - self.sensorPositions[2]
            dy = y - self.sensorPositions[3]

            if dx * math.cos(reverseTheta) + dy * math.sin(reverseTheta) > 0:

                if np.sqrt((x - self.sensorPositions[2]) ** 2 + (y - self.sensorPositions[3]) ** 2) < temp :
                    temp = np.sqrt((x - self.sensorPositions[2]) ** 2 + (y - self.sensorPositions[3]) ** 2)

        sensorOutput = temp

        # interSection_y = float(f_wallFunction(interSection_x))

        # sensorOutput = np.sqrt((interSection_x - self.sensorPositions[2]) ** 2 + \
        #                        (interSection_y - self.sensorPositions[3]) ** 2)
        
        return sensorOutput

    def frontSensorOutput(self) :

        reverseTheta = (2 * math.pi - self.theta) % (2 * math.pi)
        reverseTheta = (reverseTheta + math.pi) % (math.pi)
        reverseTheta = -reverseTheta

        

        symbol_x = sp.Symbol('x')
        sensorFunction = math.tan(reverseTheta) * (symbol_x - self.sensorPositions[4]) + self.sensorPositions[5]
        # print(self.sensorPositions[0],self.sensorPositions[1])
        # print(math.tan(reverseTheta))
        # print(self.sensorPositions[0])
        # print(sensorFunction)
        wallFunction = self.passiveObjects[0].f_symbol

        interSection_x_list = sp.solve(sensorFunction - wallFunction, symbol_x)

        if not interSection_x_list:
            return np.inf
        # print('List',interSection_x_list)

        # interSection_x = min([index for index in interSection_x_list if index > 0])
        interSection_x = [sp.re(index) for index in interSection_x_list if sp.re(index) > 0 and np.abs(sp.im(index)) < 1e-6]

        # print(interSection_x)

        f_wallFunction = sp.lambdify(symbol_x, wallFunction, modules=['numpy'])

        temp = np.inf
        for index_x in interSection_x :
            x = np.double(index_x)
            y = np.double(f_wallFunction(x))

            dx = x - self.sensorPositions[4]
            dy = y - self.sensorPositions[5]

            if dx * math.cos(reverseTheta) + dy * math.sin(reverseTheta) > 0:

                if np.sqrt((x - self.sensorPositions[4]) ** 2 + (y - self.sensorPositions[5]) ** 2) < temp :
                    temp = np.sqrt((x - self.sensorPositions[4]) ** 2 + (y - self.sensorPositions[5]) ** 2)

        sensorOutput = temp

        # interSection_y = float(f_wallFunction(interSection_x))

        # sensorOutput = np.sqrt((interSection_x - self.sensorPositions[2]) ** 2 + \
        #                        (interSection_y - self.sensorPositions[3]) ** 2)
        
        return sensorOutput

    # what happens at each timestep
    def update(self,canvas,passiveObjects,dt):
        self.move(canvas,dt)

    # draws the robot at its current position
    def draw(self,canvas):
        temp_ll = self.ll / 2
        points = [ (self.x + temp_ll*math.sin(self.theta)) - temp_ll*math.sin((math.pi/2.0)-self.theta), \
                   (self.y - temp_ll*math.cos(self.theta)) - temp_ll*math.cos((math.pi/2.0)-self.theta), \
                   (self.x - temp_ll*math.sin(self.theta)) - temp_ll*math.sin((math.pi/2.0)-self.theta), \
                   (self.y + temp_ll*math.cos(self.theta)) - temp_ll*math.cos((math.pi/2.0)-self.theta), \
                   (self.x - temp_ll*math.sin(self.theta)) + temp_ll*math.sin((math.pi/2.0)-self.theta), \
                   (self.y + temp_ll*math.cos(self.theta)) + temp_ll*math.cos((math.pi/2.0)-self.theta), \
                   (self.x + temp_ll*math.sin(self.theta)) + temp_ll*math.sin((math.pi/2.0)-self.theta), \
                   (self.y - temp_ll*math.cos(self.theta)) + temp_ll*math.cos((math.pi/2.0)-self.theta)  \
                ]
        canvas.create_polygon(points, fill="blue", tags=self.name)

        self.sensorPositions = [ (self.x + temp_ll*math.sin(self.theta)) - (temp_ll - 5) * math.cos(self.theta), \
                                 (self.y - temp_ll*math.cos(self.theta)) - (temp_ll - 5) * math.sin(self.theta), \
                                 (self.x + temp_ll*math.sin(self.theta)) + (temp_ll - 5) * math.cos(self.theta), \
                                 (self.y - temp_ll*math.cos(self.theta)) + (temp_ll - 5) * math.sin(self.theta),  \
                                 (self.x + temp_ll*math.cos(self.theta)), \
                                 (self.y + temp_ll*math.sin(self.theta))
                            ]

        centre1PosX = self.x 
        centre1PosY = self.y

        canvas.create_oval(centre1PosX-8,centre1PosY-8,\
                           centre1PosX+8,centre1PosY+8,\
                           fill="gold",tags=self.name)

        wheel1PosX = self.x - temp_ll*math.sin(self.theta)
        wheel1PosY = self.y + temp_ll*math.cos(self.theta)
        canvas.create_oval(wheel1PosX-3,wheel1PosY-3,\
                                         wheel1PosX+3,wheel1PosY+3,\
                                         fill="red",tags=self.name)

        wheel2PosX = self.x + temp_ll*math.sin(self.theta)
        wheel2PosY = self.y - temp_ll*math.cos(self.theta)
        canvas.create_oval(wheel2PosX-3,wheel2PosY-3,\
                                         wheel2PosX+3,wheel2PosY+3,\
                                         fill="green",tags=self.name)

        #Left Sensor
        sensor1PosX = self.sensorPositions[0]
        sensor1PosY = self.sensorPositions[1]
        sensor2PosX = self.sensorPositions[2]
        sensor2PosY = self.sensorPositions[3]
        canvas.create_oval(sensor1PosX-3,sensor1PosY-3, \
                           sensor1PosX+3,sensor1PosY+3, \
                           fill="yellow",tags=self.name)
        canvas.create_oval(sensor2PosX-3,sensor2PosY-3, \
                           sensor2PosX+3,sensor2PosY+3, \
                           fill="yellow",tags=self.name)

        #Front Sensor
        sensor3PosX = self.sensorPositions[4]
        sensor3PosY = self.sensorPositions[5]
        canvas.create_oval(sensor3PosX-3,sensor3PosY-3, \
                           sensor3PosX+3,sensor3PosY+3, \
                           fill="black",tags=self.name)
        
    # handles the physics of the movement
    # cf. Dudek and Jenkin, Computational Principles of Mobile Robotics
    def move(self,canvas,dt):
        # if self.sl==self.sr:
        if abs(self.sl - self.sr) < 1e-6:
            R = 0
        else:
            R = (self.ll/2.0)*((self.sr+self.sl)/(self.sl-self.sr))
        omega = (self.sl-self.sr)/self.ll
        ICCx = self.x-R*math.sin(self.theta) #instantaneous centre of curvature
        ICCy = self.y+R*math.cos(self.theta)
        m = np.matrix( [ [math.cos(omega*dt), -math.sin(omega*dt), 0], \
                        [math.sin(omega*dt), math.cos(omega*dt), 0],  \
                        [0,0,1] ] )
        v1 = np.matrix([[self.x-ICCx],[self.y-ICCy],[self.theta]])
        v2 = np.matrix([[ICCx],[ICCy],[omega*dt]])
        newv = np.add(np.dot(m,v1),v2)
        newX = newv.item(0)
        newY = newv.item(1)
        newTheta = newv.item(2)
        newTheta = newTheta%(2.0*math.pi) #make sure angle doesn't go outside [0.0,2*pi)
        self.x = newX
        self.y = newY
        self.theta = newTheta        
        # if self.sl==self.sr: # straight line movement
        if abs(self.sl - self.sr) < 1e-6:
            self.x += self.sr*math.cos(self.theta) #sr wlog
            self.y += self.sr*math.sin(self.theta)
        canvas.delete(self.name)
        self.draw(canvas)
        # print("New XY",newX,newY)

        self.pathPoints.append((self.x, self.y)) 
        self.drawPath(canvas)                    

        
    def collectDirt(self, canvas, passiveObjects, count):
        toDelete = []
        for idx,rr in enumerate(passiveObjects):
            if isinstance(rr,Dirt):
                if self.distanceTo(rr)<30:
                    canvas.delete(rr.name)
                    toDelete.append(idx)
                    count.itemCollected(canvas)
        for ii in sorted(toDelete,reverse=True):
            del passiveObjects[ii]
        return passiveObjects
        
    def drawPath(self, canvas):
        points = [coord for point in self.pathPoints for coord in point]

        if self.brain.controlType == 'direct' :
            canvas.create_line(points, fill="blue", width=1, tags="path1")
        
        elif self.brain.controlType == 'PID' :
            canvas.create_line(points, fill="red", width=1, tags="path2")

        elif self.brain.controlType == 'fuzzy' :    
            canvas.create_line(points, fill="green", width=1, tags="path3")

    def plotBotInfo(self, canvas) :

        self.pathLength = 0
        for i in range(1, len(self.pathPoints)) :
            x1,y1 = self.pathPoints[i-1]
            x2,y2 = self.pathPoints[i]

            temp = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            self.pathLength += temp
        
        if self.brain.controlType == 'direct' :
            canvas.delete('pathName1')
            canvas.create_text(50,10,anchor="w",\
                           text=self.brain.controlType+':',\
                           tags='pathName1')
            
            canvas.delete('pathLength1')
            canvas.create_text(50,25,anchor="w",\
                           text='Path Length: '+str(self.pathLength),\
                           tags='pathLength1')
        
            runTime = time.time() - self.startTime 
            canvas.delete('runTime1')
            canvas.create_text(50, 40, anchor="w",
                           text='Run Time: '+str(runTime),
                           tags='runTime1')
        
        elif self.brain.controlType == 'PID' :
            canvas.delete('pathName2')
            canvas.create_text(50,70,anchor="w",\
                           text=self.brain.controlType+':',\
                           tags='pathName2')
            
            canvas.delete('pathLength2')
            canvas.create_text(50,85,anchor="w",\
                           text='Path Length: '+str(self.pathLength),\
                           tags='pathLength2')
        
            runTime = time.time() - self.startTime 
            canvas.delete('runTime2')
            canvas.create_text(50, 100, anchor="w",
                           text='Run Time: '+str(runTime),
                           tags='runTime2')

        elif self.brain.controlType == 'fuzzy' :    
            canvas.delete('pathName3')
            canvas.create_text(50,130,anchor="w",\
                           text=self.brain.controlType+':',\
                           tags='pathName3')
            
            canvas.delete('pathLength3')
            canvas.create_text(50,145,anchor="w",\
                           text='Path Length: '+str(self.pathLength),\
                           tags='pathLength3')
        
            runTime = time.time() - self.startTime 
            canvas.delete('runTime3')
            canvas.create_text(50, 160, anchor="w",
                           text='Run Time: '+str(runTime),
                           tags='runTime3')

class Dirt:
    def __init__(self,namep,xx,yy):
        self.centreX = xx
        self.centreY = yy
        self.name = namep

    def draw(self,canvas):

        if self.brain.controlType == 'direct' :

            body = canvas.create_oval(self.centreX-1,self.centreY-1,\
                                    self.centreX+1,self.centreY+1,\
                                    fill="blue",tags=self.name)

        elif self.brain.controlType == 'PID' :

            body = canvas.create_oval(self.centreX-1,self.centreY-1,\
                                    self.centreX+1,self.centreY+1,\
                                    fill="yellow",tags=self.name)  

        elif self.brain.controlType == 'fuzzy' :   

            body = canvas.create_oval(self.centreX-1,self.centreY-1,\
                                    self.centreX+1,self.centreY+1,\
                                    fill="green",tags=self.name)        

    def getLocation(self):
        return self.centreX, self.centreY

class Wall:
    def __init__(self,namep, f_symbol,x1,x2):

        self.x = sp.Symbol('x')
        self.f_symbol = f_symbol
        self.f_x = sp.lambdify(self.x, self.f_symbol, modules=['numpy'])

        #Start point
        self.x1 = x1
        self.y1 = self.f_x(x1)
        
        #End point
        self.x2 = x2
        self.y2 = self.f_x(x2)

        self.name = namep

    def draw(self,canvas, numPoints = 2000):

        x = np.linspace(self.x1, self.x2, numPoints)
        y = np.zeros(len(x))

        for i in range(len(x)):
            y[i] = self.f_x(x[i])

        listOfPoints = []
        for x1, y1 in zip(x, y):
            listOfPoints.extend([x1, y1])

        canvas.create_line(listOfPoints, fill='black', width=5, tags=self.name, smooth=True)
        # canvas.create_line(self.x1, self.y1, self.x2, self.y2, fill = 'black', width = 5, tags = self.name)

    def getLocation(self):
        return self.x1, self.y1, self.x2, self.y2
    
    def getFunction(self):
        return self.f_symbol

class directControl:
    def __init__(self, namep):
        self.name = namep

    def directControlWallFollowing(self, leftSensor1Output, leftSensor2Output, adjust = 0.1, targetDistance = 50, base_speed=5):

        errorSensor = leftSensor1Output - leftSensor2Output
        errorDistance = targetDistance - (leftSensor1Output + leftSensor2Output) / 2

        error = errorSensor + 0.1 * errorDistance

        directOutput = error * adjust

        speedLeft = base_speed + directOutput
        speedRight = base_speed - directOutput

        return speedLeft, speedRight


class PIDControl:
    def __init__(self,namep, Pp=0.1, Ip=0, Dp=0.1):
        self.name = namep

        self.P = Pp
        self.I = Ip
        self.D = Dp

        self.preError = 0
        self.integral = 0

    def PIDWallFollowing(self, leftSensor1Output, leftSensor2Output, targetDistance = 50, base_speed=5):

        errorSensor = leftSensor1Output - leftSensor2Output
        errorDistance = targetDistance - (leftSensor1Output + leftSensor2Output) / 2

        error = errorSensor + 0.1 * errorDistance

        self.integral += error
        d = error - self.preError
        self.preError = error

        PIDValue = self.P * error + self.I * self.integral + self.D * d

        speedLeft = base_speed + PIDValue
        speedRight = base_speed - PIDValue

        return speedLeft, speedRight

class fuzzyControl:
    def __init__(self,namep):
        self.name = namep

        # Define fuzzy set
        largest_dis = np.floor(np.sqrt(1700**2 + 1000**2))
        self.leftSensor1Set = ctrl.Antecedent(np.arange(0, largest_dis, 1), 'leftSensor1')
        self.leftSensor2Set = ctrl.Antecedent(np.arange(0, largest_dis, 1), 'leftSensor2')
        self.output_speedDiff = ctrl.Consequent(np.arange(-3, 4, 0.01), 'speed_diff')
        # self.turnIntent = ctrl.Consequent(np.arange(-1, 1.01, 0.01), 'turn_intent')

        # Anticident
        self.leftSensor1Set['near'] = fuzz.trapmf(self.leftSensor1Set.universe, [0, 0, 30, 40])
        # self.leftSensor1Set['good'] = fuzz.trimf(self.leftSensor1Set.universe, [40, 60, 80])
        self.leftSensor1Set['good'] = fuzz.trapmf(self.leftSensor1Set.universe, [30, 40, 60, 70])
        self.leftSensor1Set['far'] = fuzz.trapmf(self.leftSensor1Set.universe, [60, 70, largest_dis, largest_dis])

        self.leftSensor2Set['near'] = fuzz.trapmf(self.leftSensor2Set.universe, [0, 0, 30, 40])
        # self.leftSensor2Set['good'] = fuzz.trimf(self.leftSensor2Set.universe, [40, 60, 80])
        self.leftSensor2Set['good'] = fuzz.trapmf(self.leftSensor2Set.universe, [30, 40, 60, 70])
        self.leftSensor2Set['far'] = fuzz.trapmf(self.leftSensor2Set.universe, [60, 70, largest_dis, largest_dis])

        # Consequence
        # self.output_speedDiff['left']  = fuzz.trimf(self.output_speedDiff.universe, [-2, -1.5, 0])
        # self.output_speedDiff['straight']  = fuzz.trimf(self.output_speedDiff.universe, [-0.5, 0, 0.5])
        # self.output_speedDiff['right'] = fuzz.trimf(self.output_speedDiff.universe, [0, 1.5, 2])

        self.output_speedDiff['left'] = fuzz.trapmf(self.output_speedDiff.universe, [-5, -5, -2, -1])
        self.output_speedDiff['softleft'] = fuzz.trapmf(self.output_speedDiff.universe, [-2, -1, -0.2, -0.1])
        self.output_speedDiff['straight'] = fuzz.trapmf(self.output_speedDiff.universe, [-0.2, -0.1, 0.1, 0.2])
        self.output_speedDiff['softright'] = fuzz.trapmf(self.output_speedDiff.universe, [0.1, 0.2, 1, 2])
        self.output_speedDiff['right'] = fuzz.trapmf(self.output_speedDiff.universe, [1, 2, 5, 5])

        # self.rules = [
        #     ctrl.Rule(self.leftSensor1Set['near'] & self.leftSensor2Set['far'], self.output_speedDiff['right']),
        #     ctrl.Rule(self.leftSensor1Set['far'] & self.leftSensor2Set['near'], self.output_speedDiff['left']),
        #     # ctrl.Rule(self.leftSensor1Set['good'] & self.leftSensor2Set['near'], self.output_speedDiff['left']),
        #     ctrl.Rule(self.leftSensor1Set['far'] & self.leftSensor2Set['near'], self.output_speedDiff['left']),
        #     ctrl.Rule(self.leftSensor1Set['near'] & self.leftSensor2Set['near'], self.output_speedDiff['right']),
        #     ctrl.Rule(self.leftSensor1Set['far'] & self.leftSensor2Set['far'], self.output_speedDiff['zero']),
        #     ctrl.Rule(self.leftSensor1Set['good'] & self.leftSensor2Set['good'], self.output_speedDiff['zero']),
        #         ]

        self.rules = [
        ctrl.Rule(self.leftSensor1Set['near'] & self.leftSensor2Set['near'], self.output_speedDiff['right']),
        ctrl.Rule(self.leftSensor1Set['near'] & self.leftSensor2Set['good'], self.output_speedDiff['softleft']),
        ctrl.Rule(self.leftSensor1Set['near'] & self.leftSensor2Set['far'], self.output_speedDiff['left']),

        ctrl.Rule(self.leftSensor1Set['good'] & self.leftSensor2Set['near'], self.output_speedDiff['softright']),
        ctrl.Rule(self.leftSensor1Set['good'] & self.leftSensor2Set['good'], self.output_speedDiff['straight']),
        ctrl.Rule(self.leftSensor1Set['good'] & self.leftSensor2Set['far'], self.output_speedDiff['softleft']),

        ctrl.Rule(self.leftSensor1Set['far'] & self.leftSensor2Set['near'], self.output_speedDiff['right']),
        ctrl.Rule(self.leftSensor1Set['far'] & self.leftSensor2Set['good'], self.output_speedDiff['softright']),
        ctrl.Rule(self.leftSensor1Set['far'] & self.leftSensor2Set['far'], self.output_speedDiff['softleft']),
            ]
        
        self.fuzzyControl = ctrl.ControlSystem(self.rules)
        self.fuzzyControl_sim = ctrl.ControlSystemSimulation(self.fuzzyControl)

    def fuzzyWallFollowing(self, leftSensor1Output, leftSensor2Output, base_speed=5):
        self.fuzzyControl_sim.input['leftSensor1'] = leftSensor1Output
        self.fuzzyControl_sim.input['leftSensor2'] = leftSensor2Output
        self.fuzzyControl_sim.compute()

        fuzzyOutput = self.fuzzyControl_sim.output['speed_diff']
        # print(speedDiff)
        speedLeft = base_speed + fuzzyOutput
        speedRight = base_speed - fuzzyOutput

        return speedLeft, speedRight

# This is the adjust fuzzy control
class fuzzyControl_compare:
    def __init__(self,namep):
        self.name = namep

        # Define fuzzy set
        largest_dis = np.floor(np.sqrt(1700**2 + 1000**2))
        self.leftSensor1Set = ctrl.Antecedent(np.arange(0, largest_dis, 1), 'leftSensor1')
        self.leftSensor2Set = ctrl.Antecedent(np.arange(0, largest_dis, 1), 'leftSensor2')
        self.output_speedDiff = ctrl.Consequent(np.arange(-3, 4, 0.01), 'speed_diff')
        # self.turnIntent = ctrl.Consequent(np.arange(-1, 1.01, 0.01), 'turn_intent')

        # Anticident
        self.leftSensor1Set['near'] = fuzz.trapmf(self.leftSensor1Set.universe, [0, 0, 30, 40])
        # self.leftSensor1Set['good'] = fuzz.trimf(self.leftSensor1Set.universe, [40, 60, 80])
        self.leftSensor1Set['good'] = fuzz.trapmf(self.leftSensor1Set.universe, [30, 40, 60, 70])
        self.leftSensor1Set['far'] = fuzz.trapmf(self.leftSensor1Set.universe, [60, 70, largest_dis, largest_dis])

        self.leftSensor2Set['near'] = fuzz.trapmf(self.leftSensor2Set.universe, [0, 0, 30, 40])
        # self.leftSensor2Set['good'] = fuzz.trimf(self.leftSensor2Set.universe, [40, 60, 80])
        self.leftSensor2Set['good'] = fuzz.trapmf(self.leftSensor2Set.universe, [30, 40, 60, 70])
        self.leftSensor2Set['far'] = fuzz.trapmf(self.leftSensor2Set.universe, [60, 70, largest_dis, largest_dis])

        # Consequence
        self.output_speedDiff['left'] = fuzz.trapmf(self.output_speedDiff.universe, [-3, -3, -2, -1])
        self.output_speedDiff['softleft'] = fuzz.trapmf(self.output_speedDiff.universe, [-2, -1, -0.2, -0.1])
        self.output_speedDiff['straight'] = fuzz.trapmf(self.output_speedDiff.universe, [-0.2, -0.1, 0.1, 0.2])
        self.output_speedDiff['softright'] = fuzz.trapmf(self.output_speedDiff.universe, [0.1, 0.2, 1, 2])
        self.output_speedDiff['right'] = fuzz.trapmf(self.output_speedDiff.universe, [1, 2, 3, 3])

        self.rules = [
        ctrl.Rule(self.leftSensor1Set['near'] & self.leftSensor2Set['near'], self.output_speedDiff['right']),
        ctrl.Rule(self.leftSensor1Set['near'] & self.leftSensor2Set['good'], self.output_speedDiff['softleft']),
        ctrl.Rule(self.leftSensor1Set['near'] & self.leftSensor2Set['far'], self.output_speedDiff['left']),

        ctrl.Rule(self.leftSensor1Set['good'] & self.leftSensor2Set['near'], self.output_speedDiff['softright']),
        ctrl.Rule(self.leftSensor1Set['good'] & self.leftSensor2Set['good'], self.output_speedDiff['straight']),
        ctrl.Rule(self.leftSensor1Set['good'] & self.leftSensor2Set['far'], self.output_speedDiff['softleft']),

        ctrl.Rule(self.leftSensor1Set['far'] & self.leftSensor2Set['near'], self.output_speedDiff['right']),
        ctrl.Rule(self.leftSensor1Set['far'] & self.leftSensor2Set['good'], self.output_speedDiff['softright']),
        ctrl.Rule(self.leftSensor1Set['far'] & self.leftSensor2Set['far'], self.output_speedDiff['softleft']),
            ]

        self.fuzzyControl = ctrl.ControlSystem(self.rules)
        self.fuzzyControl_sim = ctrl.ControlSystemSimulation(self.fuzzyControl)

    def fuzzyWallFollowing(self, leftSensor1Output, leftSensor2Output, base_speed=5):
        self.fuzzyControl_sim.input['leftSensor1'] = leftSensor1Output
        self.fuzzyControl_sim.input['leftSensor2'] = leftSensor2Output
        self.fuzzyControl_sim.compute()

        fuzzyOutput = self.fuzzyControl_sim.output['speed_diff']
        # print(speedDiff)
        speedLeft = base_speed + fuzzyOutput
        speedRight = base_speed - fuzzyOutput

        return speedLeft, speedRight

class Counter:
    def __init__(self):
        self.dirtCollected = 0

    def itemCollected(self,canvas):
        self.dirtCollected += 1
        canvas.delete("dirtCount")
        canvas.create_text(50,50,anchor="w",\
                           text="Dirt collected: "+str(self.dirtCollected),\
                           tags="dirtCount")
    
def initialise(window):
    # window.resizable(False,False)
    canvas = tk.Canvas(window,width=1700,height=1000)
    canvas.pack()
    return canvas

def buttonClicked(x,y,agents):
    for rr in agents:
        if isinstance(rr,Bot):
            rr.x = x
            rr.y = y

def createObjects(canvas):
    agents = []
    passiveObjects = []

    symbol_x = sp.Symbol('x')
    #Straight line
    wall1_function = 0 * symbol_x + 300 
    f_wall1_function = sp.lambdify(symbol_x, wall1_function, modules=['numpy'])

    #Quadratic function
    wall1_function2 = 0.001 * (symbol_x - 800) ** 2 + 50

    wall1_function3 = 0.000001 * ((symbol_x - 400) ** 2) * (symbol_x - 1300) + 600

    #Piecewise function
    wall1_function4 = sp.Piecewise (
        (0 * symbol_x + 750, (symbol_x >= 0) & (symbol_x < 400)), 
        ((-symbol_x + 400) + 750, (symbol_x >= 400) & (symbol_x < 800)),
        ((symbol_x - 800) + 350, (symbol_x >= 800) & (symbol_x < 1200)),
        (0 * symbol_x + 750, (symbol_x < 1700)), 
    )

    wall1_function5 = sp.Piecewise (
        (0 * symbol_x + 500, (symbol_x >= 0) & (symbol_x < 200)), 
        ((-symbol_x + 200) + 500, (symbol_x >= 200) & (symbol_x < 500)),
        ((symbol_x - 500) + 200, (symbol_x >= 500) & (symbol_x < 1100)),
        ((-symbol_x + 1100) + 800, (symbol_x >= 1100) & (symbol_x < 1400)),
        (0 * symbol_x + 500, (symbol_x < 1700)), 
    )

    # wall1 = Wall('wall1',wall1_function, 50,f_wall1_function(50),1650,f_wall1_function(1650))
    wall1 = Wall('wall1',wall1_function, 50,1650)

    passiveObjects.append(wall1)
    wall1.draw(canvas)

    count = Counter()
    
    # place Bot
    # theta = random.uniform(-math.pi / 6,math.pi / 6)
    theta = -math.pi / 4

    bot = Bot("Bot1",theta,passiveObjects,count)

    directControl1 = directControl('directWallFollow1')    
    PIDControl1 = PIDControl('PIDWallFollow1',Pp=0.2, Ip=0.01, Dp=0.1)
    fuzzyControl1 = fuzzyControl('fuzzyWallFollow1')

    fuzzyControl2 = fuzzyControl_compare('fuzzyWallFollow2')

    ##test between fuzzy
    # brain = Brain(bot,directControl1, PIDControl1, fuzzyControl1, controlTypep='fuzzy')
    # bot.setBrain(brain)
    # agents.append(bot)
    # bot.draw(canvas)

    # bot = Bot("Bot2",theta,passiveObjects,count)

    # brain = Brain(bot,directControl1, PIDControl1, fuzzyControl2, controlTypep='PID')
    # bot.setBrain(brain)
    # agents.append(bot)
    # bot.draw(canvas)

    #test between all types
    bot = Bot("Bot1",theta,passiveObjects,count)

    directControl1 = directControl('directWallFollow1')    
    PIDControl1 = PIDControl('PIDWallFollow1',Pp=0.2, Ip=0.01, Dp=0.1)
    fuzzyControl1 = fuzzyControl('fuzzyWallFollow1')

    brain = Brain(bot,directControl1, PIDControl1, fuzzyControl1, controlTypep='direct')
    bot.setBrain(brain)
    agents.append(bot)
    bot.draw(canvas)

    bot = Bot("Bot2",theta,passiveObjects,count)

    brain = Brain(bot,directControl1, PIDControl1, fuzzyControl1, controlTypep='PID')
    bot.setBrain(brain)
    agents.append(bot)
    bot.draw(canvas)


    bot = Bot("Bot3",theta,passiveObjects,count)

    brain = Brain(bot,directControl1, PIDControl1, fuzzyControl1, controlTypep='fuzzy')
    bot.setBrain(brain)
    agents.append(bot)
    bot.draw(canvas)

    # canvas.bind( "<Button-1>", lambda event: buttonClicked(event.x,event.y,agents) )


    return agents, passiveObjects, count

def createSingleBot(canvas,controlType,Pp=0.2,Ip=0,Dp=0.1):
    agents = []
    passiveObjects = []

    symbol_x = sp.Symbol('x')
    wall1_function = 0 * symbol_x + 300 
    f_wall1_function = sp.lambdify(symbol_x, wall1_function, modules=['numpy'])

    wall1_function2 = 0.001 * (symbol_x - 800) ** 2 + 50

    wall1_function3 = 0.0000025 * ((symbol_x - 400) ** 2) * (symbol_x - 1300) + 600

    wall1_function4 = sp.Piecewise (
        (0 * symbol_x + 750, (symbol_x >= 0) & (symbol_x < 400)), 
        ((-symbol_x + 400) + 750, (symbol_x >= 400) & (symbol_x < 800)),
        ((symbol_x - 800) + 350, (symbol_x >= 800) & (symbol_x < 1200)),
        (0 * symbol_x + 750, (symbol_x < 1700)), 
    )

    # wall1 = Wall('wall1',wall1_function, 50,f_wall1_function(50),1650,f_wall1_function(1650))
    wall1 = Wall('wall1',wall1_function2, 50,1650)

    passiveObjects.append(wall1)
    wall1.draw(canvas)

    count = Counter()
    
    # place Bot
    # theta = random.uniform(-math.pi / 6,math.pi / 6)
    theta = -math.pi / 4

    #all-types
    # bot = Bot("Bot1",theta,passiveObjects,count)

    # directControl1 = directControl('directWallFollow1')    
    # PIDControl1 = PIDControl('PIDWallFollow1',Pp, Ip, Dp)
    # fuzzyControl1 = fuzzyControl('fuzzyWallFollow1')

    # brain = Brain(bot,directControl1, PIDControl1, fuzzyControl1, controlTypep=controlType)
    # bot.setBrain(brain)
    # agents.append(bot)
    # bot.draw(canvas)

    #adjust_fuzzy
    bot = Bot("Bot1",theta,passiveObjects,count)

    directControl1 = directControl('directWallFollow1')    
    PIDControl1 = PIDControl('PIDWallFollow1',Pp, Ip, Dp)
    fuzzyControl1 = fuzzyControl_compare('fuzzyWallFollow1')

    brain = Brain(bot,directControl1, PIDControl1, fuzzyControl1, controlTypep=controlType)
    bot.setBrain(brain)
    agents.append(bot)
    bot.draw(canvas)

    # canvas.bind( "<Button-1>", lambda event: buttonClicked(event.x,event.y,agents) )

    return agents, passiveObjects, count

def moveIt(canvas,agents,passiveObjects,count):
    for rr in agents:
        rr.thinkAndAct(agents,passiveObjects)
        rr.update(canvas,passiveObjects,1.0)
        passiveObjects = rr.collectDirt(canvas,passiveObjects,count)

        if rr.brain.stopFlag == 0 :
            rr.plotBotInfo(canvas)
        else :
            return

    canvas.after(50,moveIt,canvas,agents,passiveObjects,count)

def moveIt2(canvas,agents,passiveObjects,count,window):
    for rr in agents:
        rr.thinkAndAct(agents,passiveObjects)
        rr.update(canvas,passiveObjects,1.0)
        passiveObjects = rr.collectDirt(canvas,passiveObjects,count)

        if rr.brain.stopFlag == 0 :
            rr.plotBotInfo(canvas)
        else :
            window.destroy()
            return

    canvas.after(50,moveIt2,canvas,agents,passiveObjects,count,window)

def simulate():
    window = tk.Tk()
    canvas = initialise(window)
    agents, passiveObjects, count = createObjects(canvas)
    moveIt(canvas,agents,passiveObjects,count)

    window.mainloop()

def optimize(controlTypep,Pp,Ip,Dp):
    window = tk.Tk()
    canvas = initialise(window)
    agents, passiveObjects, count = createSingleBot(canvas,controlTypep,Pp,Ip,Dp)
    moveIt2(canvas,agents,passiveObjects,count, window)

    window.mainloop()

    return agents[0].pathLength, agents[0].runTime

#get output by single robot simulation
# output = np.zeros((6,10))

# for i in range(10) :
#     # output[0][i], output[1][i] = optimize('direct',0.2,0,0.1)
#     # output[2][i], output[3][i] = optimize('PID',0.2,0,0.1)
#     # output[4][i], output[5][i] = optimize('fuzzy',0.2,0,0.1)

#     output[0][i], output[1][i] = optimize('fuzzy',0.2,0,0.1)

# np.savetxt("Output.csv", output, delimiter=",")

#direc compare the performance
simulate()
