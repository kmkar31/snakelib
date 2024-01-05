# !/usr/bin/env python

import rospy
import numpy as np
import os

## Defines a class Snake that represents the current state and motion of the snake robot at all times
# Variables names have all words in title case while function names only have the second word onwards in title case

class Snake():
    def __init__(self,gaitType, snakeType) -> None:
        self.N = rospy.get_param('/N') # N represents the number of joints
        self.GaitType = gaitType # Name does not have a slash as required to index parameter names. (Ex: "sidewinding" not "/sidewinding")
        self.SnakeType = snakeType # ReU, SEA, Rsnake etc ...
        self.Invert = 1 # Is the gait moving forwards or backwards
        self.NominalShape = self.parse() # Defines the nominal gait
        self.dt = 1/rospy.get_param("/Control_Frequency", 100)
        self.ShapeParams = ["A_odd", "A_even", "wS_odd", "wS_even", "wT_odd", "wT_even", "beta_odd", "beta_even", "delta"]


        # Execute the initial serpenoid curve
        self.JointAngles = self.globalSerpenoid(0)

        self.ControlMode = rospy.get_param('/control_mode',0) # Control Method to be used
        if self.ControlMode == 0: # Open-Loop Control
            pass
        elif self.ControlMode == 1: # Centralized Compliant Control
            self.centralizedComplianceINIT()
        elif self.ControlMode == 2: # Localized Compliant Control
            self.localizedComplianceINIT()

        # Log Configuration
        self.LogFile= "../" + rospy.get_param("/InternalLog/filedir") + "/" + str(rospy.Time.now()) + ".csv"
    
    '''
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~                                            COMMON FUNCTIONS                                                                         ~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Updates the snake's commanded joint angles as a function time
    '''
    # Send the state of the snake's Joint Angles at query time
    # Should be the only function accessible to functions outside this class
    def update(self,t, ExternalTorque = None):
        if self.ControlMode==0: 
            self.JointAngles = self.globalSerpenoid(t)
        elif self.ControlMode == 1:
            self.JointAngles = self.centralizedCompliance(ExternalTorque, t)
        elif self.ControlMode==2:
            self.JointAngles = self.localizedCompliance(ExternalTorque, t)
        return self.JointAngles
    
    def serpenoid(self,n,beta,A,wS,wT,delta,t): # Calculates the array of angles taken by
        #print(n,beta,A,wS,wT,delta,t)
        if self.GaitType == 'differential_turning' and (n%2):
            angle = (beta + A*n*np.sin(wS*n + self.Invert*wT*t + delta))*pow(-1,np.floor(n/2))
        else:
            angle = (beta + A*np.sin(wS*n + self.Invert*wT*t + delta))*pow(-1,np.floor(n/2))
        return angle
    
    def parse(self): # Iterates through the parameter server and creates a dictionary
        param_dict = dict()
        if self.GaitType in ["linear_progression","lateral_undulation","sidewinding","rolling","turn_in_place","slithering",
                             "rolling_helix","differential_turning","reversal_turning","stationary","Custom"]:
            param_dict = rospy.get_param("/" + self.GaitType)
        else:
            raise Exception("Unknown Gait commanded. Use Custom gait instead to try new gaits")        

        return param_dict

    # Computes the relative angle vector of the snakebot at time t
    def globalSerpenoid(self,t, shape=None):
        Angles = np.zeros(self.N)

        # Even joints actuate in the vertical plane and Odd joints actuate in the horizontal plane
        # Parse Parameters
        if shape is None:
            snakeParams = self.NominalShape
        else:
            snakeParams = shape

        for i in range(self.N):
            if (i+1)%2==0 : # Even Joint
                Angles[i] = self.serpenoid((i+1),snakeParams["beta_even"],snakeParams["A_even"],
                                                        snakeParams["wS_even"],snakeParams["wT_even"],snakeParams["delta"],t)
            else:
                Angles[i] = self.serpenoid((i+1),snakeParams["beta_odd"],snakeParams["A_odd"],
                                                        snakeParams["wS_odd"],snakeParams["wT_odd"],0,t) # delta doesn't exist for the odd wave
        self.JointAngles = Angles
        return Angles
    

    '''
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~                                         CENTRALIZED COMPLIANT CONTROL                                                               ~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Performs global Compliant control for the chosen parameters declared in the compliant_param.yaml file
    '''

    # INIT Function for Centralized Compliance
    def centralizedComplianceINIT(self):
        self.DesiredShape = self.parse()
        self.DesiredShapeRate = dict(zip(self.DesiredShape.keys(), [0 for i in range(len(self.DesiredShape))]))
        self.ShapeForce = 0
        self.CompliantParams = [key for key in list(rospy.get_param("/global_comply").keys()) if rospy.get_param("/global_comply")[key] == 1]

    def centralizedCompliance(self, tau_ext, t):
        # sigma_o represents the nominal shape parameters i.e., ones commanded by open loop gaits - DICTIONARY
        # tau_ext represents the torques on each sensor as obtained by the SEA SNAKE
        # M, B and K are hyperparameters that control sigma_d and sigma_o dynamics (they are not multiplied by J yet)
        # sigma_d_ddot and sigma_d_dot represent the second and first derivatives of sigma_d
    
        M_ii = rospy.get_param("/M_ii")
        B_ii = rospy.get_param("/B_ii")
        K_ii = rospy.get_param("/K_ii")
        if self.SnakeType not in ["SEA", "RSnake"]:
            raise Exception("You are trying to do Shape based Compliant Control on a Snake without Torque Sensing")
    
        J = self.Jacobian(t)
        CompliantIndices = [self.ShapeParams.index(self.CompliantParams[i]) for i in range(len(self.CompliantParams))]
        J = J[:,CompliantIndices]
        tau_ext = -np.reshape(np.array(tau_ext),(-1,1))
        #for i in range(len(tau_ext)):
            #tau_ext[i] = tau_ext[i]*pow(-1,np.floor((i+1)/2))
        self.ShapeForce = J.T@tau_ext
        print(self.ShapeForce)
        if np.abs(self.ShapeForce) <= 1.5:
            self.ShapeForce = [[0]]

        Ao = np.reshape([self.NominalShape[key] for key in self.CompliantParams],(-1,1)) # Nominal Parameters
        Ad = np.reshape([self.DesiredShape[key] for key in self.CompliantParams],(-1,1)) # Desired parameters
        Ad_dot = np.reshape([self.DesiredShapeRate[key] for key in self.CompliantParams],(-1,1)) # Desired Parameter Rate

        Md = M_ii*J.T@np.identity(self.N)@J
        Bd = B_ii*J.T@np.identity(self.N)@J
        Kd = K_ii*J.T@np.identity(self.N)@J
        
        Ad_ddot = np.linalg.pinv(Md)@(self.ShapeForce - Kd@(Ad - Ao) - Bd@Ad_dot)
        Ad_dot = Ad_dot + self.dt*Ad_ddot
        Ad = Ad + Ad_dot*self.dt
        Ad = np.reshape(Ad,(-1))

        for i,key in enumerate(self.CompliantParams):
            self.DesiredShape[key] = Ad[i]
            self.DesiredShapeRate[key] = Ad_dot[i]
        return self.globalSerpenoid(t, shape=self.DesiredShape)
    
    '''
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~                                         LOCALIZED COMPLIANT CONTROL                                                                 ~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Performs localized Compliant control for the chosen parameters declared in the compliant_param.yaml file
    '''

    # INIT Function for Localized Compliance
    def localizedComplianceINIT(self):
        self.WindowStart = [] # Activation Window start and end locations
        self.WindowEnd = []
        self.W = self.N # Number of Windows, Reassigned when window locations are populated
        self.m = 100 # Controls Transition width
        self.Mii = rospy.get_param("/M_ii")
        self.Bii = rospy.get_param("/B_ii")
        self.Kii = rospy.get_param("/K_ii")
        self.dt = 1/rospy.get_param("/Control_Frequency", 100)
        self.DesiredShape = dict(zip(self.ShapeParams, [[self.NominalShape[x] for i in range(self.N)]for x in self.ShapeParams])) 
        self.DesiredShapeRate = dict(zip(self.ShapeParams, [[0 for i in range(self.N)]for j in range(len(self.ShapeParams))]))
        self.CompliantParams = [key for key in list(rospy.get_param("/local_comply").keys()) if rospy.get_param("/local_comply")[key] == 1]
        self.ShapeForce = np.zeros((self.N))

        self.filename = "../" + rospy.get_param("/InternalLog/filedir") + "/" + str(rospy.Time.now()) + ".csv"

    # Runs the Compliant motion controller for a single shape parameter over all windows
    def computeCompliance(self, sigma_o, sigma_d, sigma_d_dot, J, tau):
        sigma_d_new = np.zeros(self.N)
        sigma_d_dot_new = np.zeros(self.N)
        sigma_d_ddot = np.zeros(self.N)
        for i in range(self.N):
            if np.abs(tau[i]) <= 10:
                tau[i] = 0
            elif tau[i] > 10:
                tau[i] = tau[i] - 10
            else:
                tau[i] = tau[i] + 10 
        
        for i,(s,e) in enumerate(zip(self.WindowStart, self.WindowEnd)):
            Jactive = J[s:e+1,:]
            self.ShapeForce[s:e+1] = Jactive.T@tau[s:e+1]
            #print(self.ShapeForce[s:e+1])
            Md = self.Mii*Jactive.T@np.identity(e-s+1)@Jactive
            Bd = self.Bii*Jactive.T@np.identity(e-s+1)@Jactive
            Kd = self.Kii*Jactive.T@np.identity(e-s+1)@Jactive

            stiffness = Kd*(np.reshape(np.subtract(sigma_d[s:e+1],sigma_o),(-1,1)))
            damping = Bd*np.reshape(sigma_d_dot[s:e+1],(-1,1))
            forcing = np.reshape(self.ShapeForce[s:e+1], (-1,1))

            sigma_d_ddot[s:e+1] = np.reshape(np.linalg.pinv(Md)*(forcing - stiffness - damping), (-1))
            sigma_d_dot_new[s:e+1] = sigma_d_dot[s:e+1] + self.dt*sigma_d_ddot[s:e+1]
            sigma_d_new[s:e+1] = sigma_d[s:e+1] + sigma_d_dot_new[s:e+1]*self.dt

            #print(self.ShapeForce)

        return [sigma_d_new, sigma_d_dot_new]
    
    # Creates the start end end window lengths based on points where the serpenoid function is 0
    def setActivationWindows(self, ID):
        # ID depicts if the parameter is an odd parameter or an even parameter
        WindowStart = np.zeros((self.N),dtype=int)
        WindowEnd = np.zeros((self.N), dtype=int)
        for i in range(1,self.N-1):
            if ID == 1: # ODD PARAMETER
                if i%2==0 and (np.sign(self.JointAngles[i]) == -np.sign(self.JointAngles[i-2]) or np.abs(self.JointAngles[i])<=0.002): # Zero Serpenoid Equation module
                    WindowStart[i] = i
                    WindowEnd[np.where(WindowEnd==0)[0][0]:i] = i
                else:
                    WindowStart[i] = WindowStart[i-1]
            elif ID == 2:
                if (i+1)%2==0 and (np.sign(self.JointAngles[i]) == -np.sign(self.JointAngles[i-2]) or np.abs(self.JointAngles[i])<=0.002): # Zero Serpenoid Equation module
                    WindowStart[i] = i
                    WindowEnd[np.where(WindowEnd==0)[0][0]:i] = i
                else:
                    WindowStart[i] = WindowStart[i-1]
        WindowEnd[np.where(WindowEnd==0)[0][0]:self.N] = self.N-1
        WindowStart[self.N-1] = WindowStart[self.N-2]

        self.WindowStart = WindowStart
        self.WindowEnd = WindowEnd
    
    def localizedCompliance(self, tau_ext, t):
        # param is a string that represents the key in the dictionary
        J = self.Jacobian(t,rospy.get_param('/local_comply'))
        tau_ext = -np.reshape(np.array(tau_ext),(-1,1))

        # VARIES WITH DIFFERENT SNAKES. SOME HAVE THE Z-AXIS INVERTED AND SOME DONT
        for i in range(len(tau_ext)):
            tau_ext[i] = tau_ext[i]*pow(-1,np.floor((i+1)/2))

        #self.WindowStart = [0,0,0,0,4,4,4,4,8,8,8,8,12,12,12,12]
        #self.WindowEnd = [3,3,3,3,7,7,7,7,11,11,11,11,15,15,15,15]
        for param in self.CompliantParams:
            if 'odd' in param:
                ID = 1
            else:
                ID = 2
            self.setActivationWindows(ID)
            #print(self.WindowStart, self.WindowEnd)
            Jactive = np.reshape(J[:,self.ShapeParams.index(param)], (-1,1))
            sigma_o = self.NominalShape[param] # Extracts the single parameter to comply
            [sigma_d, sigma_d_dot] = self.computeCompliance(sigma_o, self.DesiredShape[param], self.DesiredShapeRate[param], Jactive, tau_ext)
            #sigma_d = self.gaussian(sigma_d, sigma_o)
            sigma_d = self.sigmoid(sigma_d, sigma_o)
            self.DesiredShape[param] = sigma_d
            self.DesiredShapeRate[param] = sigma_d_dot
           
        self.localSerpenoid(t)
        self.localLog(t)
        print(self.DesiredShape['A_even'])
        return self.JointAngles

    # Performs Gaussian Transition over the given vector
    def gaussian(self, sigma_d, sigma_o):
        sigma_d_corrected = np.zeros((self.N))
        ws = list(np.unique(self.WindowStart))
        we = list(np.unique(self.WindowEnd))
        mu = [np.floor(np.mean([ws[i],we[i]])) for i in range(len(ws))]
        psi = [we[i]-ws[i] for i in range(len(ws))]

        for i in range(self.N):
            sigma_d_corrected[i] = sigma_o + sum([(sigma_d[ws[j]]-sigma_o)*np.exp(-(i-mu[j])**2/psi[j]**2) for j in range(len(ws))])
        return sigma_d_corrected
    
    def sigmoid(self, sigma_d, sigma_o):
        sigma_d_corrected = np.zeros(self.N)
        ws = list(np.unique(self.WindowStart))
        we = list(np.unique(self.WindowEnd))
        
        for i in range(self.N):
            if i != 0 and i!=self.N-1:
                sigma_d_corrected[i] = sum([sigma_d[ws[j]]*(1/(1+np.exp(-self.m*(i-ws[j]))) + 1/(1+np.exp(self.m*(i-we[j]))) - 1) for j in range(len(ws))])
            else:
                sigma_d_corrected[i] = sigma_o
        return sigma_d_corrected

    
    def localSerpenoid(self, t):
        Angles = np.zeros(self.N)
        for i in range(self.N):
            if (i+1)%2==0 : # Even Joint
                Angles[i] = self.serpenoid((i+1),self.DesiredShape["beta_even"][i],self.DesiredShape["A_even"][i],
                                                        self.DesiredShape["wS_even"][i],self.DesiredShape["wT_even"][i],self.DesiredShape["delta"][i],t)
            else:
                Angles[i] = self.serpenoid((i+1),self.DesiredShape["beta_odd"][i],self.DesiredShape["A_odd"][i],
                                                        self.DesiredShape["wS_odd"][i],self.DesiredShape["wT_odd"][i],0,t)
        Angles = np.clip(Angles,-1.5*np.ones(self.N), 1.5*np.ones(self.N))
        self.JointAngles = Angles
    
    def localLog(self, t):
        f = open(self.filename, 'a+')
        line1 = 'Time,'
        line2 = ','
        '''
        if os.path.getsize(self.filename) == 0:
            for key in self.NominalShape:
                line1 = line1 + 
            f.write("Time," + ','.join(list(snake.NominalShape.keys())) + "," + ','.join(list(snake.DesiredShape.keys())) + ",Shape Force" + "\n")
        '''
        f.write(str(t) + ",")
        for key in self.NominalShape:
            if key=='A_odd':
                nom = self.NominalShape[key]
                f.write(str(nom)+ ",") 
                des = list(self.DesiredShape[key])
                for i in range(len(des)):
                    f.write(str(des[i])+",")
        for i in range(self.N):
            f.write(str(self.WindowStart[i])+",") 
        for i in range(self.N):
            f.write(str(self.ShapeForce[i])+",") 
        #f.write(str(self.ShapeForce) + "\n")
        f.write("\n")
        f.close()
    


    '''
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~                                            HUMAN ORDER - GAIT CHANGE                                                                ~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Parses a keyboard command from the user and changes the commanded gait of the snake ---> Works with open-loop and compliant controllers
    '''
    def transition(self,gaitTypeNew, invert):
            self.GaitType = gaitTypeNew

            self.NominalShape = self.parse()
            if self.ControlMode ==1 :
                self.centralizedComplianceINIT()
            if self.ControlMode == 2:
                self.localizedComplianceINIT()
            #self.DesiredShape = self.parse()
            #self.sigma_d_dot = dict(zip(self.sigma_d.keys(), [0 for i in range(len(self.sigma_d))]))

            self.Invert = invert

    def resetJoints(self):
        return np.zeros((self.N,1))
    
    '''
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~                                            AUXILIARY FUNCTIONS                                                                      ~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Auxiliary Functions used by multiple control strategies
    '''
    
    def Jacobian(self, t, compliant_params=None):
        # The compliant parameters have to be parsed from parameters:
        if compliant_params is None:
            compliant_params = rospy.get_param("/global_comply")
        # [A_odd, A_even, ws_odd, ws_even, wT_odd, wT_even, beta_even, beta_odd, delta].T => 9 element vector
        J = np.zeros((self.N, 9))
        for i in range(self.N):
            J[i,0] = ((i+1)%2!=0)*compliant_params["A_odd"]*(self.NominalShape["A_odd"]!=0)*self.serpenoid((i+1),0,1,self.NominalShape["wS_odd"],
                                                                                                                  self.NominalShape["wT_odd"],0,t)

            J[i,1] = ((i+1)%2==0)*compliant_params["A_even"]*(self.NominalShape["A_even"]!=0)*self.serpenoid((i+1),0,1,self.NominalShape["wS_even"],
                                                                                        self.NominalShape["wT_even"],self.NominalShape["delta"],t)
            
            J[i,2] = ((i+1)%2!=0)*(i+1)*compliant_params["wS_odd"]*(self.NominalShape["wS_odd"]!=0)*self.serpenoid((i+1),0,self.NominalShape["A_odd"],
                                                                    self.NominalShape["wS_odd"],self.NominalShape["wT_odd"],np.pi/2,t) # add pi/2 to convert sin to cos
            
            J[i,3] = ((i+1)%2==0)*(i+1)*compliant_params["wS_even"]*(self.NominalShape["wS_even"]!=0)*self.serpenoid((i+1),0,self.NominalShape["A_even"],self.NominalShape["wS_even"],
                                                                                               self.NominalShape["wT_even"],self.NominalShape["delta"]+np.pi/2,t)
            
            # For temporal frequencies, we consider the compliant parameter to be wT*t = temporal phase. 
            # Otherwise,  when differentiating, we get a time dependence on the Jacobian i.e., the snake complies more as time goes by which doesnt make sense.
            J[i,4] = ((i+1)%2!=0)*compliant_params["wT_odd"]*(self.NominalShape["wT_odd"]!=0)*self.serpenoid((i+1),0,self.NominalShape["A_odd"],self.NominalShape["wS_odd"],
                                                                                          self.NominalShape["wT_odd"],np.pi/2,t)
            
            J[i,5] = ((i+1)%2==0)*compliant_params["wT_even"]*(self.NominalShape["wT_even"]!=0)*self.serpenoid((i+1),0,self.NominalShape["A_even"],self.NominalShape["wS_even"],
                                                                                           self.NominalShape["wT_even"],self.NominalShape["delta"]+np.pi/2,t)
            
            J[i,6] = ((i+1)%2!=0)*1*compliant_params["beta_odd"]*(self.NominalShape["beta_odd"]!=0)

            J[i,7] = ((i+1)%2==0)*1*compliant_params["beta_even"]*(self.NominalShape["beta_even"]!=0)

            J[i,8] = ((i+1)%2==0)*compliant_params["delta"]*(self.NominalShape["delta"]!=0)*self.serpenoid((i+1),0,self.NominalShape["A_even"],
                                                                    self.NominalShape["wS_even"],self.NominalShape["wT_even"],self.NominalShape["delta"]+np.pi/2,t)
        #print(J)
        return J  
    
    def log(self):
        pass
        

