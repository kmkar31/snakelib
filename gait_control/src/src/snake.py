# !/usr/bin/env python

import rospy
import numpy as np

## Defines a class Snake that represents the current state and motion of the snake robot at all times


class Snake():
    def __init__(self,gaitType, snakeType) -> None:
        self.N = rospy.get_param('/N') # N represents the number of joints
        self.gaitType = gaitType # Name does not have a slash as required to index parameter names. (Ex: "sidewinding" not "/sidewinding")
        self.snakeType = snakeType # ReU, SEA, Rsnake etc ...
        self.L = rospy.get_param('/L', 0.065)
        self.VC_module = 8

        self.invert = 1
        self.sigma_o = self.parse()
        self.sigma_d = self.parse()
        self.sigma_d_dot = dict(zip(self.sigma_d.keys(), [0 for i in range(len(self.sigma_d))]))
        self.shapeForce = 0

        self.shapeParams = ["A_odd", "A_even", "wS_odd", "wS_even", "wT_odd", "wT_even", "beta_odd", "beta_even", "delta"]
        self.compliant_params = [key for key in list(rospy.get_param("/comply_param").keys()) if rospy.get_param("/comply_param")[key] == 1]

        # Execute the initial serpenoid curve
        self.JA = self.serpenoid(0)
    
    def transition(self,gaitTypeNew, invert):
        self.gaitType = gaitTypeNew

        self.sigma_o = self.parse()
        self.sigma_d = self.parse()
        self.sigma_d_dot = dict(zip(self.sigma_d.keys(), [0 for i in range(len(self.sigma_d))]))

        self.invert = invert

    def resetJoints(self):
        return np.zeros((self.N,1))

    def update(self,t): # Send the state of the snake at query time
        self.JA = self.serpenoid(t)
        #print(np.shape(self.JA_abs))
        return self.JA

    def serpenoid(self,t, sigma=None): # Computes the relative angle vector of the snakebot at time t
        rel_ang = np.zeros(self.N)

        # Even joints actuate in the vertical plane and Odd joints actuate in the horizontal plane

        # Parse Parameters
        if sigma is None:
            snake_params = self.sigma_o
        else:
            snake_params = sigma

        for i in range(self.N):
            if (i+1)%2==0 : # Even Joint
                rel_ang[i] = self.serpenoid_computation((i+1),snake_params["beta_even"],snake_params["A_even"],
                                                        snake_params["wS_even"],snake_params["wT_even"],snake_params["delta"],t)
            else:
                rel_ang[i] = self.serpenoid_computation((i+1),snake_params["beta_odd"],snake_params["A_odd"],
                                                        snake_params["wS_odd"],snake_params["wT_odd"],0,t) # delta doesn't exist for the odd wave
        #rel_ang[0] = -rel_ang[2]
        #self.VirtualChassis(rel_ang)
        #print(self.VC_module)
        return rel_ang
    
    def parse(self): # Iterates through the parameter server and creates a dictionary
        param_dict = dict()
        if self.gaitType in ["linear_progression","lateral_undulation","sidewinding","rolling","turn_in_place","slithering",
                             "rolling_helix","differential_turning","reversal_turning","stationary","Custom"]:
            param_dict = rospy.get_param("/" + self.gaitType)
        else:
            raise Exception("Unknown Gait commanded. Use Custom gait instead to try new gaits")        

        return param_dict


    def serpenoid_computation(self,n,beta,A,wS,wT,delta,t): # Calculates the array of angles taken by
        if self.gaitType == 'differential_turning' and (n%2):
            angle = (beta + A*n*np.sin(wS*(self.N - n) + self.invert*wT*t + delta))*pow(-1,np.floor(n/2))
        else:
            angle = (beta + A*np.sin(wS*(self.N - n) + self.invert*wT*t + delta))*pow(-1,np.floor(n/2))
        return angle

    def Jacobian(self, t):
        # The compliant parameters have to be parsed from parameters:
        compliant_params = rospy.get_param("/comply_param")
        # [A_odd, A_even, ws_odd, ws_even, wT_odd, wT_even, beta_even, beta_odd, delta].T => 9 element vector
        J = np.zeros((self.N, 9))
        for i in range(self.N):
            J[i,0] = ((i+1)%2!=0)*compliant_params["A_odd"]*(self.sigma_o["A_odd"]!=0)*self.serpenoid_computation((i+1),0,1,self.sigma_o["wS_odd"],
                                                                                                                  self.sigma_o["wT_odd"],0,t)

            J[i,1] = ((i+1)%2==0)*compliant_params["A_even"]*(self.sigma_o["A_even"]!=0)*self.serpenoid_computation((i+1),0,1,self.sigma_o["wS_even"],
                                                                                        self.sigma_o["wT_even"],self.sigma_o["delta"],t)
            
            J[i,2] = ((i+1)%2!=0)*(self.N-(i+1))*compliant_params["wS_odd"]*(self.sigma_o["wS_odd"]!=0)*self.serpenoid_computation((i+1),0,self.sigma_o["A_odd"],
                                                                    self.sigma_o["wS_odd"],self.sigma_o["wT_odd"],np.pi/2,t) # add pi/2 to convert sin to cos
            
            J[i,3] = ((i+1)%2==0)*(i+1)*compliant_params["wS_even"]*(self.sigma_o["wS_even"]!=0)*self.serpenoid_computation((i+1),0,self.sigma_o["A_even"],self.sigma_o["wS_even"],
                                                                                               self.sigma_o["wT_even"],self.sigma_o["delta"]+np.pi/2,t)
            
            # For temporal frequencies, we consider the compliant parameter to be wT*t = temporal phase. 
            # Otherwise,  when differentiating, we get a time dependence on the Jacobian i.e., the snake complies more as time goes by which doesnt make sense.
            J[i,4] = ((i+1)%2!=0)*compliant_params["wT_odd"]*(self.sigma_o["wT_odd"]!=0)*self.serpenoid_computation((i+1),0,self.sigma_o["A_odd"],self.sigma_o["wS_odd"],
                                                                                          self.sigma_o["wT_odd"],np.pi/2,t)
            
            J[i,5] = ((i+1)%2==0)*compliant_params["wT_even"]*(self.sigma_o["wT_even"]!=0)*self.serpenoid_computation((i+1),0,self.sigma_o["A_even"],self.sigma_o["wS_even"],
                                                                                           self.sigma_o["wT_even"],self.sigma_o["delta"]+np.pi/2,t)
            
            J[i,6] = ((i+1)%2!=0)*1*compliant_params["beta_odd"]*(self.sigma_o["beta_odd"]!=0)

            J[i,7] = ((i+1)%2==0)*1*compliant_params["beta_even"]*(self.sigma_o["beta_even"]!=0)

            J[i,8] = ((i+1)%2==0)*compliant_params["delta"]*(self.sigma_o["delta"]!=0)*self.serpenoid_computation((i+1),0,self.sigma_o["A_even"],
                                                                    self.sigma_o["wS_even"],self.sigma_o["wT_even"],self.sigma_o["delta"]+np.pi/2,t)
        #print(J)
        return J

    def comply(self, tau_ext, t):
        # sigma_o represents the nominal shape parameters i.e., ones commanded by open loop gaits - DICTIONARY
        # tau_ext represents the torques on each sensor as obtained by the SEA SNAKE
        # M, B and K are hyperparameters that control sigma_d and sigma_o dynamics (they are not multiplied by J yet)
        # sigma_d_ddot and sigma_d_dot represent the second and first derivatives of sigma_d
    
        M_ii = rospy.get_param("/M_ii")
        B_ii = rospy.get_param("/B_ii")
        K_ii = rospy.get_param("/K_ii")
        if self.snakeType not in ["SEA", "RSnake"]:
            raise Exception("You are trying to do Shape based Compliant Control on a Snake without Torque Sensing")
    

        J = self.Jacobian(t)
        compliant_indices = [self.shapeParams.index(self.compliant_params[i]) for i in range(len(self.compliant_params))]
        J = J[:,compliant_indices]
        tau_ext = -np.reshape(np.array(tau_ext),(-1,1))
        for i in range(len(tau_ext)):
            tau_ext[i] = tau_ext[i]*pow(-1,np.floor((i+1)/2))
        self.shapeForce = J.T@tau_ext

        Ao = np.reshape([self.sigma_o[key] for key in self.compliant_params],(-1,1)) # Nominal Parameters
        Ad = np.reshape([self.sigma_d[key] for key in self.compliant_params],(-1,1)) # Desired parameters
        Ad_dot = np.reshape([self.sigma_d_dot[key] for key in self.compliant_params],(-1,1)) # Desired Parameter Rate

        Md = M_ii*J.T@np.identity(self.N)@J
        Bd = B_ii*J.T@np.identity(self.N)@J
        Kd = K_ii*J.T@np.identity(self.N)@J

        

        dt = 1/rospy.get_param("/Control_Frequency", 100)
        Ad_ddot = np.linalg.pinv(Md)@(self.shapeForce - Kd@(Ad - Ao) - Bd@Ad_dot)
        Ad_dot = Ad_dot + dt*Ad_ddot
        Ad = Ad + Ad_dot*dt
        Ad = np.reshape(Ad,(-1))
        #print(np.shape(Md), np.shape(Bd), np.shape(Kd), np.shape(Ao), np.shape(Ad), np.shape(Ad_dot), np.shape(Ad_ddot))
        #print(Ad_ddot, Ad_dot, Ad)
        # Saturate if necessary
        print(Ad)

        for i,key in enumerate(self.compliant_params):
            self.sigma_d[key] = Ad[i]
            self.sigma_d_dot[key] = Ad_dot[i]
        return self.serpenoid(t, sigma=self.sigma_d)
    
    
'''
    def VirtualChassis(self, rel_angles):
        # Computed the postion of the virtual chassis and gives the module closest to the virtual chassis origin

        # Convert a single vector of relative angles into absolute angles
        Abs_ang = np.zeros((self.N,2)) # First column is angle with X axis and second column is angle with X-Y pplane
        for i in range(self.N):
            if (i+1)%2==0 : # Even Module
                Abs_ang[i][0] = Abs_ang[i-1][0]
                if (i+1)==2:
                    Abs_ang[i][1] = rel_angles[i]*pow(-1,np.floor((i+1)/2))
                else:
                    Abs_ang[i][1] = Abs_ang[i-2][1] + rel_angles[i]*pow(-1,np.floor((i+1)/2))
            else:
                
                if (i+1)==1:
                    Abs_ang[i][0] = rel_angles[i]*pow(-1,np.floor((i+1)/2))
                    Abs_ang[i][1] = 0
                else:
                    Abs_ang[i][1] = Abs_ang[i-1][1]
                    Abs_ang[i][0] = Abs_ang[i-2][0] + rel_angles[i]*pow(-1,np.floor((i+1)/2))
        pseudo_positions = np.zeros((self.N,3))
        for i in range(1,self.N):
            pseudo_positions[i,:] = [self.L*np.cos(Abs_ang[i][1])*np.cos(Abs_ang[i][0]), self.L*np.cos(Abs_ang[i][1])*np.sin(Abs_ang[i][0]),
                               self.L*np.sin(Abs_ang[i][1])]
        pseudo_positions = pseudo_positions - np.mean(pseudo_positions,axis=0)
        [_,_,V] = np.linalg.svd(pseudo_positions)
        true_positions = pseudo_positions@V
        self.VC_origin = np.mean(true_positions, axis=0)
        module_distances = np.zeros(self.N)
        for i in range(self.N):
            module_distances[i] = np.sqrt(np.sum(np.square(true_positions[i,:])))
        self.VC_module = np.argmin(module_distances) + 1
'''     
        

