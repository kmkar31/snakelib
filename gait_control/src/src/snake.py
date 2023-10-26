# !/usr/bin/env python

import rospy
import numpy as np

## Defines a class Snake that represents the current state and motion of the snake robot at all times


class Snake():
    def __init__(self,gaitType, snakeType) -> None:
        self.N = rospy.get_param('/N') # N represents the number of joints
        self.gaitType = gaitType # Name does not have a slash as required to index parameter names. (Ex: "sidewinding" not "/sidewinding")
        self.snakeType = snakeType # ReU, SEA, Rsnake etc ...
        self.invert = 1
        self.sigma_o = self.parse()
        self.sigma_d = self.parse()
        self.sigma_d_dot = dict(zip(self.sigma_d.keys(), [0 for i in range(len(self.sigma_d))]))
        self.shapeForce = 0

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
            angle = (beta + A*n*np.sin(wS*n + self.invert*wT*t + delta))*pow(-1,np.floor(n/2))
        else:
            angle = (beta + A*np.sin(wS*n + self.invert*wT*t + delta))*pow(-1,np.floor(n/2))
        return angle

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
    
        # For now, assuming only odd Amplitude compliance exists. Update everything for more parameters
        Ao = self.sigma_o["A_odd"] # Nominal Odd Amplitude
        Ad = self.sigma_d["A_odd"] # Desired Odd Amplitude
        Ad_dot = self.sigma_d_dot["A_odd"] # Desired Odd Amplitude Rate

        J = np.zeros((self.N))
        for i in range(self.N):
            if (i+1)%2==0 : # Even Joint
                J[i] = 0
            else:
                J[i] = self.serpenoid_computation((i+1),0,1,
                                                        self.sigma_o["wS_odd"],self.sigma_o["wT_odd"],0,t)
        J = np.reshape(J, (-1,1))
        #print(np.linalg.norm(tau_ext))

        tau_ext = -np.reshape(np.array(tau_ext),(1,-1))
        for i in range(len(tau_ext)):
            tau_ext[i] = tau_ext[i]*pow(-1,np.floor((i+1)/2))
        self.shapeForce = tau_ext@J
        Md = M_ii*J.T@np.identity(self.N)@J
        Bd = B_ii*J.T@np.identity(self.N)@J
        Kd = K_ii*J.T@np.identity(self.N)@J

        dt = 0.01
        Ad_ddot = np.linalg.pinv(Md)*(self.shapeForce - Kd*(Ad - Ao) - Bd*Ad_dot)
        Ad_dot = Ad_dot + dt*Ad_ddot
        Ad = Ad + Ad_dot*dt
        #print(Ad_ddot, Ad_dot, Ad)
        if Ad >=2 or Ad <= -2:
            Ad = np.sign(Ad)*2
            Ad_dot = 0
        self.sigma_d["A_odd"] = Ad
        self.sigma_d_dot["A_odd"] = Ad_dot
        return [self.serpenoid(t, sigma=self.sigma_d), Ad]