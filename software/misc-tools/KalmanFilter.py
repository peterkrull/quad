from math import *
from numpy import matrix,identity
from numpy.linalg import pinv
import time

import controllers

def rotation_matrix_2d(x,y, angle:float):
    
    return [ 
        cos(angle)*x - sin(angle)*y ,
        sin(angle)*x + cos(angle)*y ]

class ComplementaryFilter:
    
    def __init__(self,a:float, init_val : float = None, anti_drift_tau = 0.0) -> None:
        self.a = a
        self.int = init_val
        self.lp_comp = None
        self.lp_slow = None
        self.tau = anti_drift_tau
        
    def update(self, slow : float , fast : float , Td : float ):
        
        if type(self.int) == type(None):
            self.int = slow
        
        if type(self.lp_comp) == type(None) and self.tau:
            self.lp_comp = controllers.control.low_pass(self.tau,self.int,Td)
            self.lp_slow = controllers.control.low_pass(self.tau,self.int,Td)
        
        self.int = ( self.int + fast*Td )*self.a + slow*(1-self.a)
        
        if tau:
            diff = self.lp_comp.update(self.int) - self.lp_slow.update(slow)
            return self.int - diff
        else:
            return self.int

class KalmanFilter:
       
    x_pos : matrix = None
    P_pos : matrix = None
    
    observation_models = {}
    
    def __init__(self,F:matrix,Q:matrix,x_init:matrix,P_init:matrix):
        self.F = F
        self.Q = Q
        self.x_prime = x_init
        self.P_prime = P_init
    
    def add_observation_model(self,ID:str,H:matrix, R):
        self.observation_models.update({ID:{"H":H,"R":R}})
    
    def predict (self,Td:float):
        # If new measurements have updated x and P
        if type(self.x_pos) != type(None) and type(self.P_pos) != type(None):
            
            self.P_pos = self.P_pos.dot(self.P_prime)
            
            # Symmetrize P
            self.P_pos = (self.P_pos + self.P_pos.transpose())/2
            
            # print(self.P_pos,"\n\n")
            self.x_prime = self.F.dot(self.x_pos)
            self.P_prime = self.F.dot(self.P_pos).dot(self.F.transpose()) + self.Q
            self.x_pos = None
            self.P_pos = None
            
        # Else do simple prediction
        else:
            self.x_prime = self.F.dot(self.x_prime)
            self.P_prime = self.F.dot(self.P_prime).dot(self.F.transpose()) + self.Q
            
        # TODO innovate Q to get optimal system covariances
        
        try:
            return [float(i) for i in self.x_prime]
        except TypeError:
            return [float(i.transpose()[0]) for i in self.x_prime]
    
    def update (self, ID, Z):
        
        # Fetch observation model for this sensor
        H = self.observation_models.get(ID).get("H")
        R = self.observation_models.get(ID).get("R")
        
        # Calculate some variables
               
        y = Z - H.dot(self.x_prime)
        
        # print(f"{ID = } \n\n {y = } \n\n {H = }")
        S = H.dot(self.P_prime).dot(H.transpose()) + R
        K = self.P_prime.dot(H.transpose()).dot(pinv(S))

        # Update state and 
        if type(self.x_pos) == type(None):
            self.x_pos = self.x_prime + K.dot(y)
            self.P_pos = identity(self.F.shape[0]) - K.dot(H)
        else:
            self.x_pos += K.dot(y)
            self.P_pos -= K.dot(H)
            
        # v = y - H.dot()