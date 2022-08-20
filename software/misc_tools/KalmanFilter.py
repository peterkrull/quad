from numpy import matrix,identity
from numpy.linalg import pinv

class KalmanFilter:
       
    x_pos : matrix = None
    P_pos : matrix = None
    
    observation_models = {}
    
    def __init__(self,F:matrix,Q:matrix,R:matrix,x_init:matrix,P_init:matrix):
        self.F = F
        self.Q = Q
        self.R = R
        self.x_prime = x_init
        self.P_prime = P_init
    
    def add_observation_model(self,ID:str,H:matrix):
        self.observation_models.update({ID:H})
    
    def predict (self,Td:float):
        
        # If new measurements have updated x and P
        if type(self.x_pos) != type(None) and type(self.P_pos) != type(None):
            
            self.P_pos = self.P_pos.dot(self.P_prime)
            
            self.x_prime = self.F.dot(self.x_pos)
            self.P_prime = self.F.dot(self.P_pos).dot(self.F.transpose()) + self.Q
            self.x_pos = None
            self.P_pos = None
            
        # Else do simple prediction
        else:
            self.x_prime = self.F.dot(self.x_prime)
            self.P_prime = self.F.dot(self.P_prime).dot(self.F.transpose()) + self.Q
            
        # TODO innovate Q to get optimal system covariances
            
        return [float(i) for i in self.x_prime]
    
    def update (self, ID, Z):
        
        # Fetch observation model for this sensor
        H = self.observation_models.get(ID)
        
        # Calculate some variables
        y = Z - H.dot(self.x_prime)
        S = H.dot(self.P_prime).dot(H.transpose()) + self.R
        K = self.P_prime.dot(H.transpose()).dot(pinv(S))

        # Update state and 
        if type(self.x_pos) == type(None):
            self.x_pos = self.x_prime + K.dot(y)
            self.P_pos = identity(self.F.shape[0]) - K.dot(H)
        else:
            self.x_pos += K.dot(y)
            self.P_pos -= K.dot(H)
            
        # v = y - H.dot()