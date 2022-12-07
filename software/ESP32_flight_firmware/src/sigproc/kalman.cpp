#include "kalman.h"

StateVector Kalman::predict(float Td) {

    if (newMeasurement) {

        // TODO CALCULATE P_pos 9x9 matrix
        // self.P_pos = self.P_pos.dot(self.P_prime)

        // Do simple prediction if no new measurements
        x_prime.pos.x = x_pos.pos.x + x_pos.vel.x*Td + (1/2)*x_pos.acc.x*pow(Td,2);
        x_prime.vel.x = x_pos.vel.x + x_pos.acc.x*Td;
        x_prime.acc.x = x_pos.acc.x;

        x_prime.pos.y = x_pos.pos.y + x_pos.vel.y*Td + (1/2)*x_pos.acc.y*pow(Td,2);
        x_prime.vel.y = x_pos.vel.y + x_pos.acc.y*Td;
        x_prime.acc.y = x_pos.acc.y;
        
        x_prime.pos.z = x_pos.pos.z + x_pos.vel.z*Td + (1/2)*x_pos.acc.z*pow(Td,2);
        x_prime.vel.z = x_pos.vel.z + x_pos.acc.z*Td;
        x_prime.acc.z = x_pos.acc.z;

        // TODO Calculate P_prime 9x9 matrix (based on POS)
        // self.P_prime = self.F.dot(self.P_pos).dot(self.F.transpose()) + self.Q

        newMeasurement = false;
        memcpy(&x_pos,&x_prime,sizeof(StateVector));
        memcpy(&p_pos,&p_prime,sizeof(StateCovarianceMatrix));
    
    } else {

        // Do simple prediction if no new measurements
        x_prime.pos.x = x_prime.pos.x + x_prime.vel.x*Td + (1/2)*x_prime.acc.x*pow(Td,2);
        x_prime.vel.x = x_prime.vel.x + x_prime.acc.x*Td;
        x_prime.acc.x = x_prime.acc.x;

        x_prime.pos.y = x_prime.pos.y + x_prime.vel.y*Td + (1/2)*x_prime.acc.y*pow(Td,2);
        x_prime.vel.y = x_prime.vel.y + x_prime.acc.y*Td;
        x_prime.acc.y = x_prime.acc.y;
        
        x_prime.pos.z = x_prime.pos.z + x_prime.vel.z*Td + (1/2)*x_prime.acc.z*pow(Td,2);
        x_prime.vel.z = x_prime.vel.z + x_prime.acc.z*Td;
        x_prime.acc.z = x_prime.acc.z;

        // TODO Calculate P_prime 9x9 matrix (based on PRIME)
        // self.P_prime = self.F.dot(self.P_prime).dot(self.F.transpose()) + self.Q

    }

    return x_prime;
}

void Kalman::newAcc(VectorFloat acc, unsigned timestamp) {

    // Calculate error between measurement and previously estimated state
    // VectorFloat y = {acc.x - x_prime.acc.x , acc.y - x_prime.acc.y , acc.z - x_prime.acc.z};

    

}

void Kalman::newPos(VectorFloat pos, unsigned timestamp) {

}

/*

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
        
*/