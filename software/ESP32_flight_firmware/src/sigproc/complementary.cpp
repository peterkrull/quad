#include "complementary.h"
#include "sigProc.h"

complementaryFilter::complementaryFilter(float alpha) {
    xAlpha = alpha;
}

bool complementaryFilter::isInitialized() {
    return isInit;
}

float complementaryFilter::update(float slow, float fast, float Td){
    output = ( output + fast*Td )*xAlpha + slow*(1-xAlpha);
    return output;
}

complementaryFilterAntiDrift::complementaryFilterAntiDrift(float alpha) {
    comp = complementaryFilter(alpha);
}

float complementaryFilterAntiDrift::update(float slow, float fast, float Td) {
    return 0.0;
}


/*

a = T/(T+self.tau)
a*(T+self.tau) = T
self.tau = (T-a*T)/a

*/

/*
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
        
        if self.tau:
            diff = self.lp_comp.update(self.int) - self.lp_slow.update(slow)
            return self.int - diff
        else:
            return self.int
*/