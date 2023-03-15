import time

class control:

    """
    Class that contains various methods useful for descrete time control-systems.
    Methods are the `proportional` gain, `derivative` and `integral` functions. as
    well as a complete PID controller and a lead-lag implementation using `lead_lag_comp`.
    A few filters are also present such as `roll_avg` and `low_pass` that can be used
    """

    class low_pass():
        def __init__(self,tau,init_val=0,debug_time=None, rollover_min = None, rollover_max = None):
            """
            First order discrete time low-pass filter

            Args:
                tau (float) : Time constant of filter
                init_val (float) : initial filter value
            """
            self.debug_time = debug_time
            self.tau = tau
            self.y = init_val
            self.prevtime = time.time()
            self.min = rollover_min
            self.max = rollover_max

        def update(self,value):
            """
            Updates the filter with a new input value.

            Args:
                value (float) : input value

            Returns (float) : filtered input
            """

            # Handle rollover on input (shift prev y-value)
            if self.min and self.max:
                if value-self.y < self.min:
                    self.y -= self.max-self.min
                elif value-self.y  > self.max:
                    self.y += self.max-self.min

            # Classic infinite impulse implementation
            if not self.debug_time:
                xtime = time.time()
                T = xtime-self.prevtime
                a = T/(T+self.tau)
                self.y = self.y*(1-a)+value*a
                self.prevtime = xtime
            else: # debug
                T = self.debug_time
                a = T/(T+self.tau)
                self.y = self.y*(1-a)+value*a
            
            # Handle rollover of output (constrain to limits)
            if self.min and self.max:
                if self.y < self.min:
                    return self.y + (self.max-self.min)
                elif self.y > self.max:
                    return self.y - (self.max-self.min)

            return self.y

    class low_pass_bi():
        def __init__(self,tau,init_val=0,debug_time=None, rollover_min = None, rollover_max = None):
            """
            First order discrete time low-pass filter (using bilinear transform)

            Args:
                tau (float) : Time constant of filter
                init_val (float) : initial filter value
            """
            self.debug_time = debug_time
            self.tau = tau
            self.y = init_val
            self.x = 0
            self.prevtime = time.time()
            self.min = rollover_min
            self.max = rollover_max

        def update(self,value):
            """
            Updates the filter with a new input value.

            Args:
                value (float) : input value

            Returns (float) : filtered input
            """

            # Handle rollover on input (shift prev y-value)
            if self.min and self.max:
                if value-self.y < self.min:
                    self.y -= self.max-self.min
                    self.x -= self.max-self.min
            
                elif value-self.y  > self.max:
                    self.y += self.max-self.min
                    self.x += self.max-self.min                    

            # Classic biliniar filter
            if not self.debug_time:
                xtime = time.time()
                T = (xtime-self.prevtime)
                a = (2*self.tau)/T
                self.y = (self.x + value -self.y*(1-a))/(1+a)
                self.x = value
                self.prevtime = xtime
            else: # debug
                T = self.debug_time
                a = (2*self.tau)/T
                self.y = (self.x + value -self.y*(1-a))/(1+a)
                self.x = value
            
            # Handle rollover of output (constrain to limits)
            if self.min and self.max:
                if self.y < self.min:
                    return self.y + (self.max-self.min)
                elif self.y > self.max:
                    return self.y - (self.max-self.min)

            return self.y

    
    class cascade():
        def __init__(self,system,order,**kwargs):
            """
            Allows for multiple of the same controllers or filters
            to be used in series, useful for higher order filters.

            Args:
                system (class) : Controller or filter class with `update()` method
                order (int) : Number of cascaded systems
                **kwargs : Arguments that should be passed on to system class
            """
            self.filter = system
            self.order = order
            self.filters = [system(**kwargs) for _ in range(order)]
            if hasattr(system, 'start'):
                for filter in self.filters:
                    filter.start()


        def update(self,value):
            """
            Updates the entire cascade with a new input value.

            Args:
                value (float) : input value

            Returns (float) : cascaded input
            """
            for filter in self.filters:
                value = filter.update(value) 
            return value

    class roll_avg():

        def __init__(self,nums,init_val = 0, rollover_min = None, rollover_max = None):
            """
            Takes the rolling average of some input value

            Args:
                nums (int) : number of values to take average of
                init_val (float) : initial values for array
            """
            self.min = rollover_min
            self.max = rollover_max
            self.array = [init_val for _ in range(nums)]
            self.nums = nums
            self.current = 0

        def update(self,value):
            """
            Adds a new value to the rolling average

            Args:
                value (float) : input value

            Returns (float) : average of `nums` previous input values
            """

            # Handle rollover if needed
            if self.min and self.max:
                if value-self.array[self.current] < self.min:
                    for i in range(len(self.array)):
                        self.array[i] -= self.max-self.min
                elif value-self.array[self.current]  > self.max:
                    for i in range(len(self.array)):
                        self.array[i] += self.max-self.min

            self.array[self.current] = value
            self.current += 1
            if self.current >= self.nums:
                self.current = 0
            return sum(self.array)/self.nums

    # Proportional gain
    class proportional:
        def __init__(self,K):
            self.K = K

        def update(self,error):
            return error*self.K

    # Derivative gain
    class derivative:
        def __init__(self,K = 1, tau = None, order = None,debug_time = None,rollover_min = None,rollover_max = None,**kwargs):
            
            self.min = rollover_min
            self.max = rollover_max
            self.debug_time = debug_time
            self.order = order
            self.tau = tau
            self.K = K
            
            if self.order and self.tau:
                self.lp = control.cascade(control.low_pass, order, tau=self.tau,debug_time=self.debug_time,**kwargs)
            elif self.tau:
                self.lp = control.low_pass(self.tau,debug_time=self.debug_time,**kwargs)

        def start(self):
            self.prev_time = time.time()
            self.prev_erro = 0

        def update(self,error):

            if self.tau:
                error = self.lp.update(error)

            # Handle rollover if needed
            if self.min and self.max:
                if error-self.prev_erro < self.min:
                    self.prev_erro -= self.max-self.min
                elif error-self.prev_erro  > self.max:
                    self.prev_erro += self.max-self.min

            if not self.debug_time:
                xtime = time.time()
                derivative = ((error-self.prev_erro)*self.K)/(xtime-self.prev_time)
                self.prev_time = xtime
            else:
                derivative = ((error-self.prev_erro)*self.K)/(self.debug_time)
            
            self.prev_erro = error
            return derivative
            
    # Integral gain
    class integral:
        def __init__(self,K = 1,debug_time = None,rollover_min = None,rollover_max = None):
            self.min = rollover_min
            self.max = rollover_max

            self.debug_time = debug_time
            self.K = K
            
        def start(self):
            self.prev_time = time.time()
            self.integral = 0
            self.prev_erro = 0

        def update(self,error):

            # Handle rollover if needed
            if self.min and self.max:
                if error-self.prev_erro < self.min:
                    self.integral  -= self.max-self.min
                elif error-self.prev_erro  > self.max:
                    self.integral  += self.max-self.min

            if not self.debug_time:
                xtime = time.time()
                self.integral += ((error)*self.K)*(xtime-self.prev_time)
                self.prev_time = xtime
            else:
                self.integral += ((error)*self.K)*(self.debug_time)

            self.prev_erro = error

            return self.integral

    # Combined P, I and/or D controller.
    class PID:

        def __init__(self,Kp = None, Ki = None,Kd = None,form = "parallel", **kwargs) -> object:
            """
            Complete PID controller complete PID control, with ability to use
            any combination of P, I and D and user-defined Kp, Ki and Kd constants.


            Args:
                Kp (float) : Proportional constant
                Ki (float) : Integral constant
                Kd (float) : Derivative constant
                form (str) : Output form, supports 'parallel' and 'ideal', see below

            More about 'form' output
                parallel : P + I + D
                ideal  : P * ( 1 + I + D )
            
            """
            
            self.form = form
            self.Kp,self.Ki,self.Kd = Kp,Ki,Kd
            if self.Kp:
                #print("Setting P gain to : {}".format(Kp))
                self.p = control.proportional(Kp)
            if self.Ki:
                #print("Setting I gain to : {}".format(Ki))
                self.i = control.integral(Ki,**kwargs)
            if self.Kd:
                #print("Setting D gain to : {}".format(Kd))
                self.d = control.derivative(Kd,**kwargs)

        def set_gain(self,Kp = None, Ki = None,Kd = None, **kwargs):
            """Applies new gains to the PID controller

            Args:
                Kp (float, optional): Proportional gain.
                Ki (float, optional): Integral gain.
                Kd (float, optional): Derivative gain.
            """
            if self.Kp and Kp:
                self.p.K = Kp
            elif Kp:
                self.p = control.proportional(Kp)

            if self.Ki and Ki:
                self.i.K = Ki
            elif Ki:
                self.i = control.integral(Ki,**kwargs)
                
            if self.Kd and Kd:
                self.d.K = Kd
            elif Kd:
                self.d = control.derivative(Kd,**kwargs)
        
        def start(self):
            """
            Starts the controller (only applicable to controllers using
            `integral` or `derivative`)
            """
            if self.Ki:
                self.i.start()
            if self.Kd:
                self.d.start()

        def update(self,error):
            """
            Updates the controllers with a new error 
            and calculates the appropriate correction.

            Args:
                error (float) : error to correct

            Returns (float) : correction

            """
            P,I,D = 1,0,0
            if self.Kp:
                P = self.p.update(error)
            if self.Ki:
                I = self.i.update(error)
            if self.Kd:
                D = self.d.update(error)

            if self.form == "parallel":
                return P + I + D
            if self.form == "ideal":
                return P * ( 1 + I + D )

    class lead_lag_comp():
        """
        Discrete time lead-lag compensator of the form k*(s+a)/(s+b)

        Args:
            a (float) : Location of zero
            b (float) : Location of pole
            k (float) : Amount of gain
        """
        def __init__(self,a = 0,b = 0,k = 1):
            self.a = a
            self.b = b
            self.k = k

        def start(self):
            """
            Starts the controller
            """
            self.prev_time = time.time()
            self.prev_error = 0
            self.prev_output = 0

        def update(self,error):
            """
            Updates the controllers with a new error 
            and calculates the appropriate correction.

            Args:
                error (float) : error to correct

            Returns (float) : correction
            """
            xtime = time.time()
            T = xtime-self.prev_time
            output = (self.k*(error*(1+self.a*T)-self.prev_error)+self.prev_output)/(1+self.b*T)
            self.prev_time = xtime
            self.prev_output = output
            self.prev_error = error
            return output

    def limiter(value,min,max):
        """
        Limit filter to set some `value` to equal to `min` or `max` if out of bounds.

        Args:
            value (float) : Value to limit between `min` and `max`
            min (float) : Minimum allowed value
            max (float) : Maximum allowed value
        """
        if value < min:
            return min
        elif value > max:
            return max
        else:
            return value

if __name__ == "__main__":

    def wrapper(value,min,max):
        if value < min :
            return value + max-min
        elif value > max:
            print(value)
            return value - max+min
        else: return value

    from matplotlib import pyplot as plt
    from math import sin

    signal1 = [0 for i in range(10)]
    signal1 += [1 for i in range(50)]

    signal2 = [1 for i in range(500)]

    lp1 = control.cascade(control.low_pass_bi,3,tau=0.016,debug_time=1/100,rollover_min=-180,rollover_max=180,init_val = 0)
    lp2 = control.cascade(control.low_pass_bi,10,tau=0.005,debug_time=1/100,rollover_min=-180,rollover_max=180,init_val = 0)

    xtime = [x/100 for x in range(len(signal1))]
    out1 = [lp1.update(x) for x in (signal1)]
    out2 = [lp2.update(x) for x in (signal1)]

    signal1 = [wrapper(x,-180,180) for x in signal1]

    plt.plot(xtime,signal1,xtime,out1,xtime,out2)
    plt.grid()
    plt.show()