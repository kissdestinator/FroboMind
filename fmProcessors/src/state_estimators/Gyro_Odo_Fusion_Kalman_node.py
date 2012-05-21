#!/usr/bin/env python
import roslib; roslib.load_manifest('fmProcessors')
import rospy
from fmMsgs.msg import gyroscope
from fmMsgs.msg import Vector3
from fmMsgs.msg import kalman_output
from math import *

odoAngVel = 0.
gyroAngVel = 0.
initial_xy = [0., 0.,0.]
lastInterruptTime = 0
dt = 0.02
gyroOffset = -0.01492




class matrix:
    
    # implements basic operations of a matrix class

    # ------------
    #
    # initialization - can be called with an initial matrix
    #

    def __init__(self, value = [[]]):
        self.value = value
        self.dimx  = len(value)
        self.dimy  = len(value[0])
        if value == [[]]:
            self.dimx = 0
            
    # -----------
    #
    # defines matrix equality - returns true if corresponding elements
    #   in two matrices are within epsilon of each other.
    #
    
    def __eq__(self, other):
        epsilon = 0.01
        if self.dimx != other.dimx or self.dimy != other.dimy:
            return False
        for i in range(self.dimx):
            for j in range(self.dimy):
                if abs(self.value[i][j] - other.value[i][j]) > epsilon:
                    return False
        return True
    
    def __ne__(self, other):
        return not (self == other)

    # ------------
    #
    # makes matrix of a certain size and sets each element to zero
    #

    def zero(self, dimx, dimy):
        if dimy == 0:
            dimy = dimx
        # check if valid dimensions
        if dimx < 1 or dimy < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx  = dimx
            self.dimy  = dimy
            self.value = [[0.0 for row in range(dimy)] for col in range(dimx)]

    # ------------
    #
    # makes matrix of a certain (square) size and turns matrix into identity matrix
    #

    def identity(self, dim):
        # check if valid dimension
        if dim < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx  = dim
            self.dimy  = dim
            self.value = [[0.0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1.0
                
    # ------------
    #
    # prints out values of matrix
    #

    def show(self, txt = ''):
        for i in range(len(self.value)):
            print txt + '['+ ', '.join('%.3f'%x for x in self.value[i]) + ']' 
        print ' '

    # ------------
    #
    # defines elmement-wise matrix addition. Both matrices must be of equal dimensions
    #

    def __add__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimx != other.dimx:
            raise ValueError, "Matrices must be of equal dimension to add"
        else:
            # add if correct dimensions
            res = matrix()
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res

    # ------------
    #
    # defines elmement-wise matrix subtraction. Both matrices must be of equal dimensions
    #

    def __sub__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimx != other.dimx:
            raise ValueError, "Matrices must be of equal dimension to subtract"
        else:
            # subtract if correct dimensions
            res = matrix()
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res

    # ------------
    #
    # defines multiplication. Both matrices must be of fitting dimensions
    #

    def __mul__(self, other):
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise ValueError, "Matrices must be m*n and n*p to multiply"
        else:
            # multiply if correct dimensions
            res = matrix()
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
        return res

    # ------------
    #
    # returns a matrix transpose
    #

    def transpose(self):
        # compute transpose
        res = matrix()
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res

    def take(self, list1, list2 = []):
        if list2 == []:
            list2 = list1
        if len(list1) > self.dimx or len(list2) > self.dimy:
            raise ValueError, "list invalid in take()"

        res = matrix()
        res.zero(len(list1), len(list2))
        for i in range(len(list1)):
            for j in range(len(list2)):
                res.value[i][j] = self.value[list1[i]][list2[j]]
        return res


    def expand(self, dimx, dimy, list1, list2 = []):
        if list2 == []:
            list2 = list1
        if len(list1) > self.dimx or len(list2) > self.dimy:
            raise ValueError, "list invalid in expand()"

        res = matrix()
        res.zero(dimx, dimy)
        for i in range(len(list1)):
            for j in range(len(list2)):
                res.value[list1[i]][list2[j]] = self.value[i][j]
        return res

    # ------------
    #
    # Computes the upper triangular Cholesky factorization of  
    # a positive definite matrix.
    # This code is based on http://adorio-research.org/wordpress/?p=4560

    def Cholesky(self, ztol= 1.0e-5):

        res = matrix()
        res.zero(self.dimx, self.dimx)

        for i in range(self.dimx):
            S = sum([(res.value[k][i])**2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else: 
                if d < 0.0:
                    raise ValueError, "Matrix not positive-definite"
                res.value[i][i] = sqrt(d)
            for j in range(i+1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(i)])
                if abs(S) < ztol:
                    S = 0.0
                res.value[i][j] = (self.value[i][j] - S)/res.value[i][i]
        return res 
 
    # ------------
    #
    # Computes inverse of matrix given its Cholesky upper Triangular
    # decomposition of matrix.
    # This code is based on http://adorio-research.org/wordpress/?p=4560

    def CholeskyInverse(self):
    # Computes inverse of matrix given its Cholesky upper Triangular
    # decomposition of matrix.
        # This code is based on http://adorio-research.org/wordpress/?p=4560

        res = matrix()
        res.zero(self.dimx, self.dimx)

    # Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k]*res.value[j][k] for k in range(j+1, self.dimx)])
            res.value[j][j] = 1.0/ tjj**2 - S/ tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = \
                    -sum([self.value[i][k]*res.value[k][j] for k in \
                              range(i+1,self.dimx)])/self.value[i][i]
        return res
    
    # ------------
    #
    # comutes and returns the inverse of a square matrix
    #

    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res

    # ------------
    #
    # prints matrix (needs work!)
    #

    def __repr__(self):
        return repr(self.value)

###############################################

gyroBelieve = 0.5
odoBelieve = 0.5
speedCount = [0.,0.,0.,0.,0.]
speedCountCount = 0
    
P =  matrix([[0.1,0.,0.],[0.,0.1,0.],[0.,0.,0.1]])
F =  matrix([[1., dt * 0.5, dt * 0.5], [0.,1.,0.], [0.,0.,1.]])
H =  matrix([[0.,1.,0.],[0.,0.,1.]])
R =  matrix([[0.0001,0.],[0.,0.0001]])
I =  matrix([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]] )
Q = matrix([[0.01,0.,0.],[0.,0.1,0.],[0.,0.,0.1]])

x = matrix([[initial_xy[0]], [initial_xy[1]], [initial_xy[2]]]) # initial state (location and velocity)
u = matrix([[0.],[0.],[0.]]) # external motion

###############################################


def filter(x, P, measurements):
    global F, H, R, I , u
    for n in range(len(measurements)):
        
        # prediction
        x = (F * x) + u
        P = F * P * F.transpose() + Q
        
        # measurement update
        Z = matrix([measurements[n]])
        y = Z.transpose() - (H * x)
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse()
        x = x + (K * y)
        P = (I - (K * H)) * P 
    
    return x, P

def kalman_calc(event):
    global gyroAngVel, odoAngVel, x, P, dt, lastInterruptTime, gyroBelieve, odoBelieve
    gyroAngVel = drift_correction(gyroAngVel)
    temp_mes = [[gyroAngVel,odoAngVel]]
    update_Believes()
    dt = (event.current_real.to_sec()- lastInterruptTime)
    x, P = filter(x, P, temp_mes)
    
     #Publish Message
    pub_msg = kalman_output()
    pub_msg.header.stamp = rospy.get_rostime()
    pub_msg.yaw = x.value[0][0] % (pi * 2)
    pub_msg.ang_vel = x.value[1][0]
    pub_msg.ang_vel2 = x.value[2][0]	
    global pub
    pub.publish(pub_msg)
    lastInterruptTime = event.current_real.to_sec()

def update_Believes():
    global gyroAngVel,odoAngVel,gyroBelieve,odoBelieve
    if (gyroAngVel + odoAngVel < -0.02) or (gyroAngVel + odoAngVel > 0.02) :
	gyroBelieve = 0.5
	odoBelieve = 0.5
    else:
	gyroBelieve = 0.5
        odoBelieve = 0.5

def drift_correction(gyroAng):
	if (gyroAng < 0.004) and (gyroAng > -0.004):
		return 0
        return gyroAng

def gyro_callback(data):
    global gyroAngVel, gyroOffset
    gyroAngVel = data.z - gyroOffset

def odo_callback(data):
    global odoAngVel, speedCount, speedCountCount
    speedCount[speedCountCount] = data.th
    speedCountCount += 1
    if speedCountCount > len(speedCount) - 1:
	speedCountCount = 0
    odoAngVel = sum(speedCount)
    odoAngVel = odoAngVel/len(speedCount)

def Gyro_odo_fusion():
    rospy.init_node('Razor_Kalman')
    rospy.Subscriber("/fmSensors/Gyroscope", gyroscope , gyro_callback)
    rospy.Subscriber("/fmExtractors/xyz_position", Vector3 , odo_callback)
    rospy.Timer(rospy.Duration(0.02), kalman_calc, oneshot = False)
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('Kalman_AngularVelocity', kalman_output)

    Gyro_odo_fusion()
