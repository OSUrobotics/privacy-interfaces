#!/usr/bin/env python

import numpy

class KalmanFilter():
    """ 
    Kalman filter implementation. 

    Assumes that:
    1. State variables are INDEPENDENT of each other. 
    2. Values for R and Q are the same for all state variables
    3. The initial guess for S (variance) is the same for all state variables

    Inputs (default values chosen for a face detector in image space):
    S = initial guess of variance for each state variable
    R = variance of PREDICTION error for each state variable
    Q = variance of SENSOR error for each state variable
    dt_scale = change in time for which R, Q are defined
    """

    def __init__(self, S=1.0, R=10.0, Q=1.0, dt_scale=1/30.0):
        self.S = S
        self.R = R
        self.Q = Q
        self.dt_scale = dt_scale
        self.mu = None

    def run(self, t, z):
        if self.mu is None:  # first measurement
            self.N = len(z)
            self.I = numpy.eye(self.N)
            self.mu = z
            self.sig = self.S * numpy.eye(self.N)
        else:  # not the first measurement
            dt = t - self.t
            _mu, _sig = self.predict(dt)
            if self.z is not None:
                self.mu, self.sig = self.update(_mu, _sig, z)
            
        self.t = t  # update time -- all variables now up-to-date
            
    def predict(self, dt):
        _mu = self.mu
        _sig = self.sig + self.R * self.I * dt / self.dt_scale
        return _mu, _sig

    def update(self, _mu, _sig, z):
        K = _sig.dot(numpy.linalg.inv(_sig + self.Q * self.I))
        mu = _mu + K.dot(z - _mu)
        sig = (self.I - K).dot(_sig)
        return mu, sig

        
        
