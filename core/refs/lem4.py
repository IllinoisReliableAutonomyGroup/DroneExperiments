import numpy as np 

def xyz(args, real_t):
    period, sizex, sizey = args 

    period = 16

    t = real_t/period

    real_t = np.round(real_t, 1e-4)

    tmp = real_t%16

