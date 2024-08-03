import torch

num_dim_x = 3
num_dim_control = 1

Tc = None

def setTc(_Tc):
    global Tc
    Tc = _Tc

def f_func(x):
    # x: bs x n x 1
    # f: bs x n x 1
    bs = x.shape[0]

    px, vx, ax = [x[:,i,0] for i in range(num_dim_x)]
    f = torch.zeros(bs, num_dim_x, 1).type(x.type())
    f[:, 0, 0] = vx
    f[:, 1, 0] = ax
    f[:, 2, 0] = -ax / Tc
    return f

def DfDx_func(x):
    raise NotImplemented('NotImplemented')

def B_func(x):
    bs = x.shape[0]
    B = torch.zeros(bs, num_dim_x, num_dim_control).type(x.type())

    B[:, 2, 0] = 1 / Tc
    return B

def DBDx_func(x):
    raise NotImplemented('NotImplemented')
