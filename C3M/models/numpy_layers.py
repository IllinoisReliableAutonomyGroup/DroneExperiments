import numpy as np

class Linear():
    def __init__(self, w, b):
        self.w = w
        self.b = b

    def __call__(self, x):
        y = self.w.dot(x) + self.b
        return y

class Tanh():
    def __init__(self):
        pass
    def __call__(self, x):
        y = np.tanh(x)
        return y

class Sequential():
    def __init__(self, *args):
        self.layers = args

    def __call__(self, x):
        for l in self.layers:
            x = l(x)
        return x
