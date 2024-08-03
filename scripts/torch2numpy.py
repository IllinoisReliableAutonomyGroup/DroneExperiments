import torch
import numpy as np
import pickle5 as pickle
import argparse

parser = argparse.ArgumentParser(description="")
parser.add_argument('--torch', type=str, help='filename to the pytorch model file.')
parser.add_argument('--numpy', type=str, help='filename to save the numpy pkl.')
args = parser.parse_args()

ck = torch.load(args.torch, map_location='cpu')
w1 = ck['model_u_w1']
w2 = ck['model_u_w2']

npmodel = {}

npmodel['w1_0w'] = w1['0.weight'].numpy()
npmodel['w1_0b'] = w1['0.bias'].numpy()
npmodel['w1_1w'] = w1['2.weight'].numpy()
npmodel['w1_1b'] = w1['2.bias'].numpy()

npmodel['w2_0w'] = w2['0.weight'].numpy()
npmodel['w2_0b'] = w2['0.bias'].numpy()
npmodel['w2_1w'] = w2['2.weight'].numpy()
npmodel['w2_1b'] = w2['2.bias'].numpy()

with open(args.numpy, 'wb') as handle:
    pickle.dump(npmodel, handle, protocol=pickle.HIGHEST_PROTOCOL)
