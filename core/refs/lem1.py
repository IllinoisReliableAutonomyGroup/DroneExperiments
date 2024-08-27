import numpy as np 

def find_index_range(array, value_range):
    start_idx = np.searchsorted(array, value_range[0], side='left')
    end_idx = np.searchsorted(array, value_range[1], side='right') - 1
    return start_idx, end_idx

def generate_template(array):
    template = np.zeros((len(array), 2)) 
    for i in range(12):
        start_idx, end_idx = find_index_range(array, (i, i+1))
        if i==0:
            template[start_idx:end_idx,0] = 2
            template[start_idx:end_idx,1] = np.linspace(2,1,end_idx-start_idx+1)[:-1]
        elif i==1:
            template[start_idx:end_idx,0] = np.linspace(2,1,end_idx-start_idx+1)[:-1]
            template[start_idx:end_idx,1] = 1
        elif i==2:
            template[start_idx:end_idx,0] = np.linspace(1,0,end_idx-start_idx+1)[:-1]
            template[start_idx:end_idx,1] = 1
        elif i==3:
            template[start_idx:end_idx,0] = 0
            template[start_idx:end_idx,1] = np.linspace(1,2,end_idx-start_idx+1)[:-1]
        elif i==4:
            template[start_idx:end_idx,0] = np.linspace(0,-1,end_idx-start_idx+1)[:-1]
            template[start_idx:end_idx,1] = 2
        elif i==5:
            template[start_idx:end_idx,0] = np.linspace(-1,-2,end_idx-start_idx+1)[:-1]
            template[start_idx:end_idx,1] = 2
        elif i==6:
            template[start_idx:end_idx,0] = -2
            template[start_idx:end_idx,1] = np.linspace(2,1,end_idx-start_idx+1)[:-1]
        elif i==7:
            template[start_idx:end_idx,0] = np.linspace(-2,-1,end_idx-start_idx+1)[:-1]
            template[start_idx:end_idx,1] = 1
        elif i==8:
            template[start_idx:end_idx,0] = np.linspace(-1,0,end_idx-start_idx+1)[:-1]
            template[start_idx:end_idx,1] = 1
        elif i==9:
            template[start_idx:end_idx,0] = 0
            template[start_idx:end_idx,1] = np.linspace(1,2,end_idx-start_idx+1)[:-1]
        elif i==10:
            template[start_idx:end_idx,0] = np.linspace(0,1,end_idx-start_idx+1)[:-1]
            template[start_idx:end_idx,1] = 2
        elif i==11:
            template[start_idx:end_idx+1,0] = np.linspace(1,2,end_idx-start_idx+1)
            template[start_idx:end_idx,1] = 2
    for i in range(template.shape[0]):
        if np.all(template[i,:] == np.array([0,0])):
            template[i,:] = (template[i-1,:]+template[i+1,:])/2

    return template


def xyz(args, real_t):
    # period, sizex, sizey = args 

    # period = 16

    # t = real_t/period
    t = real_t*0.6

    t = np.round(t, 6)

    y = np.zeros(t.shape)
    z = np.zeros(t.shape)

    start_idx, end_idx = find_index_range(t, (0,12))
    template = generate_template(t[start_idx:end_idx+1])
    y[start_idx:end_idx+1] = template[0:y[start_idx:end_idx+1].shape[0],0]
    z[start_idx:end_idx+1] = template[0:z[start_idx:end_idx+1].shape[0],1]


    start_idx, end_idx = find_index_range(t, (12,24))
    # template = generate_template(t[start_idx:end_idx])
    y[start_idx:end_idx+1] = template[0:y[start_idx:end_idx+1].shape[0],0]
    z[start_idx:end_idx+1] = template[0:z[start_idx:end_idx+1].shape[0],1]


    start_idx, end_idx = find_index_range(t, (24,36))
    # template = generate_template(t[start_idx:end_idx])
    y[start_idx:end_idx+1] = template[0:y[start_idx:end_idx+1].shape[0],0]
    z[start_idx:end_idx+1] = template[0:z[start_idx:end_idx+1].shape[0],1]

    y += 0.5
    x = np.ones_like(y) * 3.22
    return x, y, z 

if __name__ == "__main__":
    args = None
    real_t = np.arange(0,40,0.05) 

    x,y,z = xyz(args, real_t)

    import matplotlib.pyplot as plt 

    fig = plt.figure(0)
    ax = plt.axes(projection='3d')
    ax.plot3D(x,y,z)
    plt.figure(1)
    plt.plot(real_t, x)
    plt.plot(real_t, y)
    plt.plot(real_t, z)
    plt.show()