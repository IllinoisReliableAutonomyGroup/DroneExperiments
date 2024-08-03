import numpy as np

def xyz(args, real_t):
    period, sizex, sizey = args
    print('period/sizex/sizey: ', period, sizex, sizey)
    print(real_t)
    if not period:
        period = real_t[-1]
    
    starting_amplitude =0
    increments = 0.5
    period = 2*np.pi

    def generate_sine_curve(real_t, starting_amplitude, period):
        sine_curve = np.zeros_like(real_t)
        current_amplitude = starting_amplitude
        
        for i, t in enumerate(real_t):
            # Check if we are at the start of a new period
            if t >= period * (current_amplitude - starting_amplitude):
                current_amplitude += increments
            sine_curve[i] = current_amplitude * np.sin(t)
            
        return sine_curve


    # Generate the sine curve
    y = generate_sine_curve(real_t, starting_amplitude, period)

    x = np.ones_like(y)
    z = np.ones_like(y) * 1.5
    return x, y, z