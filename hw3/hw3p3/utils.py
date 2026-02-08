import numpy as np
import matplotlib.pyplot as plt

def generate_2d_cloud(mu=0, sigma=1, N=100):
    points = np.random.normal(loc=mu, scale=sigma, size=(2, N))
    return points

def body_transforn(A, theta=5, t=[0.1, 0.3]):
    theta = np.deg2rad(theta)

    R = [
            [np.cos(theta), -np.sin(theta) ],
            [np.sin(theta),  np.cos(theta) ]
        ]
    
    R = np.array(R)
    t = np.array(t)
        
    B = R @  A + t[:, np.newaxis]
    return R, t, B

def generate_ab_clouds(N=25):
    A = generate_2d_cloud(N=N)
    R, t, B = body_transforn(A)
    np.save("data/a.npy", A)
    np.save("data/b.npy", B)
    np.save("data/r.npy", R)
    np.save("data/t.npy", t)