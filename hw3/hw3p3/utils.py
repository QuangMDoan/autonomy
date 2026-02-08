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
    
    B = R @  A + np.array(t).reshape(-1, 1)
    return B

def generate_ab_clouds():
    A = generate_2d_cloud(N=25)
    B = body_transforn(A)
    np.save("data/a.npy", A)
    np.save("data/b.npy", B)