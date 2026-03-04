import numpy as np
import matplotlib.pyplot as plt

def generate_point_cloud(mu, sigma, N=25):
    points = np.random.multivariate_normal(mean=mu, cov=sigma, size=N).T
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
    A = generate_point_cloud(mu=[1, 1], sigma=[[0.5, 0], [0, 0.5]], N=25)
    R, t, B = body_transforn(A)

    np.savetxt('data/A.txt', A)
    np.savetxt('data/B.txt', B)
    np.savetxt("data/R.txt", R)
    np.savetxt("data/t.txt", t)

def plot_point_clouds(pcd_list, azim=-60, elev=30):
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    color_fns = [plt.cm.viridis, plt.cm.inferno]
    for i, pcd in enumerate(pcd_list):
      points = np.asarray(pcd.points)
      colors = color_fns[i](points[:, 2] / points[:, 2].max())
      ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=10, c=colors)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.view_init(azim=azim, elev=elev)

    plt.show()

