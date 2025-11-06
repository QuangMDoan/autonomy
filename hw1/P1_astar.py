import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from utils import plot_line_segments

class AStar(object):
    """Represents a motion planning problem to be solved using A*"""

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, distance_func, resolution=1):
        if not callable(distance_func):
            raise TypeError("Expected a function")
        
        self.distance_func = distance_func
        if distance_func == AStar.eucludean_distance:
            self.distance_func_name = "eucludean distance" 
        elif distance_func == AStar.manhattan_distance:
            self.distance_func_name = "manhattan distance"
        elif distance_func == AStar.chebyshev_distance:
            self.distance_func_name = "chebyshev distance"

        self.statespace_lo = statespace_lo         # state space lower bound (e.g., [-5, -5])
        self.statespace_hi = statespace_hi         # state space upper bound (e.g., [5, 5])
        self.occupancy = occupancy                 # occupancy grid (a DetOccupancyGrid2D object)
        self.resolution = resolution               # resolution of the discretization of state space (cell/m)
        self.x_offset = x_init                     
        self.x_init = self.snap_to_grid(x_init)    # initial state
        self.x_goal = self.snap_to_grid(x_goal)    # goal state

        self.closed_set = set()    # the set containing the states that have been visited
        self.open_set = set()      # the set containing the states that are candidate for future expension

        self.est_cost_through = {}  # dictionary of the estimated cost from start to goal passing through state (often called f score)
        self.cost_to_arrive = {}    # dictionary of the cost-to-arrive at state from start (often called g score)
        self.came_from = {}         # dictionary keeping track of each state's parent to reconstruct the path

        self.open_set.add(self.x_init)
        self.cost_to_arrive[self.x_init] = 0
        self.est_cost_through[self.x_init] = self.distance(self.x_init,self.x_goal)

        self.path = None        # the final path as a list of states

    def is_free(self, x):
        if x[0] < self.statespace_lo[0] or x[1] < self.statespace_lo[1]:
            return False

        if x[0] > self.statespace_hi[0] or x[1] > self.statespace_hi[1]:
            return False

        return self.occupancy.is_free(x)
    
    def distance(self, *args, **kwargs):
        return self.distance_func(self, *args, **kwargs)  # note: pass self explicitly

    def eucludean_distance(self, x1, x2):
        return np.linalg.norm(np.array(x1) - np.array(x2))

    def manhattan_distance(self, x1, x2):
        return np.sum(np.abs(np.array(x1) - np.array(x2)))

    def chebyshev_distance(self, x1, x2):
        return np.max(np.abs(np.array(x1) - np.array(x2)))
    
    def snap_to_grid(self, x):
        return (
            self.resolution * round((x[0] - self.x_offset[0]) / self.resolution) + self.x_offset[0],
            self.resolution * round((x[1] - self.x_offset[1]) / self.resolution) + self.x_offset[1],
        )

    def get_neighbors(self, x):
        neighbor_candidates = []
        dx, dy = self.resolution, self.resolution
        neighbor_candidates.append((x[0] - dx, x[1])) # left 
        neighbor_candidates.append((x[0] + dx, x[1])) # right

        neighbor_candidates.append((x[0], x[1] - dy)) # up
        neighbor_candidates.append((x[0], x[1] + dy)) # down

        neighbor_candidates.append((x[0] - dx, x[1] - dy)) # top-left 
        neighbor_candidates.append((x[0] + dx, x[1] - dy)) # top-right 
        
        neighbor_candidates.append((x[0] - dx, x[1] + dy)) # bottom-left 
        neighbor_candidates.append((x[0] + dx, x[1] + dy)) # bottom-right 

        neighbor_candidates2 = [self.snap_to_grid(x) for x in neighbor_candidates]
        neighbors = [ x for x in neighbor_candidates2 if self.is_free(x) ]

        return neighbors

    def find_best_est_cost_through(self):
        return min(self.open_set, key=lambda x: self.est_cost_through[x])

    def reconstruct_path(self):
        path = [self.x_goal]
        current = path[-1]
        while current != self.x_init:
            path.append(self.came_from[current])
            current = path[-1]
        return list(reversed(path))

    def plot_path(self, fig_num=0, show_init_label=True):
        """Plots the path found in self.path and the obstacles"""
        if not self.path:
            return

        self.occupancy.plot(fig_num)

        solution_path = np.asarray(self.path)
        plot_label = f"A* solution path: {self.distance_func_name}"
        plt.plot(solution_path[:,0],solution_path[:,1], color="green", linewidth=2, label=plot_label, zorder=10)
        plt.scatter([self.x_init[0], self.x_goal[0]], [self.x_init[1], self.x_goal[1]], color="green", s=30, zorder=10)
        if show_init_label:
            plt.annotate(r"$x_{init}$", np.array(self.x_init) + np.array([.2, .2]), fontsize=16)
        plt.annotate(r"$x_{goal}$", np.array(self.x_goal) + np.array([.2, .2]), fontsize=16)
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.03), fancybox=True, ncol=3)

        plt.axis([0, self.occupancy.width, 0, self.occupancy.height])

    def plot_tree(self, point_size=15):
        plot_line_segments([(x, self.came_from[x]) for x in self.open_set if x != self.x_init], linewidth=1, color="blue", alpha=0.2)
        plot_line_segments([(x, self.came_from[x]) for x in self.closed_set if x != self.x_init], linewidth=1, color="blue", alpha=0.2)
        px = [x[0] for x in self.open_set | self.closed_set if x != self.x_init and x != self.x_goal]
        py = [x[1] for x in self.open_set | self.closed_set if x != self.x_init and x != self.x_goal]
        plt.scatter(px, py, color="blue", s=point_size, zorder=10, alpha=0.2)

    def solve(self):
        while bool(self.open_set):
            x_current = self.find_best_est_cost_through()
            if x_current == self.x_goal:
                self.path = self.reconstruct_path()
                return True
            
            self.open_set.remove(x_current)
            self.closed_set.add(x_current)

            for xn in self.get_neighbors(x_current):
                if xn in self.closed_set:
                    continue
                
                est_cost_to_arrive = self.cost_to_arrive[x_current] + \
                    self.distance(x_current, xn)
                
                if xn not in self.open_set:
                    self.open_set.add(xn)
                elif est_cost_to_arrive > self.cost_to_arrive[xn]:
                    continue
                
                self.came_from[xn] = x_current
                self.cost_to_arrive[xn] = est_cost_to_arrive
                self.est_cost_through[xn] = est_cost_to_arrive + \
                    self.distance(xn, self.x_goal)
        
        return False

class DetOccupancyGrid2D(object):
    """
    A 2D state space grid with a set of rectangular obstacles. The grid is
    fully deterministic
    """
    def __init__(self, width, height, obstacles):
        self.width = width
        self.height = height
        self.obstacles = obstacles

    def is_free(self, x):
        """Verifies that point is not inside any obstacles by some margin"""
        for obs in self.obstacles:
            if x[0] >= obs[0][0] - self.width * .01 and \
               x[0] <= obs[1][0] + self.width * .01 and \
               x[1] >= obs[0][1] - self.height * .01 and \
               x[1] <= obs[1][1] + self.height * .01:
                return False
        return True

    def plot(self, fig_num=0):
        """Plots the space and its obstacles"""
        fig = plt.figure(fig_num)
        ax = fig.add_subplot(111, aspect='equal')
        for obs in self.obstacles:
            ax.add_patch(
            patches.Rectangle(
            obs[0],
            obs[1][0]-obs[0][0],
            obs[1][1]-obs[0][1],))
        ax.set(xlim=(0,self.width), ylim=(0,self.height))
