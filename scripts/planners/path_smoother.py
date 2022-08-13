import numpy as np
import scipy.interpolate

def compute_smoothed_traj(path, V_des, alpha, dt):
    """
    Fit cubic spline to a path and generate a resulting trajectory for our
    wheeled robot.

    Inputs:
        path (np.array [N,2]): Initial path
        V_des (float): Desired nominal velocity, used as a heuristic to assign nominal
            times to points in the initial path
        alpha (float): Smoothing parameter (see documentation for
            scipy.interpolate.splrep)
        dt (float): Timestep used in final smooth trajectory
    Outputs:
        traj_smoothed (np.array [N,7]): Smoothed trajectory
        t_smoothed (np.array [N]): Associated trajectory times
    Hint: Use splrep and splev from scipy.interpolate
    """
    ########## Code starts here ##########
    path = np.array(path)
    x_init = path[:,0]
    y_init = path[:,1]
    t_delta = np.sqrt(np.diff(x_init)**2 + np.diff(y_init)**2)/V_des
    t = np.append([0], np.cumsum(t_delta))
    t_smoothed = np.arange(0, t[-1], dt)
    tck_x = scipy.interpolate.splrep(t, x_init, s=alpha)
    x = scipy.interpolate.splev(t_smoothed, tck_x)
    xd = scipy.interpolate.splev(t_smoothed, tck_x, der=1)
    xdd = scipy.interpolate.splev(t_smoothed, tck_x, der=2)
    tck_y = scipy.interpolate.splrep(t, y_init, s=alpha)
    y = scipy.interpolate.splev(t_smoothed, tck_y)
    yd = scipy.interpolate.splev(t_smoothed, tck_y, der=1)
    ydd = scipy.interpolate.splev(t_smoothed, tck_y, der=2)
    th = np.arctan2(yd, xd)
    traj_smoothed = np.vstack([x,y,th,xd,yd,xdd,ydd]).transpose()
    ########## Code ends here ##########

    return traj_smoothed, t_smoothed
