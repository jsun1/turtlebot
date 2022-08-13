import numpy as np

EPSILON_OMEGA = 1e-3

def compute_dynamics(xvec, u, dt, compute_jacobians=True):
    """
    Compute Turtlebot dynamics (unicycle model).

    Inputs:
                     xvec: np.array[3,] - Turtlebot state (x, y, theta).
                        u: np.array[2,] - Turtlebot controls (V, omega).
        compute_jacobians: bool         - compute Jacobians Gx, Gu if true.
    Outputs:
         g: np.array[3,]  - New state after applying u for dt seconds.
        Gx: np.array[3,3] - Jacobian of g with respect to xvec.
        Gu: np.array[3,2] - Jacobian of g with respect to u.
    """
    ########## Code starts here ##########
    # TODO: Compute g, Gx, Gu
    # HINT: To compute the new state g, you will need to integrate the dynamics of x, y, theta
    # HINT: Since theta is changing with time, try integrating x, y wrt d(theta) instead of dt by introducing om
    # HINT: When abs(om) < EPSILON_OMEGA, assume that the theta stays approximately constant ONLY for calculating the next x, y
    #       New theta should not be equal to theta. Jacobian with respect to om is not 0.


    ########## Code ends here ##########

    x = xvec[0]
    y = xvec[1]
    theta = xvec[2]
    V = u[0]
    om = u[1]

    # DO NOT CHANGE! Full score verified by Jessie.
    if abs(om) < EPSILON_OMEGA:
        theta_new = theta + om * dt
        x_new = x + V * (np.cos(theta_new) + np.cos(theta)) * dt / 2 
        y_new = y + V * (np.sin(theta_new) + np.sin(theta)) * dt / 2
        g = np.array([x_new, y_new, theta_new])

        Gx = np.array([[1, 0, -V * dt / 2.0 * (np.sin(theta_new) + np.sin(theta))],
                       [0, 1, V * dt / 2.0 * (np.cos(theta_new) + np.cos(theta))],
                       [0, 0, 1]])
        Gu_00 = dt/2.0 * (np.cos(theta_new) + np.cos(theta))
        Gu_10 = dt/2.0 * (np.sin(theta_new) + np.sin(theta))
        Gu_01 = - V * dt * dt /2.0 * np.sin(theta_new)
        Gu_11 = V * dt * dt /2.0 * np.cos(theta_new)
        Gu = np.array([[Gu_00, Gu_01], [Gu_10, Gu_11], [0, dt]])
    else:
        theta_new = theta + om * dt
        x_new = x + V * (np.sin(theta_new) - np.sin(theta)) / om
        y_new = y + V * (np.cos(theta) - np.cos(theta_new)) / om
    
    	g = np.array([x_new, y_new, theta_new])

        Gx = np.array([[1, 0, V/om * (np.cos(theta_new) - np.cos(theta))],
                       [0, 1, V/om * (np.sin(theta_new) - np.sin(theta))],
                       [0, 0, 1]])
        Gu_00 = (np.sin(theta_new) - np.sin(theta)) /om
        Gu_10 = (np.cos(theta) - np.cos(theta_new)) /om
        Gu_01 = V/(om**2) * (np.sin(theta) - np.sin(theta_new) + om * dt * np.cos(theta_new))
        Gu_11 = V/(om**2) * (np.cos(theta_new) - np.cos(theta) + om * dt * np.sin(theta_new))
        Gu = np.array([[Gu_00, Gu_01], [Gu_10, Gu_11], [0, dt]]) 

    if not compute_jacobians:
        return g

    return g, Gx, Gu

def transform_line_to_scanner_frame(line, x, tf_base_to_camera, compute_jacobian=True):
    """
    Given a single map line in the world frame, outputs the line parameters
    in the scanner frame so it can be associated with the lines extracted
    from the scanner measurements.

    Input:
                     line: np.array[2,] - map line (alpha, r) in world frame.
                        x: np.array[3,] - pose of base (x, y, theta) in world frame.
        tf_base_to_camera: np.array[3,] - pose of camera (x, y, theta) in base frame.
         compute_jacobian: bool         - compute Jacobian Hx if true.
    Outputs:
         h: np.array[2,]  - line parameters in the scanner (camera) frame.
        Hx: np.array[2,3] - Jacobian of h with respect to x.
    """
    alpha, r = line

    ########## Code starts here ##########
    # TODO: Compute h, Hx
    # HINT: Calculate the pose of the camera in the world frame (x_cam, y_cam, th_cam), a rotation matrix may be useful
    # HINT: To compute line parameters in the camera frame h = (alpha_in_cam, r_in_cam), 
    #       draw a diagram with a line parameterized by (alpha,r) in the world frame and 
    #       a camera frame with origin at x_cam, y_cam rotated by th_cam wrt to the world frame
    # HINT: What is the projection of the camera location (x_cam, y_cam) on the line r? 
    # HINT: To find Hx, write h in terms of the pose of the base in world frame (x_base, y_base, th_base)


    ########## Code ends here ##########

    x_cam = x[0] + tf_base_to_camera[0] * np.cos(x[2]) - tf_base_to_camera[1] * np.sin(x[2])
    y_cam = x[1] + tf_base_to_camera[0] * np.sin(x[2]) + tf_base_to_camera[1] * np.cos(x[2])
    theta_cam = x[2] + tf_base_to_camera[2]

    alpha_cam = alpha - theta_cam

    r_cam = r - x_cam*np.cos(alpha) - y_cam*np.sin(alpha)
    h = np.array([alpha_cam, r_cam])
    Hx_12 = np.cos(alpha) * (tf_base_to_camera[0] * np.sin(x[2]) + tf_base_to_camera[1] * np.cos(x[2])) - np.sin(alpha) * (tf_base_to_camera[0] * np.cos(x[2]) - tf_base_to_camera[1] * np.sin(x[2]))
    Hx = np.array([[0, 0, -1],[-np.cos(alpha), -np.sin(alpha), Hx_12]])

    if not compute_jacobian:
        return h

    return h, Hx


def normalize_line_parameters(h, Hx=None):
    """
    Ensures that r is positive and alpha is in the range [-pi, pi].

    Inputs:
         h: np.array[2,]  - line parameters (alpha, r).
        Hx: np.array[2,n] - Jacobian of line parameters with respect to x.
    Outputs:
         h: np.array[2,]  - normalized parameters.
        Hx: np.array[2,n] - Jacobian of normalized line parameters. Edited in place.
    """
    alpha, r = h
    if r < 0:
        alpha += np.pi
        r *= -1
        if Hx is not None:
            Hx[1,:] *= -1
    alpha = (alpha + np.pi) % (2*np.pi) - np.pi
    h = np.array([alpha, r])

    if Hx is not None:
        return h, Hx
    return h
