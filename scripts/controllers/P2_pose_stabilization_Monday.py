import numpy as np
from utils import wrapToPi
import rospy
from std_msgs.msg import Float64

# rospy.init_node('P2_node', anonymous=True)
alpha_pub = rospy.Publisher('/controller/alpha', Float64, queue_size=10)
delta_pub = rospy.Publisher('/controller/delta', Float64, queue_size=10)
rho_pub = rospy.Publisher('/controller/rho', Float64, queue_size=10)

# command zero velocities once we are this close to the goal
RHO_THRES = 0.05
ALPHA_THRES = 0.1
DELTA_THRES = 0.1

class PoseController:
    """ Pose stabilization controller """
    def __init__(self, k1, k2, k3, V_max=0.5, om_max=1):
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3

        self.V_max = V_max
        self.om_max = om_max

    def load_goal(self, x_g, y_g, th_g):
        """ Loads in a new goal position """
        self.x_g = x_g
        self.y_g = y_g
        self.th_g = th_g

    def compute_control(self, x, y, th, t):
        """
        Inputs:
            x,y,th: Current state
            t: Current time (you shouldn't need to use this)
        Outputs: 
            V, om: Control actions

        Hints: You'll need to use the wrapToPi function. The np.sinc function
        may also be useful, look up its documentation
        """
        ########## Code starts here ##########
        # When goal is not at the origin, we need to compute new state relative to the new origin
        th_rel = th - self.th_g
        # Before rotating axes
        x_rel_init = x - self.x_g
        y_rel_init = y - self.y_g
        # Rotate axes
        x_rel = x_rel_init * np.cos(self.th_g) + y_rel_init * np.sin(self.th_g)
        y_rel = -x_rel_init * np.sin(self.th_g) + y_rel_init * np.cos(self.th_g)
        
        rho = np.sqrt(x_rel**2 + y_rel**2)
        alpha = wrapToPi(np.arctan2(y_rel, x_rel) - th_rel + np.pi)
        delta = wrapToPi(alpha + th_rel)
        # Publish rho, alpha and delta
    	rate = rospy.Rate(1)
 	while not rospy.is_shutdown():
            alpha_pub.publish(alpha)
	    delta_pub.publish(delta)
	    rho_pub.publish(rho)
            rate.sleep()
		
 
        if abs(rho) < RHO_THRES and abs(alpha) < ALPHA_THRES and abs(delta) < DELTA_THRES:
            V = 0
            om = 0
        else:
            V = self.k1 * rho * np.cos(alpha)
            om = self.k2 * alpha + self.k1 * np.cos(alpha) * np.sinc(alpha / np.pi) * (alpha + self.k3 * delta)
        ########## Code ends here ##########

        # apply control limits
        V = np.clip(V, -self.V_max, self.V_max)
        om = np.clip(om, -self.om_max, self.om_max)

        return V, om
