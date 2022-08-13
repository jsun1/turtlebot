import rospy

def publisher():
	pass

if __name__ == '__main__':
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    pub.publish(twist)
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
