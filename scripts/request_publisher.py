#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from asl_turtlebot.msg import DeliveryRequest

class DeliveryRequestPublisher:
    """Publishes food delivery requests to the PavoneCart food delivery service. """

    def __init__(self):
        #initialize node
        rospy.init_node('delivery_request_publisher', anonymous=True)
        # initialize requested items
        self.delivery_request = None
        # whether to use shortest path
        self.use_shortest_path = False
        #create publisher
        self.request_publisher = rospy.Publisher('/delivery_request', DeliveryRequest, queue_size=10)

    def publish_request(self):
        #publish the request t times, once every s seconds
        t = 10
        s = 1.
        request = DeliveryRequest()
        request.vendors = self.delivery_request
        request.use_shortest_path = self.use_shortest_path
        for i in range(t):
            self.request_publisher.publish(request)
            rospy.sleep(s)
    
    def loop(self):
        """The main loop of the script. The script will ask for food items to add to the 
        delivery_request string until an empty answer ("") is given, at which point it will 
        publish the string. The current request will be published several few times after which the user 
        will be prompted to create a new request."""
        if self.delivery_request is None:
            
            #initialize string of requested items
            delivery_request = []
            request_not_complete = True
            
            #gather requests from user input
            tmp_request = []
            while request_not_complete:
                new_item = raw_input("Add an item to your delivery request: ")
                if new_item == "":
                    request_not_complete = False
                else:
                    tmp_request.append(new_item)

            pickup_order = raw_input("Please provide the pickup order (e.g. 2,1,3), press enter for shortest path order: ")
            if pickup_order == "":
                pickup_order = []
            else:
                pickup_order = pickup_order.split(",")
            
            if len(pickup_order) == 0:
                self.use_shortest_path = True
                delivery_request = tmp_request[:]
            elif len(pickup_order) != len(tmp_request):
                print "The number of orders is different from the requests"
                self.delivery_request = None
            else:
                delivery_request = tmp_request[:]
                for i, s in enumerate(pickup_order):
                    index = int(s) - 1
                    if index >= len(tmp_request):
                        print "Invalid order"
                        self.delivery_request = None
                        break
                    delivery_request[index] = tmp_request[i]
                         

            #eliminate trailing comma and publish
            # self.delivery_request = self.delivery_request[:-1]
            self.delivery_request = ",".join(delivery_request)
            print "Order sending..."
            self.publish_request()

            #reset delivery request to be ready for new inputs
            self.delivery_request = None
            print "\n", "Place another request?"

    def run(self):
        print "Create a delivery request:"
        print "You'll be prompted to enter food items one at a time. Once your order list is " \
            "complete, press enter to send your order."
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    may_i_take_your_order = DeliveryRequestPublisher()
    may_i_take_your_order.run()
