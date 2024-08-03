import rospy
import pickle
from std_msgs.msg import Float64MultiArray

class ViconSubscriber:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('vicon_subscriber', anonymous=True)

        # Subscribe to the /vicon_estimate topic
        self.subscriber = rospy.Subscriber('/vicon_estimate', Float64MultiArray, self.callback)

        # Initialize an empty list to store data
        self.data = []

        # Specify the filename for the pickle file
        self.pickle_file = 'vicon_logger_P0_lead.pkl'

    def callback(self, msg):
        # Append the received data to the list
        self.data.append(msg.data)

        # Save the data to a pickle file
        with open(self.pickle_file, 'wb') as f:
            pickle.dump(self.data, f)

        rospy.loginfo(f"Data saved: {msg.data}")

    def spin(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        vicon_subscriber = ViconSubscriber()
        vicon_subscriber.spin()
    except rospy.ROSInterruptException:
        pass
