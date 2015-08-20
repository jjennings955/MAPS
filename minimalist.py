# Minimalist ROS Node usage

import rospy
from std_msgs.msg import String


def take_action(pressure_data):
    """
    Declare your own function here!
    :param pressure_data: A dictionary containing sensor_id and pressure data
    :return: None
    """
    print(pressure_data)

rospy.init_node("MAPS_EXAMPLE")
commands = rospy.Publisher('/MAPScommand', String, queue_size=1)
commands.publish("autocalibrate 0.01")

subscriber = rospy.Subscriber("/pressure_events", String, take_action)
rospy.spin()