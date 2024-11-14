import rospy
import numpy as np

class Topic(object):

    def __init__(self, message_type, allow_anonymous=False, queue_size=10):
        
        self.message_type = message_type
        self.queue_size = queue_size
        self.allow_anonymous = allow_anonymous
        self.subscribers_names = []
        self.publishers_names = []
        self.subscribers = {}  
        self.publishers = {}   

    def _dummy_subscriber_callback(self, new_message):
        print("update on topic %s, new message:\n" % self.name)
        print(new_message)

    def register_publisher(self, node_index, node_name=None):
        if node_name is None and self.allow_anonymous:
            return False
        if node_name not in self.publishers:
            publisher = rospy.Publisher(self.name+"_"+str(node_index), self.message_type, queue_size=self.queue_size)
            self.publishers_names.append(node_name)
            self.publishers[node_name] = publisher
            return publisher
        return False

    # def get_subscriber(self, topic_name= None, node_name=None , callback=None):
    #     if node_name is None and self.allow_anonymous:
    #         return False
    #     if node_name not in self.subscribers:
    #         subscriber = rospy.Subscriber(self.name if topic_name is None else topic_name, self.message_type, self._dummy_subscriber_callback if callback is None else callback)
    #         self.subscribers_names.append(node_name)
    #         self.subscribers[node_name] = subscriber
    #         return subscriber
    #     return False

    def publish(self, node_name, message):

        if not isinstance(message, self.message_type):
            rospy.logerr("Message type mismatch. Expected %s but received %s." % (self.message_type, type(message)))
            return False

        if node_name not in self.publishers_names:
            rospy.logerr("This node is not registered as a publisher for the topic %s." % self.name)
            return False
            
        publisher = self.publishers.get(node_name)

        if publisher is None:
            rospy.loginfo("No publisher found for this topic.")
            return None
        
        publisher.publish(message)
        return "##########################################\nPublished \n{}\non topic ==> {}\n##########################################\n".format(message, self.name+"_"+str(node_name[-1]))

    def subscibe(self, topic_name, node_name, callback = None):
        if node_name is None and self.allow_anonymous:
            return False
        if node_name not in self.subscribers:
            subscriber = rospy.Subscriber(self.name if topic_name is None else topic_name, self.message_type, self._dummy_subscriber_callback if callback is None else callback)
            self.subscribers_names.append(node_name)
            self.subscribers[node_name] = subscriber
            return subscriber
        return False
