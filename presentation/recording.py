import os
import subprocess
import signal


class DataRecorder(object):
    def __init__(self, topics=None):
        """
        Record data from a set of rostopics using a subprocess interface to the rosbag utility
        :param topics: A list of topics to subscribe to
        :return: None
        """
        self.record_params = ["rosbag", "record"]
        self.process = None
        self.recording = False
        self._topics = {}
        for topic in topics:
            self.add_topic(topic)

    def add_topic(self, topic):
        """
        Add a topic to subscribe to
        :param topic: The topic name
        :return: None
        """
        self._topics[topic] = True

    def topics(self):
        """
        Get the topics subscribed to
        :return: A list of topics
        """
        return self._topics.keys()

    def remove_topic(self, topic):
        """
        Unsubscribe from topics
        :param topic: The topic to unsubscribe from
        :return: None
        """
        if topic in self._topics:
            del self._topics[topic]

    def start(self):
        """
        Start the recording process
        :return:
        """
        self.process = subprocess.Popen(self.record_params + self.topics(), preexec_fn=os.setsid)
        self.recording = True

    def stop(self):
        """
        Kill the recording process
        :return:
        """
        os.killpg(self.process.pid, signal.SIGINT)

