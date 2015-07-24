import os
import subprocess
import signal

""" Record data from a set of rostopics using a subprocess interface to the rosbag utility"""
class DataRecorder(object):
    def __init__(self, topics=None):
        self.record_params = ["rosbag", "record"]
        self.process = None
        self.recording = False
        self._topics = {}
        for topic in topics:
            self.add_topic(topic)

    def add_topic(self, topic):
        self._topics[topic] = True
    def topics(self):
        return self._topics.keys()
    def remove_topic(self, topic):
        if topic in self._topics:
            del self._topics[topic]

    def start(self):
        self.process = subprocess.Popen(self.record_params + self.topics(), preexec_fn=os.setsid)
        self.recording = True
    def stop(self):
        os.killpg(self.process.pid, signal.SIGINT)

