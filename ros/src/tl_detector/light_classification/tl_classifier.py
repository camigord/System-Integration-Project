from styx_msgs.msg import TrafficLight

import os
import sys
import tensorflow as tf

import numpy as np

from utils import label_map_util
from utils import visualization_utils as vis_util

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        curr_dir = os.path.dirname(os.path.realpath(__file__))

        path_to_ckpt = curr_dir + '/trafficlight_model_sim/frozen_inference_graph.pb'

        path_to_labels = curr_dir + '/label_map.pbtxt'
        num_classes = 4

        self.detection_graph = tf.Graph()

        #Load frozen Tensorflow model into memory
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(path_to_ckpt, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session(graph=self.detection_graph)

        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        self.boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        print("Model graph loaded")

        #Loading label map
        label_map = label_map_util.load_labelmap(path_to_labels)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=num_classes, use_display_name=True)

        self.category_index = label_map_util.create_category_index(categories)


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        image_expanded = np.expand_dims(image, axis=0)
        with self.detection_graph.as_default():
            (boxes, scores, classes, num) = self.sess.run(
            [self.boxes, self.scores,
            self.classes, self.num_detections],
            feed_dict={self.image_tensor: image_expanded})

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        #set unknown as default
        current_light = TrafficLight.UNKNOWN

        max_score_idx = scores.argmax()
        max_score = max(scores)

        if scores[max_score_idx] > 0.5:
            light_color = self.category_index[classes[max_score_idx]]['name']
            if light_color == 'Green':
                current_light = TrafficLight.GREEN
            elif light_color == 'Red':
                current_light = TrafficLight.RED
            elif light_color == 'Yellow':
                current_light = TrafficLight.YELLOW


        return current_light
        #return TrafficLight.UNKNOWN
