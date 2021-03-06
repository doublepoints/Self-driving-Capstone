from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import datetime
import rospy


class TLClassifier(object):
    def __init__(self, is_carla):

        self.image_size=(600,800)
        self.cur_ls = TrafficLight.UNKNOWN

        if is_carla:
            PATH_TO_GRAPH = r'light_classification/models/ssd_inc_v2_real_finetune/graph_optimized.pb'
        else:
            PATH_TO_GRAPH = r'light_classification/models/ssd_inc_v2_sim_finetune/graph_optimized.pb'

        self.threshold = 0.5
        self.warmup=True
        self.load_model(PATH_TO_GRAPH)
        self.warmup_model()
        

    def load_model(self,PATH_TO_GRAPH):
        self.detection_graph = tf.Graph() 
        rospy.loginfo('Loading model: %s', PATH_TO_GRAPH)
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_GRAPH, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self.sess = tf.Session(graph=self.detection_graph)

        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    def warmup_model(self):
        image = np.zeros((self.image_size[0], self.image_size[1], 3), dtype=np.uint8)
        self.get_classification(image)
        self.warmup=False
        
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
           traffic light color (specified in styx_msgs/TrafficLight)

        """

        # with self.detection_graph.as_default():
        image_expanded = np.expand_dims(image, axis=0)
        start = datetime.datetime.now()
        (boxes, scores, classes, num) = self.sess.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_expanded})

        end = datetime.datetime.now()
        c = end - start
        if self.warmup==True:
            rospy.loginfo('Tensorflow warmup completed (Time elapsed: %.3f ms)', c.total_seconds())
        else:
            print('Time: ', c.total_seconds())
            print('SCORES: ', scores[0, 0])
            print('CLASSES: ', classes[0, 0])

        if scores[0, 0] > self.threshold:
            if (classes[0, 0] == 1):
                self.cur_ls = TrafficLight.GREEN
                rospy.loginfo("Current light: GREEN")
            elif (classes[0, 0] == 2):
                self.cur_ls = TrafficLight.YELLOW
                rospy.loginfo("Current light: YELLOW")
            elif (classes[0, 0] == 3):
                self.cur_ls = TrafficLight.RED
                rospy.loginfo("Current light: Red")
            else:
                self.cur_ls = TrafficLight.UNKNOWN

        return self.cur_ls

