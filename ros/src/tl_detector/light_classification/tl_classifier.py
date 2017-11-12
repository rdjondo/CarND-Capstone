from __future__ import division, print_function # TODO: maybe hide in ROS

import tensorflow as tf
import numpy as np

# from styx_msgs.msg import TrafficLight # TODO:  enable for ROS

PATH_SSD_PROTOBUF = 'frozen_model.pb' #TODO: where to put a pre-trained TF SSD in ROS?
THRESH_SCORE      = 0.5
TL_CLASS          = 10  # COCO

class TLClassifier(object):
    def __init__(self):
        """
        Loads a pretrained SSD detector and intializes a TF session.
        """

        detection_graph = tf.Graph()

        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile (PATH_SSD_PROTOBUF, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self._image_tensor      = detection_graph.get_tensor_by_name('image_tensor:0')       # Input tensor expecting shape (n_images, n_rows, n_cols, 3)
        self._detection_boxes   = detection_graph.get_tensor_by_name('detection_boxes:0')    # Each box represents a part of the image where a particular object was detected.
        self._detection_scores  = detection_graph.get_tensor_by_name('detection_scores:0')   # Each score represent how level of confidence for each of the objects.
        self._detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')  # Classes

        self._TF_session = tf.Session(graph=detection_graph)



    def get_boxes (self, image, min_score=THRESH_SCORE):
        """
        Return boxes bounding traffic lights in the image, if the confidence score is bigger than min_score.

        Run TF, retrieve boxes, bring them to images space, filter by score

        Returns:
            boxes    (nBoxes, 4), storing y0,x0,y1,x1 coordinates (image coords)
            scores   (nBoxes, 1)
        """

        tensor = image [np.newaxis]
        (boxes, scores, classes) = self._TF_session.run(
            fetches   = [self._detection_boxes, self._detection_scores, self._detection_classes],
            feed_dict = {self._image_tensor: tensor}
        )

        # sanity checks of boxes [y0,x0,y1,x1]. Shape is 1,100,4
        np.testing.assert_array_less(boxes[..., 0], boxes[..., 2])  # y0 < y1 must hold for every BB
        np.testing.assert_array_less(boxes[..., 1], boxes[..., 3])  # x0 < x1 must hold for every BB
        np.testing.assert_array_less(classes, 90)                   # 90 classes in COCO

        # bring relative coordinates to image coordinates
        boxes *= [image.shape[0], image.shape[1], image.shape[0], image.shape[1]]

        # remove the leading dimension of 1 (TF required a 4D tensor)
        boxes   =   boxes.squeeze().astype(int) #TODO floor for bb0, ceil for bb1, check if not exceeding image
        classes = classes.squeeze().astype(int)
        scores  =  scores.squeeze()

        i_filtered = np.argwhere((classes == TL_CLASS) & (scores > min_score)).flatten()

        return boxes[i_filtered], scores[i_filtered]


    def classify_bbox (self, image, bbox):
        """classify bbox of the image
        TrafficLight.UNKNOWN, TrafficLight.RED, TrafficLight.YELLOW, TrafficLight.GREEN
        """

        result = 0 # TODO

        y0,x0,y1,x1 = bbox
        crop = image [y0:y1, x0:x1]
        print (crop.shape)

        return -1 #TODO





    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        result = 0 # TODO TrafficLight.UNKNOWN

        try:
            assert image.ndim     == 3, 'Expecting images of shape (nRows, nCols, nChannels)'
            assert image.shape[2] == 3, 'Expecting the channel axis at the end'

            # SSD to yield TL boxes and scores
            tl_boxes, tl_scores = self.get_boxes(image)

            # classify only the box with highest confidence
            result = self.classify_bbox (image, tl_boxes[0])

        except Exception as e:
            print (e) # TODO logging
            pass

        return result



if __name__ == '__main__':
    import cv2

    TLC = TLClassifier()

    for fn in 'tl_0_g.png frame_009275.jpg simss0.jpg tl.jpg tl2.jpg tl3.jpg'.split() [:1]:
        image = cv2.imread(fn) [...,:3]# [::-1] # BGR to RGB
        print (fn,TLC.get_classification(image))
        print ()
