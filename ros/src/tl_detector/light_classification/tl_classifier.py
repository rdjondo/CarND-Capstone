from __future__ import division, print_function # TODO: maybe hide in ROS

import tensorflow as tf
import numpy as np

import cv2

import rospy
import os.path

# for out-of-ROS testing
try:
    from styx_msgs.msg import TrafficLight
except ImportError:
    import TrafficLight


FILE_MODEL        = 'ssd_mobilenet_v1_coco_11_06_2017_frozen_inference_graph.pb' # f0158b9e01424dc5bd20f000bcdad847

THRESH_SCORE      = 0.2
TL_CLASS          = 10  # COCO

LIGHT_NAMES = 'RED YELLOW GREEN undefined UNKNOWN'.split()

class TLClassifier(object):
    def __init__(self):
        """
        Loads a pretrained SSD detector and intializes a TF session.
        """

        try:
            path_to_model = '{}/{}'.format(os.path.dirname (os.path.realpath(__file__)), FILE_MODEL)

            rospy.loginfo ('\n\n\nInitializing tensorflow model from ' + path_to_model)

            detection_graph = tf.Graph()

            with detection_graph.as_default():
                od_graph_def = tf.GraphDef()
                with tf.gfile.GFile (path_to_model, 'rb') as fid:
                    serialized_graph = fid.read()
                    od_graph_def.ParseFromString(serialized_graph)
                    tf.import_graph_def(od_graph_def, name='')

            self._image_tensor      = detection_graph.get_tensor_by_name('image_tensor:0')       # Input tensor expecting shape (n_images, n_rows, n_cols, 3)
            self._detection_boxes   = detection_graph.get_tensor_by_name('detection_boxes:0')    # Each box represents a part of the image where a particular object was detected.
            self._detection_scores  = detection_graph.get_tensor_by_name('detection_scores:0')   # Each score represent how level of confidence for each of the objects.
            self._detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')  # Classes

            self._TF_session = tf.Session(graph=detection_graph)

        except Exception as e:
            rospy.logfatal ('\n\n\n')
            rospy.logfatal ('Failed to load / initialize Tensorflow model. Did you already downloaded and renamed it?')
            rospy.logfatal ('Reason follows:')
            rospy.logfatal (e)



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
        rospy.loginfo ('TLC: Box scores {}'.format (scores[i_filtered]))

        return boxes[i_filtered], scores[i_filtered]


    def classify_bbox (self, image, bbox):
        """Classify a bounding bbox in the image.


        TrafficLight.UNKNOWN, TrafficLight.RED, TrafficLight.YELLOW, TrafficLight.GREEN
        """

        thresh_hue = 0, 15, 45, 90   # threshold to separate red yellow green and rest in the HSV space
        thresh_area_ratio = 0.01     # classification will be ignored if area ratio is smaller

        result = 3 # if nothing detected -> UNKNOWN

        y0, x0, y1, x1 = bbox
        crop = image[y0:y1, x0:x1]


        hsv = cv2.cvtColor (crop, cv2.COLOR_RGB2HSV)
        mask = np.zeros_like(hsv)

        # both hsv and mask are of shape (rows, cols, 3)
        for i in range(3):
            lower = np.array ([ thresh_hue[i  ],   0, 200 ])
            upper = np.array ([ thresh_hue[i+1], 255, 255 ])

            mask [..., i] = cv2.inRange (hsv, lower, upper)

#       area_ratios = np.count_nonzero(mask, axis=(0, 1)) / np.product(mask.shape[:2])
        area_ratios = np.array ([
             np.count_nonzero(mask[...,0]),
             np.count_nonzero(mask[...,1]),
             np.count_nonzero(mask[...,2]),
        ], dtype=np.float) / np.product(mask.shape[:2])

        if (area_ratios > thresh_area_ratio).any():
            result = area_ratios.argmax()

        rospy.loginfo ('TLC: Box Hue ratios = {:.3f} {:.3f} {:.3f} => {}'.format(
            area_ratios[0], area_ratios[1], area_ratios[2],
            LIGHT_NAMES [result]
        ))

        return [TrafficLight.RED, TrafficLight.YELLOW, TrafficLight.GREEN, TrafficLight.UNKNOWN] [result]





    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        result = TrafficLight.UNKNOWN

        try:
            assert image.ndim     == 3, 'Expecting images of shape (nRows, nCols, nChannels)'
            assert image.shape[2] == 3, 'Expecting the channel axis at the end'

            # SSD to yield TL boxes and scores
            tl_boxes, tl_scores = self.get_boxes(image)

            # classify only the box with highest confidence
            # result = self.classify_bbox (image, tl_boxes[0])

            # classify all detected lights
            lights = np.array ([self.classify_bbox(image, bbox) for bbox in tl_boxes])

#            rospy.loginfo ('TLC: Colours {}', format (lights))

            # in simulator require 2 or 3 lights of the same colour, 1 is not enough
            if (2 <= len(lights) <= 3) and (lights == lights[0]).all():
                result = lights[0]

            rospy.loginfo ('TLC: Overall light prediction: {}'.format (LIGHT_NAMES[result]))

        except Exception as e:
            rospy.logwarn ('TLC: {}'.format (e))

        return result



if __name__ == '__main__':
    TLC = TLClassifier()

    for fn in 'tl_0_g.png frame_009275.jpg simss0.jpg tl.jpg tl2.jpg tl3.jpg'.split() [:1]:
        image = cv2.imread(fn) [...,:3]# [::-1] # BGR to RGB
        print (fn,TLC.get_classification(image))
        print ()
