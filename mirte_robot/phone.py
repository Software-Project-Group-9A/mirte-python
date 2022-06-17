#!/usr/bin/env python
import time
import rospy
import rosservice
import signal
import sys
import math
import atexit

from sensor_msgs.msg import CompressedImage

phone = {}

class Phone():
    """ Robot API

        This class allows you to control the robot from Python. The getters and setters
        are just wrappers calling ROS topics or services.
    """


    def __init__(self):

        # Start timing
        self.begin_time = time.time()
        self.last_call = 0

        rospy.init_node('mirte_python_api', anonymous=True)

        # Publishers for sensorlib ImageSubscriber instances
        if rospy.has_param("/mirte/phone_image_output"):
            phone_image_outputs = rospy.get_param("/mirte/phone_image_output")
            self.phone_image_outputs = {}
            # for each ImageSubscriber, create a publisher
            for actuator in phone_image_outputs:
                self.phone_image_outputs[actuator] = PhoneImageOutput(phone_image_outputs[actuator]["name"])
        
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def setPhoneImage(self, imageSubscriber, imageName):
        """Shows an image on an ImageSubscriber located on a phone. 

        Parameters:
            imageSubscriber (str): The name of the ImageSubscriber as specified in the settings.
            image (str): Image name as defined in the images folder of the mirte-oled-images repository (excl file extension). 
            Image must be a png file.
        """
        imageLocation = "/usr/local/src/mirte/mirte-oled-images/images/" + imageName + ".png"
        self.phone_image_outputs[imageSubscriber].setImage(imageLocation)

    def _signal_handler(self, sig, frame):
        sys.exit()

## PhoneImageOutput is used to publish an image to ROS. In turn, a ImageSubscriber from mirtesensorlib
## can then draw this published image to the smartphone screen.
## Images are currently taken from the same folder as the mirte OLED images
class PhoneImageOutput:
    def __init__(self, name):
        self.name = name
        self.publisher = rospy.Publisher('/mirte/phone_image_output/' + name, CompressedImage, queue_size=10)

    def setImage(self, imageLocation):
        """Shows an image on a ImageSubscriber located on a phone. 

        Parameters:
            imageSubscriber (str): The name of the ImageSubscriber as specified in the settings.
            image (str): location of image file. Must be png file.
        """
        # open image file
        imageFile = open(imageLocation, "rb")
        # create CompressedImage message
        msg = CompressedImage()
        msg.format = 'png'
        msg.data = imageFile.read()
        # publish
        self.publisher.publish(msg)
        # close image file
        imageFile.close()


# We need a special function to initiate the Robot() because the main.py need to call the
# init_node() (see: https://answers.ros.org/question/266612/rospy-init_node-inside-imported-file/)
def createPhone():
    """Creates and return instance of the phone class.

    Returns:
       Robot: The initialize Robot class.
    """

    global phone
    phone = Phone()
    return phone
