#!/usr/bin/env python
import time
import rospy
import rosservice
import signal
import sys
import math
import atexit

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import String

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

        # Subscribers for phone sliders 
        if rospy.has_param("/mirte/phone_slider"):
            phone_sliders = rospy.get_param("/mirte/phone_slider")
            self.phone_slider_subscribers = {}
            for sensor in phone_sliders:
                self.phone_slider_subscribers[sensor] = TopicSubscriber(
                    '/mirte/phone_slider/' + phone_sliders[sensor]["name"], Int32)

        # Publishers for sensorlib ImageSubscriber instances
        if rospy.has_param("/mirte/phone_image_output"):
            phone_image_outputs = rospy.get_param("/mirte/phone_image_output")
            self.phone_image_outputs = {}
            # for each ImageSubscriber, create a publisher
            for actuator in phone_image_outputs:
                self.phone_image_outputs[actuator] = PhoneImageOutput(phone_image_outputs[actuator]["name"])

        # Publishers for sensorlib FlashlightSubscriber
        # only one instance of flashlight is ever expected, with the name instance
        if rospy.has_param("/mirte/phone_flashlight"):
            phone_flashlights = rospy.get_param("/mirte/phone_flashlight")
            self.phone_flashlights = {}
            for flashlight in phone_flashlights:
                topicName = '/mirte/phone_flashlight/' + phone_flashlights[flashlight]["name"]
                self.phone_flashlights[flashlight] = rospy.Publisher(topicName, Bool, queue_size=10)

        # Publishers for sensorlib TextSubscribers
        if rospy.has_param("/mirte/phone_text_output"):
            phone_text_outputs = rospy.get_param("/mirte/phone_text_output")
            self.phone_text_outputs = {}
            for text in phone_text_outputs:
                topicName = '/mirte/phone_text_output/' + phone_text_outputs[text]["name"]
                self.phone_text_outputs[text] = rospy.Publisher(topicName, String, queue_size=10)
      
        
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def getSliderValue(self, slider):
        """Gets the slider value of slider.

        Parameters:
            slider (str): The name of the slider as specified in the settings.

        Returns:
            int: Value of slider.
        """

        value = self.phone_slider_subscribers[slider].getValue()
        return value.data
    
    def setPhoneImage(self, imageSubscriber, imageName):
        """Shows an image on an ImageSubscriber located on a phone. 

        Parameters:
            imageSubscriber (str): The name of the ImageSubscriber as specified in the settings.
            image (str): Image name as defined in the images folder of the mirte-oled-images repository (excl file extension). 
            Image must be a png file.
        """
        imageLocation = "/usr/local/src/mirte/mirte-oled-images/images/" + imageName + ".png"
        self.phone_image_outputs[imageSubscriber].setImage(imageLocation)
    
    def setFlashlight(self, name, state):
        """Turns the flashlight on (state=True) or off (state=False)

        Parameters:
            flashlight (str): The name of the sensor as defined in the configuration.
            state (bool): The state in which the flashlight should change.
                          true is on, false is off.

        Returns:
            bool: True if set successfully.
        """
        flash = self.phone_flashlights[name]
        return flash.publish(state)

    def printText(self, publisher, text):
        """Prints the text received on a topic.

        Parameters:
            flashlight (str): The name of the sensor as defined in the configuration.
            state (bool): The state in which the flashlight should change.
                          true is on, false is off.

        Returns:
            bool: True if set successfully.
        """

        
        text_pub = self.phone_text_outputs[publisher]
        return text_pub.publish(text)

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

# TopicSubscriber subscribes to a topic and can be used to
# save the last received value.

class TopicSubscriber:

    def __init__(self, topic_name, messageType):
        self.value = messageType()
        rospy.Subscriber(topic_name, messageType, self.callback)

    def callback(self, data):
        self.value = data

    def getValue(self):
        return self.value


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
