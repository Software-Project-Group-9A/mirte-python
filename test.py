import unittest
from time import sleep
from unittest.mock import Mock
from unittest.mock import patch
from unittest.mock import mock_open

from mirte_robot import phone
from sensor_msgs.msg import CompressedImage
import rospy

class MockPublisher:
    def __init__(self, topicName, callback, queue_size):
        self.publish = Mock()

class MockPhoneImageOutput:
    def setImage(self, imageLocation):
        self.imageLocation = imageLocation

class TestPhoneImageOutput(unittest.TestCase):

    @patch('rospy.has_param')
    def test_initialize_with_empty_config(self, hasParamMock):
        # arange
        def has_param_side_effect(value):
            return False

        hasParamMock.side_effect = has_param_side_effect
        
        # act
        phoneAPI = phone.createPhone()
        
        # assert
        self.assertFalse(hasattr(phoneAPI, "phone_image_outputs"))

    @patch('rospy.get_param')
    @patch('rospy.has_param')
    def test_initialize_with_single_item_config(self, hasParamMock, getParamMock):
        # arange
        def has_param_side_effect(value):
            return value == "/mirte/phone_image_output"

        # dictionary containing single phone_image_ouput instance
        def get_param_side_effect(value):
            return {
                "output_a": {
                    "name": "output_a",
                }
            }

        hasParamMock.side_effect = has_param_side_effect
        getParamMock.side_effect = get_param_side_effect
        
        # act
        phoneAPI = phone.createPhone()
        
        # assert
        self.assertTrue(hasattr(phoneAPI, "phone_image_outputs"))
        self.assertNotEqual(phoneAPI.phone_image_outputs.get("output_a"), None)

    @patch('rospy.get_param')
    @patch('rospy.has_param')
    def test_initialize_with_multi_item_config(self, hasParamMock, getParamMock):
        # arange
        def has_param_side_effect(value):
            return value == "/mirte/phone_image_output"

        # dictionary containing single phone_image_ouput instance
        def get_param_side_effect(value):
            return {
                "output_a": {
                    "name": "output_a",
                },
                "output_b": {
                    "name": "output_b",
                }
            }

        hasParamMock.side_effect = has_param_side_effect
        getParamMock.side_effect = get_param_side_effect
        
        # act
        phoneAPI = phone.createPhone()
        
        # assert
        self.assertTrue(hasattr(phoneAPI, "phone_image_outputs"))
        self.assertNotEqual(phoneAPI.phone_image_outputs.get("output_a"), None)
        self.assertNotEqual(phoneAPI.phone_image_outputs.get("output_b"), None)


    def test_correct_image_output_call(self):
        # arange
        phoneAPI = phone.createPhone()

        phoneAPI.phone_image_outputs = {}
        phoneAPI.phone_image_outputs['image_out'] = MockPhoneImageOutput()

        # act
        phoneAPI.setPhoneImage('image_out', 'zoef_logo')

        # assert
        expectedLocation = "/usr/local/src/mirte/mirte-oled-images/images/zoef_logo.png"
        self.assertEqual(phoneAPI.phone_image_outputs['image_out'].imageLocation, expectedLocation)

    @patch('builtins.open', mock_open(read_data='test_content'))
    def test_open_correct_file_location(self):
        # arange
        phoneAPI = phone.createPhone()
        imageOutput = phone.PhoneImageOutput("unit_tests/test_image")
        fileName = "test"

        # act
        imageOutput.setImage(fileName)

        # assert
        open.assert_called_once_with(fileName, "rb")

    # mock the publisher
    @patch('builtins.open', mock_open(read_data='test_content'))
    @patch('rospy.Publisher', MockPublisher)
    def test_publish_correct_message(self):
        # arange
        phoneAPI = phone.createPhone()
        topicName = "unit_tests/test_image"
        imageOutput = phone.PhoneImageOutput(topicName)

        # act
        imageOutput.setImage("test")

        # assert
        mockPublish = imageOutput.publisher.publish
        mockPublish.assert_called_once()

        expectedMessage = CompressedImage()
        expectedMessage.format = "png"
        expectedMessage.data = 'test_content'

        mockPublish.assert_called_once_with(expectedMessage)

class TestPhoneFlashlight(unittest.TestCase): 
    @patch('rospy.has_param')
    def test_initialize_with_empty_config(self, hasParamMock):
        # arange
        def has_param_side_effect(value):
            return False

        hasParamMock.side_effect = has_param_side_effect
        
        # act
        phoneAPI = phone.createPhone()
        
        # assert
        self.assertFalse(hasattr(phoneAPI, "phone_flashlight"))

    @patch('rospy.has_param')
    def test_initialize_with_single_item_config(self, hasParamMock):
        # arange
        def has_param_side_effect(value):
            return value == "/mirte/phone_flashlight"

        hasParamMock.side_effect = has_param_side_effect
        
        # act
        phoneAPI = phone.createPhone()
        
        # assert
        self.assertTrue(hasattr(phoneAPI, "phone_flashlight"))
    
    @patch('rospy.Publisher', MockPublisher)
    @patch('rospy.has_param')
    def test_turn_on_flashlight(self, hasParamMock):
        # arange
        def has_param_side_effect(value):
            return value == "/mirte/phone_flashlight"

        hasParamMock.side_effect = has_param_side_effect
        phoneAPI = phone.createPhone()

        # act
        phoneAPI.setFlashlight(True)

        # assert
        self.assertTrue(hasattr(phoneAPI, "phone_flashlight"))
        self.assertTrue(hasattr(phoneAPI.phone_flashlight, "publish"))
        phoneAPI.phone_flashlight.publish.assert_called_once_with(True)

if __name__ == '__main__':
    unittest.main()

# phone = phone.createPhone()

# print('starting tests')

# # test 1
# phone.phone_image_outputs = {}
# phone.phone_image_outputs['image_out'] = MockPhoneImageOutput()

# phone.setPhoneImage('image_out', 'zoef_logo')

# assert phone.phone_image_outputs['image_out'].imageLocation == "/usr/local/src/mirte/mirte-oled-images/images/zoef_logo.png" 

# # test 