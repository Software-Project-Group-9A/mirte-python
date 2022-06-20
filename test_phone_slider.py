import unittest
from unittest.mock import patch

from mirte_robot import phone
from std_msgs.msg import Int32
import rospy

class TestPhoneSlider(unittest.TestCase):

    @patch('rospy.has_param')
    def test_initialize_with_empty_config(self, hasParamMock):
        # arange
        def has_param_side_effect(value):
            return False

        hasParamMock.side_effect = has_param_side_effect
        
        # act
        phoneAPI = phone.createPhone()
        
        # assert
        self.assertFalse(hasattr(phoneAPI, "phone_slider_subscribers"))

    @patch('rospy.get_param')
    @patch('rospy.has_param')
    def test_initialize_with_single_item_config(self, hasParamMock, getParamMock):
        # arange
        def has_param_side_effect(value):
            return value == "/mirte/phone_slider"

        # dictionary containing single phone_image_ouput instance
        def get_param_side_effect(value):
            return {
                "slider_a": {
                    "name": "slider_a",
                }
            }

        hasParamMock.side_effect = has_param_side_effect
        getParamMock.side_effect = get_param_side_effect
        
        # act
        phoneAPI = phone.createPhone()
        
        # assert
        self.assertTrue(hasattr(phoneAPI, "phone_slider_subscribers"))
        self.assertNotEqual(phoneAPI.phone_slider_subscribers.get("slider_a"), None)

    @patch('rospy.get_param')
    @patch('rospy.has_param')
    def test_initialize_with_multi_item_config(self, hasParamMock, getParamMock):
        # arange
        def has_param_side_effect(value):
            return value == "/mirte/phone_slider"

        # dictionary containing single phone_image_ouput instance
        def get_param_side_effect(value):
            return {
                "slider_a": {
                    "name": "output_a",
                },
                "slider_b": {
                    "name": "output_b",
                }
            }

        hasParamMock.side_effect = has_param_side_effect
        getParamMock.side_effect = get_param_side_effect
        
        # act
        phoneAPI = phone.createPhone()
        
        # assert
        self.assertTrue(hasattr(phoneAPI, "phone_slider_subscribers"))
        self.assertNotEqual(phoneAPI.phone_slider_subscribers.get("slider_a"), None)
        self.assertNotEqual(phoneAPI.phone_slider_subscribers.get("slider_b"), None)


    @patch('rospy.get_param')
    @patch('rospy.has_param')
    def test_get_correct_slider_value(self, hasParamMock, getParamMock):
        # arange
        def has_param_side_effect(value):
            return value == "/mirte/phone_slider"

        # dictionary containing single phone_image_ouput instance
        def get_param_side_effect(value):
            return {
                "slider_a": {
                    "name": "slider_a",
                }
            }

        hasParamMock.side_effect = has_param_side_effect
        getParamMock.side_effect = get_param_side_effect

        msg = Int32()
        msg.data = 32
        
        # act
        phoneAPI = phone.createPhone()
        slider_subscriber = phoneAPI.phone_slider_subscribers.get("slider_a")
        slider_subscriber.callback(msg)
        
        # assert
        self.assertEqual(phoneAPI.getSliderValue("slider_a"), msg.data)
