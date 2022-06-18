import unittest
from unittest.mock import patch

from mirte_robot import phone
from std_msgs.msg import Bool
import rospy

class TestPhoneButton(unittest.TestCase):

    @patch('rospy.has_param')
    def test_initialize_with_empty_config(self, hasParamMock):
        # arange
        def has_param_side_effect(value):
            return False

        hasParamMock.side_effect = has_param_side_effect
        
        # act
        phoneAPI = phone.createPhone()
        
        # assert
        self.assertFalse(hasattr(phoneAPI, "phone_button_subscribers"))

    @patch('rospy.get_param')
    @patch('rospy.has_param')
    def test_initialize_with_single_item_config(self, hasParamMock, getParamMock):
        # arange
        def has_param_side_effect(value):
            return value == "/mirte/phone_button"

        # dictionary containing single phone_image_ouput instance
        def get_param_side_effect(value):
            return {
                "button_a": {
                    "name": "button_a",
                }
            }

        hasParamMock.side_effect = has_param_side_effect
        getParamMock.side_effect = get_param_side_effect
        
        # act
        phoneAPI = phone.createPhone()
        
        # assert
        self.assertTrue(hasattr(phoneAPI, "phone_button_subscribers"))
        self.assertNotEqual(phoneAPI.phone_button_subscribers.get("button_a"), None)

    @patch('rospy.get_param')
    @patch('rospy.has_param')
    def test_initialize_with_multi_item_config(self, hasParamMock, getParamMock):
        # arange
        def has_param_side_effect(value):
            return value == "/mirte/phone_button"

        # dictionary containing single phone_image_ouput instance
        def get_param_side_effect(value):
            return {
                "button_a": {
                    "name": "output_a",
                },
                "button_b": {
                    "name": "output_b",
                }
            }

        hasParamMock.side_effect = has_param_side_effect
        getParamMock.side_effect = get_param_side_effect
        
        # act
        phoneAPI = phone.createPhone()
        
        # assert
        self.assertTrue(hasattr(phoneAPI, "phone_button_subscribers"))
        self.assertNotEqual(phoneAPI.phone_button_subscribers.get("button_a"), None)
        self.assertNotEqual(phoneAPI.phone_button_subscribers.get("button_b"), None)


    @patch('rospy.get_param')
    @patch('rospy.has_param')
    def test_get_correct_button_value(self, hasParamMock, getParamMock):
        # arange
        def has_param_side_effect(value):
            return value == "/mirte/phone_button"

        # dictionary containing single phone_image_ouput instance
        def get_param_side_effect(value):
            return {
                "button_a": {
                    "name": "button_a",
                }
            }

        hasParamMock.side_effect = has_param_side_effect
        getParamMock.side_effect = get_param_side_effect

        msg = Bool()
        msg.data = True
        
        # act
        phoneAPI = phone.createPhone()
        button_subscriber = phoneAPI.phone_button_subscribers.get("button_a")
        button_subscriber.callback(msg)
        
        # assert
        self.assertEqual(phoneAPI.getButtonValue("button_a"), msg.data)
