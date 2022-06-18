import unittest
from unittest.mock import Mock
from unittest.mock import patch

from mirte_robot import phone
import rospy

class MockPublisher:
    def __init__(self, topicName, callback, queue_size):
        self.publish = Mock()

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
        self.assertFalse(hasattr(phoneAPI, "phone_flashlights"))

    @patch('rospy.get_param')
    @patch('rospy.has_param')
    def test_initialize_with_single_item_config(self, hasParamMock, getParamMock):
        # arange
        def has_param_side_effect(value):
            return value == "/mirte/phone_flashlight"

        def get_param_side_effect(value):
            return {
                "flashlight": {
                    "name": "flashlight",
                }
            }

        hasParamMock.side_effect = has_param_side_effect
        getParamMock.side_effect = get_param_side_effect
        
        # act
        phoneAPI = phone.createPhone()
        
        # assert
        self.assertTrue(hasattr(phoneAPI, "phone_flashlights"))
        self.assertNotEqual(phoneAPI.phone_flashlights["flashlight"], None)
    
    @patch('rospy.Publisher', MockPublisher)
    @patch('rospy.get_param')
    @patch('rospy.has_param')
    def test_turn_on_flashlight(self, hasParamMock, getParamMock):
        # arange
        def has_param_side_effect(value):
            return value == "/mirte/phone_flashlight"

        def get_param_side_effect(value):
            return {
                "flashlight": {
                    "name": "flashlight",
                }
            }

        hasParamMock.side_effect = has_param_side_effect
        getParamMock.side_effect = get_param_side_effect

        phoneAPI = phone.createPhone()

        # act
        phoneAPI.setFlashlight("flashlight", True)

        # assert
        mockPublish = phoneAPI.phone_flashlights["flashlight"].publish
        mockPublish.assert_called_once_with(True)

    @patch('rospy.Publisher', MockPublisher)
    @patch('rospy.get_param')
    @patch('rospy.has_param')
    def test_turn_off_flashlight(self, hasParamMock, getParamMock):
        # arange
        def has_param_side_effect(value):
            return value == "/mirte/phone_flashlight"

        def get_param_side_effect(value):
            return {
                "flashlight": {
                    "name": "flashlight",
                }
            }

        hasParamMock.side_effect = has_param_side_effect
        getParamMock.side_effect = get_param_side_effect

        phoneAPI = phone.createPhone()

        # act
        phoneAPI.setFlashlight("flashlight", True)

        # assert
        mockPublish = phoneAPI.phone_flashlights["flashlight"].publish
        mockPublish.assert_called_once_with(True)
