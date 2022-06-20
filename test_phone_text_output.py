import unittest
from unittest.mock import Mock
from unittest.mock import patch

from mirte_robot import phone
import rospy

class MockPublisher:
    def __init__(self, topicName, callback, queue_size):
        self.publish = Mock()

class TestPhoneTextOutput(unittest.TestCase): 
    @patch('rospy.has_param')
    def test_initialize_with_empty_config(self, hasParamMock):
        # arange
        def has_param_side_effect(value):
            return False

        hasParamMock.side_effect = has_param_side_effect
        
        # act
        phoneAPI = phone.createPhone()
        
        # assert
        self.assertFalse(hasattr(phoneAPI, "phone_text_output"))

    @patch('rospy.get_param')
    @patch('rospy.has_param')
    def test_initialize_with_single_item_config(self, hasParamMock, getParamMock):
        # arange
        def has_param_side_effect(value):
            return value == "/mirte/phone_text_output"
        
        def get_param_side_effect(value):
            return {
                "outputA": {
                    "name": "outputA"
                }
            }

        hasParamMock.side_effect = has_param_side_effect
        getParamMock.side_effect = get_param_side_effect
        
        # act
        phoneAPI = phone.createPhone()
        
        # assert
        self.assertTrue(hasattr(phoneAPI, "phone_text_outputs"))
        self.assertNotEqual(phoneAPI.phone_text_outputs["outputA"], None)
    
    @patch('rospy.Publisher', MockPublisher)
    @patch('rospy.get_param')
    @patch('rospy.has_param')
    def test_print_text(self, hasParamMock, getParamMock):
        # arange
        def has_param_side_effect(value):
            return value == "/mirte/phone_text_output"
        
        def get_param_side_effect(value):
            return {
                "outputA": {
                    "name": "outputA"
                }
            }

        hasParamMock.side_effect = has_param_side_effect
        getParamMock.side_effect = get_param_side_effect

        phoneAPI = phone.createPhone()
        publishText = "Hi mom!"

        # act
        phoneAPI.printText("outputA", publishText)

        # assert
        mockPublish = phoneAPI.phone_text_outputs["outputA"].publish
        mockPublish.assert_called_once_with(publishText)
