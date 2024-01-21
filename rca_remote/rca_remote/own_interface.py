#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rca_msgs.action import RcaTask
import threading

threading.Thread(target=lambda: rclpy.init()).start()

action_client = ActionClient(Node("own_interface"), RcaTask, "task_server")





class LaunchRequestHandler():
        pass
        
       # goal = RcaTask.Goal()
       # goal.task_number = 0
       # action_client.send_goal_async(goal)
       # return handler_input.response_builder.response


class PickIntentHandler():
    pass
        
        # goal = RcaTask.Goal()
        # goal.task_number = 1
        # action_client.send_goal_async(goal)

        # return handler_input.response_builder.response


class SleepIntentHandler():
    pass
        # goal = RcaTask.Goal()
        # goal.task_number = 2
        # action_client.send_goal_async(goal)

        # return handler_input.response_builder.response
    

class WakeIntentHandler():
        pass

        goal = RcaTask.Goal()
#        goal.task_number = 0
 #       action_client.send_goal_async(goal)
  #      
   #     return handler_input.response_builder.response
    
class AllExceptionHandler():
        pass

    




if __name__=='__main__':
    pass