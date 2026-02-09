#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Restaurant Service State Machine for TIAGo Robot

This file ONLY defines the high-level task flow (state machine).
All perception, navigation, manipulation, and speech functions
are implemented by other team members and MUST be connected later.

Author: Chenhao Guan
"""

import rospy
import smach
import smach_ros
import subprocess
import actionlib
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from std_msgs.msg import String
from std_msgs.msg import Int32

_tts_client = None

def tts_say(text: str):
    global _tts_client

    if _tts_client is None:
        rospy.loginfo("Connecting to /tts ...")
        _tts_client = actionlib.SimpleActionClient("/tts", TtsAction)
        _tts_client.wait_for_server()   # 死等直到可用
        rospy.loginfo("Connected to /tts.")

    goal = TtsGoal()
    goal.rawtext.text = text
    goal.rawtext.lang_id = "en_GB"
    goal.speakerName = ""
    goal.wait_before_speaking = 0.0

    rospy.loginfo(f"Sending TTS: {text}")
    _tts_client.send_goal(goal)



# ============================================================
# State 1: Navigate to the restaurant entrance
# ============================================================
class NavigateToEntrance(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived', 'failed'])
        self.navigator = None 

    def execute(self, userdata):
        rospy.loginfo("STATE: Navigate to restaurant entrance")

        # Step 1: Run localization EVERY time
        rospy.loginfo("Running localization before navigation")

        if rospy.has_param("/localization_done"):
            rospy.delete_param("/localization_done")

        proc = subprocess.Popen(
            ["rosrun", "navigation", "localization.py"]
        )
        
        proc.wait() 

        # Step 2: Navigation
        rospy.loginfo("Running simple_navigator to the entrance")

        proc_nav = subprocess.Popen(
            ["rosrun", "navigation", "simple_navigator.py", "1"]
        )

        proc_nav.wait()

        if proc_nav.returncode == 0:
            rospy.loginfo("Navigation to entrance finished successfully")
            return 'arrived'
        else:
            rospy.logerr("Navigation failed")
            return 'failed'
            

# ============================================================
# State 2: Play welcome speech
# ============================================================
class WelcomeSpeech(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['done']
        )

    def execute(self, userdata):
        rospy.loginfo("STATE: WelcomeSpeech")
        tts_say("Welcome to our restaurant.")
        rospy.sleep(10.0) 
        return 'done'


# ============================================================
# State 3: Count number of customers at the entrance
# ============================================================
class CountCustomers(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['counted'],
            output_keys=['num_customers']
        )

        # Configuration
        self.topic = "/guest_confirmed_count"
        self.timeout = 30.0

    def execute(self, userdata):
        rospy.loginfo("STATE: Count customers (waiting for perception result)")

        try:
            msg = rospy.wait_for_message(
                self.topic,
                Int32,
                timeout=self.timeout
            )
            userdata.num_customers = msg.data
            rospy.loginfo("Detected %d customers", msg.data)

        except rospy.ROSException:
            rospy.logwarn(
                "Timeout waiting for %s, fallback to 1 customer",
                self.topic
            )
            userdata.num_customers = 2

        return 'counted'


# ============================================================
# State 4: Decide which table to guide customers to
# ============================================================
class DecideTable(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['table_decided'],
            input_keys=['num_customers', 'table_id'],
            output_keys=['table_id']
        )

    def execute(self, userdata):
        rospy.loginfo("STATE: Decide table based on customer number")

        num = userdata.num_customers
        rospy.loginfo("Number of customers detected: %d", num)

        # Table allocation logic
        if num <= 2:
            userdata.table_id = "Table_1"
        elif num <= 4:
            userdata.table_id = "Table_2"
        else:
            rospy.logwarn("Customer number exceeds supported range, using Table_2 as fallback")
            userdata.table_id = "Table_2"

        rospy.loginfo("Assigned table: %s", userdata.table_id)
        return 'table_decided'


# ============================================================
# State 5: Ask customers to follow the robot
# ============================================================
class AskFollow(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['done'],
            input_keys=['num_customers']
        )

    def execute(self, userdata):
        rospy.loginfo("STATE: Ask customers to follow the robot")

        n = userdata.num_customers
        if n == 1:
            text = "I see one guest. Please follow me."
        else:
            text = f"I see {n} guests. Please follow me."

        tts_say(text)
        return 'done'


# ============================================================
# State 6: Navigate to the assigned table
# ============================================================
class NavigateToTable(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['arrived', 'failed'],
            input_keys=['table_id']
        )

    def execute(self, userdata):
        rospy.loginfo("STATE: Navigate to assigned table")


        table_id = userdata.table_id
        rospy.loginfo("Target table: %s", table_id)

        if table_id == "Table_1":
            rospy.loginfo("Running navigation to table 1")
            proc_nav = subprocess.Popen(
                ["rosrun", "navigation", "simple_navigator.py", "2"]
            )
        elif table_id == "Table_2":
            rospy.loginfo("Running navigation to table 2")
            proc_nav = subprocess.Popen(
                ["rosrun", "navigation", "simple_navigator.py", "3"]
            )

        else:
            rospy.logwarn("Unknown table_id: %s, navigation skipped", table_id)
            return 'failed'

        proc_nav.wait()

        if proc_nav.returncode == 0:
            rospy.loginfo("Navigation to table finished successfully")
            return 'arrived'
        else:
            rospy.logerr("Navigation to table failed")
            return 'failed'


# ============================================================
# State 7: Ask customers to place their order
# ============================================================
class AskForOrder(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['order_requested']
        )

    def execute(self, userdata):
        rospy.loginfo("STATE: Ask customers to place their order")
        tts_say("Please scan the code and start ordering.")
        return 'order_requested'


# ============================================================
# State 8: Receive customer order
# ============================================================
class ReceiveOrder(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['order_received'],
            input_keys=['order_list'],
            output_keys=['order_list']
        )

        # Import here to avoid circular dependency at load time
        from ordering.msg import Order
        self._order_msg_type = Order

    def execute(self, userdata):
        rospy.loginfo("STATE: Receive customer order")
        rospy.loginfo("Waiting for confirmed order on /orders/new ...")

        try:
            # Block until a new confirmed order is published
            msg = rospy.wait_for_message(
                "/orders/status",
                self._order_msg_type,
                timeout=90.0
            )
        except rospy.ROSException:
            rospy.logerr("Timeout while waiting for /orders/new")
            userdata.order_list = []
            return 'order_received'

        # Extract order items
        order_list = []
        for item in msg.items:
            order_list.append({
                "name": item.name,
                "amount": int(item.amount)
            })

        userdata.order_list = order_list

        rospy.loginfo(
            "Order received: order_id=%s table=%s items=%s",
            msg.order_id,
            msg.table_id,
            userdata.order_list
        )

        return 'order_received'


# ============================================================
# State 9: Navigate to food pickup counter
# ============================================================
class NavigateToPickup(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['arrived', 'failed']
        )

        # Wait time after arrival (seconds)
        self.wait_time = 10.0

    def execute(self, userdata):
        rospy.loginfo("STATE: Navigate to food pickup area")

        rospy.loginfo("Running navigation to pickup area")

        proc_nav = subprocess.Popen(
            ["rosrun", "navigation", "simple_navigator.py", "4"]
        )

        proc_nav.wait()

        # Call navigation module to kitchen bar
        if proc_nav.returncode != 0:
            rospy.logerr("Navigation to pickup area failed")
            return 'failed'

        rospy.loginfo("Arrived at pickup area")

        rospy.loginfo(
            "Waiting %.1f seconds before starting grasping...",
            self.wait_time
        )

        start_time = rospy.Time.now()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= self.wait_time:
                break
            rate.sleep()
        
        rospy.loginfo("Pickup waiting time finished")
        return 'arrived'


# ============================================================
# State 10: Detect and grasp food item
# ============================================================
class PickUpFood(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['picked'],
            input_keys=['order_list']
        )

        # Map dish name to grasp launch file
        self.launch_map = {
            "Apple":  "grasp_apple.launch",
            "Cola": "grasp_bottel.launch",
            "Burger": "grasp_burger.launch",
        }

        self.max_retry = -1

    def execute(self, userdata):
        rospy.loginfo("STATE: PickUpFood")

        if not userdata.order_list:
            rospy.logwarn("Order list empty, nothing to pick")
            return 'picked'

        # Take the first dish in the order
        current = userdata.order_list[0]
        dish_name = current["name"]

        rospy.loginfo("Preparing to grasp dish: %s", dish_name)

        if dish_name not in self.launch_map:
            rospy.logerr("No grasp launch defined for dish: %s", dish_name)
            rospy.logwarn("Skipping grasp and continuing")
            return 'picked'

        launch_file = self.launch_map[dish_name]
        rospy.loginfo("Launching grasp: %s", launch_file)
        retry_count = 0

        # Run grasp launch
        while not rospy.is_shutdown():
            retry_count += 1
            rospy.loginfo(
                "Grasp attempt %d for dish: %s",
                retry_count, dish_name
            )

            # Launch grasp
            proc = subprocess.Popen([
                "roslaunch",
                "grasp",
                launch_file
            ])

            proc.wait()

            if proc.returncode == 0:
                rospy.loginfo("Grasp launch finished normally → assume SUCCESS")
                rospy.loginfo("Calling tuck_arm after successful grasp...")
                tuck_proc = subprocess.Popen([
                    "rosrun",
                    "grasp",
                    "tuck_arm.py"
                ])
                tuck_proc.wait()

                rospy.loginfo("Tuck arm finished")
                return 'picked'
            else:
                rospy.logwarn(
                    "Grasp launch returned code %d → retrying...",
                    proc.returncode
                )

            # Optional: limit retry count
            if self.max_retry > 0 and retry_count >= self.max_retry:
                rospy.logerr(
                    "Reached max grasp retry (%d), giving up",
                    self.max_retry
                )
                return 'picked'

            rospy.sleep(1.0)  # Small pause before retry


# ============================================================
# State 11: Return to the table
# ============================================================
class ReturnToTable(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['arrived', 'failed'],
            input_keys=['table_id']
        )

        # Do NOT create navigator here
        self.wait_time = 10.0

    def execute(self, userdata):
        rospy.loginfo("STATE: Return to customer table")

        table_id = userdata.table_id
        rospy.loginfo("Returning to table: %s", table_id)

        if table_id == "Table_1":
            func_id = "2"
        elif table_id == "Table_2":
            func_id = "3"
        else:
            rospy.logwarn("Unknown table_id: %s, navigation skipped", table_id)
            return 'failed'

        proc_nav = subprocess.Popen(
            ["rosrun", "navigation", "simple_navigator.py", func_id]
        )
        proc_nav.wait()

        if proc_nav.returncode != 0:
            rospy.logerr("Navigation back to table failed")
            return 'failed'

        rospy.loginfo("Arrived at table")

        rospy.loginfo(
            "Waiting %.1f seconds before placing...",
            self.wait_time
        )

        start_time = rospy.Time.now()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= self.wait_time:
                break
            rate.sleep()

        rospy.loginfo("Table waiting time finished")
        return 'arrived'
        

# ============================================================
# State 12: Place food on the table
# ============================================================
class PlaceFood(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['placed']
        )

        self.max_retry = -1  # -1 = infinite retry

    def execute(self, userdata):
        rospy.loginfo("STATE: Place food on table")

        retry_count = 0

        while not rospy.is_shutdown():
            retry_count += 1
            rospy.loginfo("Place attempt %d", retry_count)

            rospy.loginfo("Launching grasp_place.launch")

            proc = subprocess.Popen([
                "roslaunch",
                "grasp",
                "grasp_place.launch"
            ])

            proc.wait()
            rospy.sleep(0.5)

            # Check placing result
            if proc.returncode == 0:
                rospy.loginfo("Grasp launch finished normally → assume SUCCESS")
                tuck_proc = subprocess.Popen([
                    "rosrun",
                    "grasp",
                    "tuck_arm.py"
                ])
                tuck_proc.wait()

                rospy.loginfo("Tuck arm finished")
                return 'placed'
            else:
                rospy.logwarn(
                    "Grasp launch returned code %d → retrying...",
                    proc.returncode
                )

            if self.max_retry > 0 and retry_count >= self.max_retry:
                rospy.logerr(
                    "Reached max place retry (%d), giving up",
                    self.max_retry
                )
                return 'placed'

            rospy.sleep(1.0)  # Small pause before retry


# ============================================================
# State 13: Check whether all dishes are served
# ============================================================
class CheckOrderCompletion(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['complete', 'incomplete'],
            input_keys=['order_list'],
            output_keys=['order_list']
        )

    def execute(self, userdata):
        rospy.loginfo("STATE: Check order completion")

        order_list = userdata.order_list

        # Safety check
        if not order_list:
            rospy.loginfo("All dishes have been served")
            return 'complete'

        # We assume the first item was just served
        current = order_list[0]
        dish_name = current["name"]
        current_amount = int(current["amount"])

        rospy.loginfo(
            "Updating order: served one '%s' (remaining before update: %d)",
            dish_name, current_amount
        )

        # Decrease amount or remove item
        if current_amount > 1:
            current["amount"] = current_amount - 1
        else:
            # amount == 1, remove this dish from the list
            order_list.pop(0)

        userdata.order_list = order_list

        # Check if there are remaining dishes
        if len(order_list) == 0:
            rospy.loginfo("Order fully completed")
            return 'complete'
        else:
            rospy.loginfo("Remaining dishes to serve: %s", order_list)
            return 'incomplete'

    rospy.sleep(1.0)


# ============================================================
# State 14: Announce service completion (Enjoy your meal)
# ============================================================
class AnnounceServiceComplete(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['done']
        )

    def execute(self, userdata):
        rospy.loginfo("STATE: Announce service completion")
        tts_say("Service finished. Enjoy your meal.")
        return 'done'
        
    rospy.sleep(1.0)


# ============================================================
# Main Trajectory
# ============================================================
def main():
    rospy.init_node('restaurant_service_state_machine')

    sm = smach.StateMachine(outcomes=['END'])

    with sm:
        smach.StateMachine.add('NAVIGATE_TO_ENTRANCE', NavigateToEntrance(),
                               transitions={'arrived': 'WELCOME_SPEECH',
                                            'failed': 'NAVIGATE_TO_ENTRANCE'})

        smach.StateMachine.add('WELCOME_SPEECH', WelcomeSpeech(),
                               transitions={'done': 'COUNT_CUSTOMERS'})

        smach.StateMachine.add('COUNT_CUSTOMERS', CountCustomers(),
                               transitions={'counted': 'DECIDE_TABLE'})

        smach.StateMachine.add('DECIDE_TABLE', DecideTable(),
                               transitions={'table_decided': 'ASK_FOLLOW'})

        smach.StateMachine.add('ASK_FOLLOW', AskFollow(),
                               transitions={'done': 'NAVIGATE_TO_TABLE'})

        smach.StateMachine.add('NAVIGATE_TO_TABLE', NavigateToTable(),
                               transitions={'arrived': 'ASK_FOR_ORDER',
                                            'failed': 'NAVIGATE_TO_TABLE'})

        smach.StateMachine.add('ASK_FOR_ORDER', AskForOrder(),
                               transitions={'order_requested': 'RECEIVE_ORDER'})

        smach.StateMachine.add('RECEIVE_ORDER', ReceiveOrder(),
                               transitions={'order_received': 'NAVIGATE_TO_PICKUP'})

        smach.StateMachine.add('NAVIGATE_TO_PICKUP', NavigateToPickup(),
                               transitions={'arrived': 'PICK_UP_FOOD',
                                            'failed': 'NAVIGATE_TO_PICKUP'})

        smach.StateMachine.add('PICK_UP_FOOD', PickUpFood(),
                               transitions={'picked': 'RETURN_TO_TABLE'})

        smach.StateMachine.add('RETURN_TO_TABLE', ReturnToTable(),
                               transitions={'arrived': 'PLACE_FOOD',
                                            'failed': 'RETURN_TO_TABLE'})

        smach.StateMachine.add('PLACE_FOOD', PlaceFood(),
                               transitions={'placed': 'CHECK_ORDER_COMPLETION'})

        smach.StateMachine.add('CHECK_ORDER_COMPLETION', CheckOrderCompletion(),
                            transitions={'complete': 'ANNOUNCE_SERVICE_COMPLETE',
                                            'incomplete': 'NAVIGATE_TO_PICKUP'})
        smach.StateMachine.add('ANNOUNCE_SERVICE_COMPLETE', AnnounceServiceComplete(),
                            transitions={'done': 'NAVIGATE_TO_ENTRANCE'})

    outcome = sm.execute()
    rospy.spin()


if __name__ == '__main__':
    main()
