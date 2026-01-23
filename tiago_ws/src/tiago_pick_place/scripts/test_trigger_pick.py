#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import Trigger


def main():
    rospy.init_node("test_trigger_pick")

    srv_name = rospy.get_param("~service", "/tiago_pick_place/trigger_pick")
    target_ns = rospy.get_param("~target_ns", "apple")

    # Set param on pick_place node namespace
    rospy.set_param("/tiago_pick_place/target_ns", target_ns)
    rospy.loginfo("Set /tiago_pick_place/target_ns = %s", target_ns)

    rospy.loginfo("Calling service: %s", srv_name)
    rospy.wait_for_service(srv_name, timeout=10.0)
    call = rospy.ServiceProxy(srv_name, Trigger)
    resp = call()
    rospy.loginfo("success=%s message=%s", str(resp.success), resp.message)


if __name__ == "__main__":
    main()
