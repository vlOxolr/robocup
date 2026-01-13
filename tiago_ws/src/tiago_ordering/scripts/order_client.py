# -*- coding: utf-8 -*-
import rospy
from tiago_ordering.srv import SetOrderStatus, SetOrderStatusRequest

VALID = {"CREATED", "CONFIRMED", "DENIED", "SERVING", "SERVED"}

def _u(s): return (s or "").strip().upper()

class OrderClient:
    def __init__(self, service_name="/orders/set_status", timeout=3.0):
        self.service_name = service_name
        rospy.wait_for_service(service_name, timeout=timeout)
        self._proxy = rospy.ServiceProxy(service_name, SetOrderStatus)

    def set_status(self, order_id: str, status: str, reason: str = "") -> None:
        oid = (order_id or "").strip()
        st = _u(status)
        if not oid:
            raise ValueError("order_id is empty")
        if st not in VALID:
            raise ValueError(f"invalid status: {status}")

        req = SetOrderStatusRequest()
        req.order_id = oid
        req.status = st
        req.reason = (reason or "").strip()

        resp = self._proxy(req)
        if not resp.ok:
            raise RuntimeError(resp.reason)
