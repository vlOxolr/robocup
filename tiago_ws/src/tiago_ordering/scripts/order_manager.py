#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from threading import Lock

from tiago_ordering.msg import Order, OrderStatusUpdate
from tiago_ordering.srv import SetOrderStatus, SetOrderStatusResponse

STATUS_CREATED   = "CREATED"
STATUS_CONFIRMED = "CONFIRMED"
STATUS_DENIED    = "DENIED"
STATUS_SERVING   = "SERVING"
STATUS_SERVED    = "SERVED"

VALID_STATUSES = {STATUS_CREATED, STATUS_CONFIRMED, STATUS_DENIED, STATUS_SERVING, STATUS_SERVED}

def _u(s: str) -> str:
    return (s or "").strip().upper()

def _clean_reason(s: str, max_len: int = 120) -> str:
    s = (s or "").strip()
    s = s.replace("\n", " ").replace("\r", " ")
    if len(s) > max_len:
        s = s[:max_len]
    return s

class OrderManager:
    """
    唯一合法的“订单状态修改入口”：
    - 外部只允许改 status + reason
    - 其它内容（items/special_request/table_id）严格禁止外部传入/更改
    """
    def __init__(self):
        self.lock = Lock()
        self.cache = {}  # order_id -> Order(latest)

        rospy.Subscriber("/orders/new", Order, self._on_order_msg, queue_size=50)
        rospy.Subscriber("/orders/status", Order, self._on_order_msg, queue_size=200)

        self.pub_update = rospy.Publisher("/orders/update", OrderStatusUpdate, queue_size=50)
        self.pub_status = rospy.Publisher("/orders/status", Order, queue_size=50)  # 可选：同步发 status

        self.srv = rospy.Service("/orders/set_status", SetOrderStatus, self.handle_set_status)

        rospy.loginfo("[order_manager] started. service=/orders/set_status publish=/orders/update")

    def _on_order_msg(self, msg: Order):
        oid = str(msg.order_id).strip()
        if not oid:
            return
        # 复制一份缓存（避免引用被外部修改）
        cp = Order()
        cp.order_id = msg.order_id
        cp.table_id = msg.table_id
        cp.timestamp = msg.timestamp
        cp.special_request = msg.special_request
        cp.status = msg.status
        cp.items = list(msg.items)

        with self.lock:
            self.cache[oid] = cp

    def handle_set_status(self, req):
        oid = str(req.order_id).strip()
        st = _u(req.status)
        rs = _clean_reason(req.reason)

        if not oid:
            return SetOrderStatusResponse(ok=False, reason="order_id is empty")
        if st not in VALID_STATUSES:
            return SetOrderStatusResponse(ok=False, reason=f"invalid status '{req.status}'")

        with self.lock:
            base = self.cache.get(oid)

        if not base:
            # 强约束：必须命中缓存，避免发布“无 table_id 的幽灵更新”
            return SetOrderStatusResponse(ok=False, reason="order not found in cache (subscribe /orders/new first)")

        old = _u(base.status)

        # 1) 发布“更新消息”（带 reason）
        up = OrderStatusUpdate()
        up.order_id = oid
        up.status = st
        up.reason = rs
        up.timestamp = rospy.Time.now()
        self.pub_update.publish(up)

        # 2) 可选：同步发布 /orders/status（只改 status，其它内容完全沿用 base）
        out = Order()
        out.order_id = base.order_id
        out.table_id = base.table_id
        out.special_request = base.special_request
        out.items = list(base.items)
        out.status = st
        out.timestamp = rospy.Time.now()
        self.pub_status.publish(out)

        # 3) 更新缓存中的 status（reason 不写入 Order）
        base.status = st
        base.timestamp = out.timestamp
        with self.lock:
            self.cache[oid] = base

        # 日志（统一风格）
        if rs:
            rospy.loginfo("[order_manager] order status changed: table=%s order_id=%s %s -> %s reason=%s",
                          out.table_id, oid, old, st, rs)
        else:
            rospy.loginfo("[order_manager] order status changed: table=%s order_id=%s %s -> %s",
                          out.table_id, oid, old, st)

        return SetOrderStatusResponse(ok=True, reason="OK")

if __name__ == "__main__":
    rospy.init_node("order_manager", anonymous=False)
    OrderManager()
    rospy.spin()
