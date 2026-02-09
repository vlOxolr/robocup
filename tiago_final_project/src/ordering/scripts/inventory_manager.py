#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from threading import Lock

from ordering.msg import Inventory, InventoryItem
from ordering.srv import Reserve, ReserveResponse

def _to_int(v, default=0):
    try:
        return int(v)
    except Exception:
        return default

class InventoryManager:
    """
    独立库存管理节点：
    - 发布 /inventory (latched)
    - 提供 /inventory/reserve 服务：原子检查+扣减
    """
    def __init__(self):
        self.lock = Lock()

        initial = rospy.get_param("~initial_inventory", {})
        self.inv = {str(k): _to_int(v, 0) for k, v in initial.items()}

        self.pub = rospy.Publisher("/inventory", Inventory, queue_size=1, latch=True)
        self.srv = rospy.Service("/inventory/reserve", Reserve, self.handle_reserve)

        self.pub_rate = float(rospy.get_param("~publish_rate_hz", 1.0))
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.pub_rate, 0.1)), self._timer_cb)

        rospy.loginfo("[inventory_manager] started with inventory: %s", self.inv)
        self.publish_inventory()

    def _timer_cb(self, _evt):
        self.publish_inventory()

    def publish_inventory(self):
        msg = Inventory()
        msg.timestamp = rospy.Time.now()

        with self.lock:
            for name, qty in sorted(self.inv.items()):
                it = InventoryItem()
                it.name = name
                it.quantity = int(qty)
                msg.items.append(it)

        self.pub.publish(msg)

    def handle_reserve(self, req):
        with self.lock:
            # 1) 检查
            for it in req.items:
                name = it.name
                need = int(it.amount)
                have = int(self.inv.get(name, 0))
                if need <= 0:
                    return ReserveResponse(
                        ok=False,
                        reason=f"Invalid amount for '{name}': {need}",
                        inventory_after=self._inventory_msg_locked(),
                    )
                if have < need:
                    return ReserveResponse(
                        ok=False,
                        reason=f"Not enough '{name}' (need={need}, have={have})",
                        inventory_after=self._inventory_msg_locked(),
                    )

            # 2) 扣减
            for it in req.items:
                self.inv[it.name] = int(self.inv.get(it.name, 0)) - int(it.amount)

            after = self._inventory_msg_locked()

        self.pub.publish(after)
        return ReserveResponse(ok=True, reason="OK", inventory_after=after)

    def _inventory_msg_locked(self):
        msg = Inventory()
        msg.timestamp = rospy.Time.now()
        for name, qty in sorted(self.inv.items()):
            it = InventoryItem()
            it.name = name
            it.quantity = int(qty)
            msg.items.append(it)
        return msg

if __name__ == "__main__":
    rospy.init_node("inventory_manager", anonymous=False)
    InventoryManager()
    rospy.spin()
