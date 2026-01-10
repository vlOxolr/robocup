#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import uuid
from threading import Lock
from datetime import datetime

from flask import Flask, request, redirect, url_for, render_template

from tiago_ordering.msg import Order, OrderItem
from tiago_ordering.srv import Reserve, ReserveRequest
from tiago_ordering.msg import ReserveItem

STATUS_CREATED   = "CREATED"
STATUS_CONFIRMED = "CONFIRMED"
STATUS_SERVING   = "SERVING"
STATUS_SERVED    = "SERVED"
STATUS_DENIED    = "DENIED"

ACTIVE_STATUSES = {STATUS_CREATED, STATUS_CONFIRMED, STATUS_SERVING}
BLOCKING_STATUSES = {STATUS_CONFIRMED, STATUS_SERVING}

class OrderStore:
    def __init__(self):
        self.lock = Lock()
        self.table_active = {}   # table_id -> order_id
        self.orders = {}         # order_id -> data
        self.published = set()   # published order_ids
        self.table_latest_status = {}  # table_id -> status (from /orders/status)


    def get_active_order_id(self, table_id):
        with self.lock:
            return self.table_active.get(table_id)

    def set_active(self, table_id, order_id):
        with self.lock:
            self.table_active[table_id] = order_id

    def clear_active_if_matches(self, table_id, order_id):
        with self.lock:
            if self.table_active.get(table_id) == order_id:
                self.table_active.pop(table_id, None)

    def put_order(self, order_id, data):
        with self.lock:
            self.orders[order_id] = data

    def get_order(self, order_id):
        with self.lock:
            return self.orders.get(order_id)

    def mark_published_once(self, order_id):
        with self.lock:
            if order_id in self.published:
                return False
            self.published.add(order_id)
            return True

    def update_status(self, order_id, status, reason=None):
        status = (status or "").upper()
        reason_str = (reason or "").strip()

        with self.lock:
            od = self.orders.get(order_id)
            if not od:
                return False

            old = (od.get("status") or "").upper()
            if old == status and (not reason_str):
                return False  # 状态没变且没新 reason，就不刷日志

            # 更新状态
            if old != status:
                od["status"] = status

            # 尽可能保存 reason（可用于后续页面/调试）
            if reason_str:
                od["reason"] = reason_str

            table_id = od.get("table_id", "?")

        # 锁外打印，避免阻塞
        if reason_str:
            rospy.loginfo("[web_order_server] order status changed: table=%s order_id=%s %s -> %s reason=%s",
                        table_id, order_id, old, status, reason_str)
        else:
            rospy.loginfo("[web_order_server] order status changed: table=%s order_id=%s %s -> %s",
                        table_id, order_id, old, status)
        return True

    def set_table_status(self, table_id, status):
        with self.lock:
            self.table_latest_status[str(table_id)] = str(status).upper()

    def get_table_status(self, table_id):
        with self.lock:
            return self.table_latest_status.get(str(table_id))



def make_order_id_4():
    return uuid.uuid4().hex[:4].upper()

def now_iso():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


class WebOrderServer:
    def __init__(self):
        pkg_dir = os.path.dirname(os.path.abspath(__file__))          # .../scripts
        templates_dir = os.path.join(os.path.dirname(pkg_dir), "templates")  # .../templates
        self.app = Flask(__name__, template_folder=templates_dir)
        self.store = OrderStore()

        self.menu_items = [str(x) for x in rospy.get_param("~menu_items", ["cola", "water"])]
        self.host = rospy.get_param("~host", "0.0.0.0")
        self.port = int(rospy.get_param("~port", 8080))

        self.order_new_pub = rospy.Publisher("/orders/new", Order, queue_size=10)
        self.order_status_pub = rospy.Publisher("/orders/status", Order, queue_size=10)

        self.order_status_sub = rospy.Subscriber("/orders/status", Order, self._on_status_msg, queue_size=50)

        self.status_hz = float(rospy.get_param("~status_publish_hz", 1.0))
        self.status_timer = rospy.Timer(rospy.Duration(1.0 / max(self.status_hz, 0.1)), self._publish_status_tick)

        rospy.wait_for_service("/inventory/reserve")
        self.reserve_srv = rospy.ServiceProxy("/inventory/reserve", Reserve)

        self._register_routes()

        rospy.loginfo("[web_order_server] menu_items=%s", self.menu_items)
        rospy.loginfo("[web_order_server] serving http://%s:%d", self.host, self.port)

    def _register_routes(self):
        app = self.app

        @app.route("/t/<table_id>", methods=["GET"])
        def table_entry(table_id):
            active_id = self.store.get_active_order_id(table_id)
            if active_id:
                od = self.store.get_order(active_id)
                if od and od["status"] in ACTIVE_STATUSES:
                    return redirect(url_for("status_page", order_id=active_id))
                self.store.clear_active_if_matches(table_id, active_id)

            return render_template("menu.html", table_id=table_id, menu_items=self.menu_items)

        @app.route("/submit/<table_id>", methods=["POST"])
        def submit_order(table_id):
            order_id = make_order_id_4()

            items = []
            for name in self.menu_items:
                qty = int(request.form.get(f"qty_{name}", "0") or "0")
                if qty > 0:
                    items.append({"name": name, "amount": qty})

            special = (request.form.get("special_request", "") or "").strip()

            # 0) 提交瞬间：先 CREATED 入库
            status = STATUS_CREATED
            self._store_order(table_id, order_id, items, special, status)

            # 1) 若没选菜：按你要求也 DENIED（原来是 CANCELED）
            if len(items) == 0:
                self.store.update_status(order_id, STATUS_DENIED, "No items selected.")
                return render_template(
                    "result.html",
                    ok=False,
                    reason="No items selected.",
                    table_id=table_id,
                    order_id=order_id,
                    status=STATUS_DENIED
                )

            # 2) 检查“同桌是否已有 CONFIRMED/SERVING”（topic + 本地兜底）
            blocked = False

            topic_st = self.store.get_table_status(table_id)
            if topic_st in BLOCKING_STATUSES:
                blocked = True

            if not blocked:
                active_id = self.store.get_active_order_id(table_id)
                if active_id:
                    od_active = self.store.get_order(active_id)
                    if od_active and (od_active.get("status") in BLOCKING_STATUSES):
                        blocked = True

            if blocked:
                self.store.update_status(order_id, STATUS_DENIED,
                    "This table already has an active order.")
                return render_template(
                    "result.html",
                    ok=False,
                    reason="This table already has an active order.",
                    table_id=table_id,
                    order_id=order_id,
                    status=STATUS_DENIED
                )

            # 3) 库存检查：不足则 DENIED（原来是 CANCELED）
            ok, reason = self._reserve_inventory(items)
            if not ok:
                self.store.update_status(order_id, STATUS_DENIED, reason)
                return render_template(
                    "result.html",
                    ok=False,
                    reason=reason,
                    table_id=table_id,
                    order_id=order_id,
                    status=STATUS_DENIED
                )

            # 4) 通过：CREATED -> CONFIRMED，并 set_active + publish
            self.store.update_status(order_id, STATUS_CONFIRMED, "OK")
            self.store.set_active(table_id, order_id)

            self._publish_order_new_once(order_id)

            return render_template(
                "result.html",
                ok=True,
                reason="OK",
                table_id=table_id,
                order_id=order_id,
                status=STATUS_CONFIRMED
            )


        @app.route("/status/<order_id>", methods=["GET"])
        def status_page(order_id):
            od = self.store.get_order(order_id)
            if not od:
                return "Order not found", 404

            items_pre = "\n".join([f"- {x['name']} x {x['amount']}" for x in od["items"]]) or "(none)"
            return render_template(
                "status.html",
                table_id=od["table_id"],
                order_id=order_id,
                status=od["status"],
                items_pre=items_pre,
                special_request=od["special_request"] or "(none)"
            )

        @app.route("/api/status/<table_id>", methods=["GET"])
        def api_status(table_id):
            active_id = self.store.get_active_order_id(table_id)
            if not active_id:
                return {"ok": False, "reason": "no active order"}

            od = self.store.get_order(active_id)
            if not od:
                return {"ok": False, "reason": "missing order"}

            return {"ok": True, "order_id": active_id, "status": od["status"]}

    def _store_order(self, table_id, order_id, items, special, status):
        data = {
            "order_id": order_id,
            "table_id": table_id,
            "timestamp": now_iso(),
            "items": items,
            "special_request": special,
            "status": status,
            "reason": ""
        }
        self.store.put_order(order_id, data)

        rospy.loginfo("[web_order_server] order created: table=%s order_id=%s status=%s reason=%s",
              table_id, order_id, status, "submitted")

        if status in {STATUS_SERVED, STATUS_DENIED}:
            self.store.clear_active_if_matches(table_id, order_id)


    def _reserve_inventory(self, items):
        req = ReserveRequest()
        for x in items:
            it = ReserveItem()
            it.name = x["name"]
            it.amount = int(x["amount"])
            req.items.append(it)

        try:
            resp = self.reserve_srv(req)
            if resp.ok:
                return True, "OK"
            return False, resp.reason or "inventory not enough"
        except Exception as e:
            return False, f"inventory service error: {e}"

    def _publish_order_new_once(self, order_id):
        if not self.store.mark_published_once(order_id):
            rospy.logwarn("[web_order_server] duplicated publish blocked for order_id=%s", order_id)
            return

        od = self.store.get_order(order_id)
        if not od:
            return

        msg = Order()
        msg.order_id = od["order_id"]
        msg.table_id = od["table_id"]
        msg.timestamp = rospy.Time.now()
        msg.special_request = od["special_request"]
        msg.status = od["status"]

        for x in od["items"]:
            oi = OrderItem()
            oi.name = x["name"]
            oi.amount = int(x["amount"])
            msg.items.append(oi)

        self.order_new_pub.publish(msg)
        rospy.loginfo("[web_order_server] published /orders/new: table=%s order_id=%s status=%s",
              msg.table_id, msg.order_id, msg.status)

    def _publish_status_tick(self, _evt):
        """
        每秒发布当前活跃订单的状态到 /orders/status
        """
        # 收集当前活跃订单（防止遍历时锁太久：先拷贝 key 再逐个取）
        with self.store.lock:
            active_order_ids = list(self.store.table_active.values())

        for oid in active_order_ids:
            od = self.store.get_order(oid)
            if not od:
                continue

            # 只对活跃态做状态心跳
            if od["status"] not in ACTIVE_STATUSES:
                continue

            msg = Order()
            msg.order_id = od["order_id"]
            msg.table_id = od["table_id"]
            msg.timestamp = rospy.Time.now()
            msg.special_request = od["special_request"]
            msg.status = od["status"]

            for x in od["items"]:
                oi = OrderItem()
                oi.name = x["name"]
                oi.amount = int(x["amount"])
                msg.items.append(oi)

            self.order_status_pub.publish(msg)

    def _on_status_msg(self, msg: Order):
        """
        监听 /orders/status，把 topic 上的状态缓存下来，并同步更新本地订单状态（若存在）。
        """
        table_id = str(msg.table_id)
        order_id = str(msg.order_id)
        st = (msg.status or "").upper()

        # 1) 记录“该桌最新状态”（用于 submit 时判断是否挡单）
        self.store.set_table_status(table_id, st)

        # 2) 若本地 store 中存在该 order_id，也同步更新（便于 status 页面显示）
        od = self.store.get_order(order_id)
        if od:
            self.store.update_status(order_id, st)

            # served/canceled 后释放 active
            if st in {STATUS_SERVED, STATUS_DENIED}:
                self.store.clear_active_if_matches(table_id, order_id)

    def run(self):
        self.app.run(host=self.host, port=self.port, debug=False, use_reloader=False)


if __name__ == "__main__":
    rospy.init_node("web_order_server", anonymous=False)
    WebOrderServer().run()
