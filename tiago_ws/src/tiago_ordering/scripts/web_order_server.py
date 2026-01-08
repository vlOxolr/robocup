#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import uuid
from threading import Lock
from datetime import datetime

from flask import Flask, request, redirect, url_for, render_template_string

from tiago_ordering.msg import Order, OrderItem
from tiago_ordering.srv import Reserve, ReserveRequest
from tiago_ordering.msg import ReserveItem

STATUS_CREATED   = "CREATED"
STATUS_CONFIRMED = "CONFIRMED"
STATUS_CANCELED  = "CANCELED"
STATUS_SERVING   = "SERVING"
STATUS_SERVED    = "SERVED"

ACTIVE_STATUSES = {STATUS_CREATED, STATUS_CONFIRMED, STATUS_SERVING}

MENU_HTML = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>Table {{table_id}} - Menu</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 18px; max-width: 720px; }
    .card { border: 1px solid #ddd; border-radius: 8px; padding: 12px; margin: 10px 0; }
    .row { display: flex; gap: 10px; align-items: center; }
    input[type=number] { width: 72px; padding: 4px; }
    textarea { width: 100%; height: 72px; }
    button { padding: 10px 14px; border: none; border-radius: 8px; cursor: pointer; }
    .btn { background: #222; color: white; }
    .muted { color: #666; font-size: 12px; }
  </style>
</head>
<body>
  <h2>Table {{table_id}} - Menu</h2>
  <p class="muted">Submit once. Re-scan QR will show status if an active order exists.</p>

  <form method="POST" action="{{ url_for('submit_order', table_id=table_id) }}">
    {% for name in menu_items %}
      <div class="card">
        <div class="row">
          <div style="flex:1;"><b>{{name}}</b></div>
          <div>
            Qty:
            <input type="number" min="0" max="99" name="qty_{{name}}" value="0"/>
          </div>
        </div>
      </div>
    {% endfor %}

    <div class="card">
      <div><b>Special request</b></div>
      <textarea name="special_request" placeholder="e.g., no ice, less sugar..."></textarea>
    </div>

    <button class="btn" type="submit">Submit Order</button>
  </form>
</body>
</html>
"""

RESULT_HTML = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>Order Result</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 18px; max-width: 720px; }
    .ok { color: #0a7; }
    .bad { color: #c33; }
    .card { border: 1px solid #ddd; border-radius: 8px; padding: 12px; margin: 10px 0; }
    .muted { color: #666; font-size: 12px; }
  </style>
</head>
<body>
  <h2>Order Result</h2>

  {% if ok %}
    <p class="ok"><b>Success.</b> Order received.</p>
  {% else %}
    <p class="bad"><b>Failed.</b> {{reason}}</p>
  {% endif %}

  <div class="card">
    <div><b>Table:</b> {{table_id}}</div>
    <div><b>Order ID:</b> {{order_id}}</div>
    <div><b>Status:</b> {{status}}</div>
    <div class="muted">Re-scan QR to view status page.</div>
  </div>

  <a href="{{ url_for('table_entry', table_id=table_id) }}">Back</a>
</body>
</html>
"""

STATUS_HTML = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>Order Status</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 18px; max-width: 720px; }
    .card { border: 1px solid #ddd; border-radius: 8px; padding: 12px; margin: 10px 0; }
    .muted { color: #666; font-size: 12px; }
    pre { background: #f6f6f6; padding: 10px; border-radius: 8px; }
  </style>
</head>
<body>
  <h2>Order Status</h2>
  <div class="card">
    <div><b>Table:</b> {{table_id}}</div>
    <div><b>Order ID:</b> {{order_id}}</div>
    <div><b>Status:</b> <span id="st">{{status}}</span></div>
    <div class="muted">Auto refresh every 2 seconds.</div>
  </div>

  <div class="card">
    <b>Items</b>
    <pre id="items">{{items_pre}}</pre>
    <b>Special request</b>
    <pre>{{special_request}}</pre>
  </div>

<script>
async function refresh() {
  const r = await fetch("{{ url_for('api_status', table_id=table_id) }}");
  if (!r.ok) return;
  const j = await r.json();
  if (!j.ok) return;
  document.getElementById("st").innerText = j.status;
}
setInterval(refresh, 2000);
</script>
</body>
</html>
"""

class OrderStore:
    def __init__(self):
        self.lock = Lock()
        self.table_active = {}   # table_id -> order_id
        self.orders = {}         # order_id -> data
        self.published = set()   # published order_ids

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

    def update_status(self, order_id, status):
        with self.lock:
            if order_id in self.orders:
                self.orders[order_id]["status"] = status


def make_order_id_4():
    return uuid.uuid4().hex[:4].upper()

def now_iso():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


class WebOrderServer:
    def __init__(self):
        self.app = Flask(__name__)
        self.store = OrderStore()

        self.menu_items = [str(x) for x in rospy.get_param("~menu_items", ["cola", "water"])]
        self.host = rospy.get_param("~host", "0.0.0.0")
        self.port = int(rospy.get_param("~port", 8080))

        self.order_pub = rospy.Publisher("/orders", Order, queue_size=10)

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

            return render_template_string(MENU_HTML, table_id=table_id, menu_items=self.menu_items)

        @app.route("/submit/<table_id>", methods=["POST"])
        def submit_order(table_id):
            order_id = make_order_id_4()

            items = []
            for name in self.menu_items:
                qty = int(request.form.get(f"qty_{name}", "0") or "0")
                if qty > 0:
                    items.append({"name": name, "amount": qty})

            special = (request.form.get("special_request", "") or "").strip()

            if len(items) == 0:
                status = STATUS_CANCELED
                reason = "No items selected."
                self._store_order(table_id, order_id, items, special, status)
                return render_template_string(RESULT_HTML, ok=False, reason=reason,
                                              table_id=table_id, order_id=order_id, status=status)

            ok, reason = self._reserve_inventory(items)
            if not ok:
                status = STATUS_CANCELED
                self._store_order(table_id, order_id, items, special, status)
                return render_template_string(RESULT_HTML, ok=False, reason=reason,
                                              table_id=table_id, order_id=order_id, status=status)

            status = STATUS_CREATED
            self._store_order(table_id, order_id, items, special, status)
            self.store.set_active(table_id, order_id)

            self._publish_order_once(order_id)
            return render_template_string(RESULT_HTML, ok=True, reason="OK",
                                          table_id=table_id, order_id=order_id, status=status)

        @app.route("/status/<order_id>", methods=["GET"])
        def status_page(order_id):
            od = self.store.get_order(order_id)
            if not od:
                return "Order not found", 404

            items_pre = "\n".join([f"- {x['name']} x {x['amount']}" for x in od["items"]]) or "(none)"
            return render_template_string(
                STATUS_HTML,
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
            "status": status
        }
        self.store.put_order(order_id, data)

        if status in {STATUS_CANCELED, STATUS_SERVED}:
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

    def _publish_order_once(self, order_id):
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

        self.order_pub.publish(msg)
        rospy.loginfo("[web_order_server] published /orders: table=%s order_id=%s status=%s",
                      msg.table_id, msg.order_id, msg.status)

    def run(self):
        self.app.run(host=self.host, port=self.port, debug=False, use_reloader=False)


if __name__ == "__main__":
    rospy.init_node("web_order_server", anonymous=False)
    WebOrderServer().run()
