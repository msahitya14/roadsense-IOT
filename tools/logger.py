#!/usr/bin/env python3
"""Log every RoadSense MQTT message to logs/<session>.jsonl (and print a short line).

    python tools/logger.py                                   # local broker, creds from broker/credentials.txt
    python tools/logger.py --host xxx.hivemq.cloud --tls --user dashboard --password ...   # cloud broker
"""
import argparse, json, time
import mqtt_common as mc
from csvlog import SessionCSV, stamp_now


def main():
    ap = mc.add_args(argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter))
    ap.add_argument("--topic", default=mc.BASE + "/#")
    ap.add_argument("--no-csv", action="store_true", help="JSONL only, no logs/pose|pothole|link-*.csv")
    a = ap.parse_args()

    (mc.ROOT / "logs").mkdir(exist_ok=True)
    path = mc.ROOT / "logs" / time.strftime("session-%Y%m%d-%H%M%S.jsonl")
    fh = path.open("a", buffering=1)
    print(f"logging to {path}", flush=True)
    sess = None if a.no_csv else SessionCSV(mc.ROOT / "logs", path.stem.replace("session-", ""))

    def on_connect(cl, _u, _f, rc, _p=None):
        print("connected" if rc == 0 else f"connect failed: {rc}", flush=True)
        cl.subscribe(a.topic, qos=1)

    def on_message(_cl, _u, msg):
        rx = stamp_now()  # this computer's system time
        txt = msg.payload.decode(errors="replace")
        try:
            data = json.loads(txt)
        except ValueError:
            data = txt
        fh.write(json.dumps({"rx": time.time(), "topic": msg.topic, "data": data}) + "\n")
        kind = msg.topic.rsplit("/", 1)[-1]
        if sess and isinstance(data, dict) and kind == "pose":
            sess.pose(data, rx)
        elif sess and isinstance(data, dict) and kind == "pothole":
            sess.pothole(data, rx)
        elif sess and kind in ("status", "info"):
            sess.link(kind, data, rx)
        if kind == "pose" and isinstance(data, dict):
            if rx[0] - on_message.last_print < 1.0:  # poses can arrive at 50+/s: print 1/s
                return
            on_message.last_print = rx[0]
            print(f"pose  x={data['x_m']:7.2f} y={data['y_m']:7.2f} hdg={data['hdg_deg']:6.1f} "
                  f"d={data.get('dist_m', 0):6.2f} {data['surf']:7s} vib={data.get('vib_g', 0):.3f} "
                  f"label={data.get('label')}", flush=True)
        else:
            print(f"{kind:7s} {txt}", flush=True)

    on_message.last_print = 0.0
    cl = mc.connect(a, f"logger-{int(time.time())}")
    cl.on_connect = on_connect
    cl.on_message = on_message
    try:
        cl.loop_forever()
    except KeyboardInterrupt:
        print(f"\nsaved {path}")


if __name__ == "__main__":
    main()
