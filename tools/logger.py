#!/usr/bin/env python3
"""Log every RoadSense MQTT message to logs/<session>.jsonl (and print a short line).

    python tools/logger.py                                   # local broker, creds from broker/credentials.txt
    python tools/logger.py --host xxx.hivemq.cloud --tls --user dashboard --password ...   # cloud broker
"""
import argparse, json, time
import mqtt_common as mc


def main():
    ap = mc.add_args(argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter))
    ap.add_argument("--topic", default=mc.BASE + "/#")
    a = ap.parse_args()

    (mc.ROOT / "logs").mkdir(exist_ok=True)
    path = mc.ROOT / "logs" / time.strftime("session-%Y%m%d-%H%M%S.jsonl")
    fh = path.open("a", buffering=1)
    print(f"logging to {path}", flush=True)

    def on_connect(cl, _u, _f, rc, _p=None):
        print("connected" if rc == 0 else f"connect failed: {rc}", flush=True)
        cl.subscribe(a.topic, qos=1)

    def on_message(_cl, _u, msg):
        txt = msg.payload.decode(errors="replace")
        try:
            data = json.loads(txt)
        except ValueError:
            data = txt
        fh.write(json.dumps({"rx": time.time(), "topic": msg.topic, "data": data}) + "\n")
        kind = msg.topic.rsplit("/", 1)[-1]
        if kind == "pose" and isinstance(data, dict):
            print(f"pose  x={data['x_m']:7.2f} y={data['y_m']:7.2f} hdg={data['hdg_deg']:6.1f} "
                  f"d={data.get('dist_m', 0):6.2f} {data['surf']:7s} vib={data.get('vib_g', 0):.3f} "
                  f"label={data.get('label')}", flush=True)
        else:
            print(f"{kind:7s} {txt}", flush=True)

    cl = mc.connect(a, f"logger-{int(time.time())}")
    cl.on_connect = on_connect
    cl.on_message = on_message
    try:
        cl.loop_forever()
    except KeyboardInterrupt:
        print(f"\nsaved {path}")


if __name__ == "__main__":
    main()
