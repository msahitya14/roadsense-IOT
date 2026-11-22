"""Shared MQTT connection options for the RoadSense tools.

Local broker (default): reads users/passwords from broker/credentials.txt.
Cloud broker:           --host xxx.s1.eu.hivemq.cloud --port 8883 --tls --user ... --password ...
Environment variables RS_MQTT_HOST / _PORT / _USER / _PASS / _TLS=1 work too.
"""
import argparse, os, pathlib
import paho.mqtt.client as mqtt

ROOT = pathlib.Path(__file__).resolve().parent.parent
BASE = os.environ.get("RS_TOPIC_BASE", "rs/T24")


def creds():
    out = {}
    f = ROOT / "broker" / "credentials.txt"
    if f.exists():
        for line in f.read_text().splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                out[k.strip()] = v.strip()
    return out


def add_args(ap: argparse.ArgumentParser, role: str = "dashboard"):
    """role: 'dashboard' (logger, reads everything) or 'device' (simulator, publishes as rs01)."""
    c = creds()
    user_key, pass_key = ("DEVICE_USER", "DEVICE_PASS") if role == "device" else ("DASH_USER", "DASH_PASS")
    env = os.environ.get
    ap.add_argument("--host", default=env("RS_MQTT_HOST", "localhost"))
    ap.add_argument("--port", type=int, default=int(env("RS_MQTT_PORT", "0")) or None,
                    help="default 1883, or 8883 with --tls")
    ap.add_argument("--tls", action="store_true", default=env("RS_MQTT_TLS", "0") == "1")
    ap.add_argument("--user", default=env("RS_MQTT_USER", c.get(user_key, "rs01" if role == "device" else "dashboard")))
    ap.add_argument("--password", default=env("RS_MQTT_PASS", c.get(pass_key, "")))
    return ap


def connect(a, client_id: str, will=None) -> mqtt.Client:
    cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    cl.username_pw_set(a.user, a.password)
    if a.tls:
        cl.tls_set()  # system CA store (HiveMQ Cloud / Let's Encrypt is trusted)
    if will:
        cl.will_set(*will)
    port = a.port or (8883 if a.tls else 1883)
    print(f"connecting to {'mqtts' if a.tls else 'mqtt'}://{a.host}:{port} as {a.user}")
    cl.connect(a.host, port, 30)
    return cl
