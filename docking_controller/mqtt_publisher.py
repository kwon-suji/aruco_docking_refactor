import json
import paho.mqtt.client as mqtt

MQTT_BROKER = "10.10.30.62"
MQTT_PORT = 31883
CMD_VEL_TOPIC = "/ros2/cmd_vel"

mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_BROKER, MQTT_PORT)
mqtt_client.loop_start()

def publish_cmd_vel(linear=0.0, angular=0.0, req_id="auto"):
    msg = {
        "req_id": req_id,
        "linear": linear,
        "angular": angular
    }
    mqtt_client.publish(CMD_VEL_TOPIC, json.dumps(msg))

def stop():
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
