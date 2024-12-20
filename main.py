from machine import Pin, I2C
import network
import time
import bme280
import ssl
from umqtt.simple import MQTTClient

# Wi-Fi and MQTT credentials
SSID = "Shafin_B14_2.4"
WIFI_PASSWORD = "22446688"
BROKER = "d227fdb4a0bf4d4dbe239e1a7d4bb1c5.s1.eu.hivemq.cloud"
PORT = 8883  # SSL/TLS port
MQTT_USERNAME = "shafinsaleem"
MQTT_PASSWORD = "Oulu2023"
TOPIC_TEMP = "sensors/bme280/temp"
TOPIC_PRESSURE = "sensors/bme280/pressure"
TOPIC_HUMIDITY = "sensors/bme280/humidity"
TOPIC_CONTROL = "picow/control"  # New control topic

# Initialize I2C and BME280
i2c = I2C(0, scl=Pin(21), sda=Pin(20))
bme = bme280.BME280(i2c=i2c)

# Setup LED
led_pin = Pin("LED", Pin.OUT)  # Built-in LED on Pico W

# Connect to Wi-Fi
def connect_to_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(SSID, WIFI_PASSWORD)
    print("Connecting to Wi-Fi...")
    connection_timeout = 10
    while connection_timeout > 0:
        if wlan.status() == 3:  # Connected
            break
        connection_timeout -= 1
        print("Waiting for Wi-Fi connection...")
        time.sleep(1)

    if wlan.status() != 3:
        raise RuntimeError('[ERROR] Failed to establish a network connection')
    else:
        print('[INFO] CONNECTED!')
        network_info = wlan.ifconfig()
        print('[INFO] IP address:', network_info[0])
    return wlan

# Connect to MQTT broker
def connect_to_mqtt():
    # Configure SSL context
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    context.verify_mode = ssl.CERT_NONE  # Disable server certificate validation

    client = MQTTClient(client_id="pico",
                        server=BROKER,
                        port=PORT,
                        user=MQTT_USERNAME,
                        password=MQTT_PASSWORD,
                        ssl=context,
                        keepalive=300)
    try:
        client.connect(clean_session=False)
        print("Connected to MQTT broker.")
    except Exception as e:
        print(f"Failed to connect to MQTT broker: {e}")
        raise
    return client

# Reconnect to MQTT broker
def reconnect_mqtt(client):
    try:
        client.connect()
        print("Reconnected to MQTT broker.")
    except Exception as e:
        print(f"Reconnection failed: {e}")
        time.sleep(1)  # Retry after a delay

# Publish data to MQTT topic
def publish(client, topic, message):
    try:
        client.publish(topic, message, retain=False, qos=0)
        print(f"[INFO][PUB] Published {message} to {topic}")
    except OSError as e:
        print(f"Failed to publish to {topic}: {e}")
        reconnect_mqtt(client)
    except Exception as e:
        print(f"Unexpected error while publishing: {e}")

# MQTT message callback for control topic
def on_message(topic, msg):
    print(f"[INFO][MQTT] Received message: {msg} on topic: {topic}")
    if msg == b"ON":
        led_pin.on()  # Turn on the LED
        print("[INFO][CONTROL] LED turned ON")
    elif msg == b"OFF":
        led_pin.off()  # Turn off the LED
        print("[INFO][CONTROL] LED turned OFF")

# Main loop to publish sensor data and handle control signals
def publish_sensor_data(client):
    try:
        print("Checking MQTT connection...")
        client.ping()  # Ensure the connection is alive
        temp = bme.values[0]  # Temperature from BME280
        pressure = bme.values[1]  # Pressure from BME280
        humidity = bme.values[2]  # Humidity from BME280
        publish(client, TOPIC_TEMP, str(temp))
        publish(client, TOPIC_PRESSURE, str(pressure))
        publish(client, TOPIC_HUMIDITY, str(humidity))
    except Exception as e:
        print(f"Unexpected error in sensor data publishing: {e}")

# Main program
try:
    wlan = connect_to_wifi()
    mqtt_client = connect_to_mqtt()

    # Set up MQTT callback and subscribe to control topic
    mqtt_client.set_callback(on_message)
    mqtt_client.subscribe(TOPIC_CONTROL)
    print(f"[INFO] Subscribed to topic: {TOPIC_CONTROL}")

    while True:
        # Publish sensor data
        publish_sensor_data(mqtt_client)

        # Check for control messages
        mqtt_client.check_msg()

        time.sleep(15)  # Publish every 15 seconds
except KeyboardInterrupt:
    print("Program stopped.")