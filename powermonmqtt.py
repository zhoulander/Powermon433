# MIT License
#
# Copyright (c) 2020 Brian Zhou
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

import json
import paho.mqtt.client as paho
import serial


PWRMONMQTT_NAME           = 'powermon433_mqtt'
PWRMONMQTT_VERSION        = '0.0.1'
PWRMONMQTT_MANUFACTURER   = 'Blue Line Innovations'
PWRMONMQTT_MODEL_LIST     = ['BLI-28000']
PWRMONMQTT_MQTT_TOPIC     = 'powermon433'
PWRMONMQTT_MODEL_1        = 0

# homeassistant mqtt device discovery entity information payload
def ha_device(device_model, device_sn):
    device = dict()
    device['name']         = "{0}_{1}".format(PWRMONMQTT_MANUFACTURER.lower().replace(' ',''), device_sn)
    device['model']        = PWRMONMQTT_MODEL_LIST[device_model]
    device['manufacturer'] = PWRMONMQTT_MANUFACTURER
    device['sw_version']   = "{0} {1}".format(PWRMONMQTT_NAME, PWRMONMQTT_VERSION)
    device['identifiers']  = ['{0}_{1}'.format(device['model'].lower().replace(' ',''), device_sn)]
    return device

# homeassistant mqtt device discovery configuration payload
def ha_device_config_payload(sensor_name, device_sn, sensor_unit, sensor_class, sensor_device):
    config_value = ['name', 'state_topic','unit_of_measurement','value_template','device', 'device_class', 'json_attributes_topic', 'unique_id']
    config_payload = dict()
    config_payload[config_value[0]] = sensor_name
    config_payload[config_value[1]] = "{0}/sensor/{1}".format(PWRMONMQTT_MQTT_TOPIC, device_sn)
    config_payload[config_value[2]] = sensor_unit
    config_payload[config_value[3]] = "{{{{ value_json.{0} }}}}".format(sensor_name)
    config_payload[config_value[4]] = sensor_device
    if sensor_class is not None:
        config_payload[config_value[5]] = sensor_class
    config_payload[config_value[6]] = "{0}/sensor/{1}".format(PWRMONMQTT_MQTT_TOPIC, device_sn)
    config_payload[config_value[7]] = "{0}_{1}_{2}".format(device_sn, sensor_name, PWRMONMQTT_NAME)
    return config_payload


NUMBER_OF_SENSORS = 4
sensor_data    = [None]*NUMBER_OF_SENSORS
sensor_units   = ["ms", "Wh", "W", "degC"]
sensor_names   = ['PrintDelta_ms', 'Total_Energy_Wh', 'Power_W', 'Temp_C']
sensor_classes = [None, None, 'power', 'temperature']


# put your own config here
broker="192.168.1.193"
port=1883
ser = serial.Serial('/dev/ttyACM0',38400)
sensorid = '0x0000'

def on_publish(client,userdata,result):             #create function for callback
    print("data published \n")
    pass

client1= paho.Client("powermon433")                           #create client object
client1.on_publish = on_publish                          #assign function to callback
client1.connect(broker,port)                                 #establish connection


while True:

# fix fake json from arduino sketch
    read_serial=ser.readline().decode().strip().replace("'",'"')
    payload = json.loads(read_serial)
    try:
        sensorid = payload['sensor']        
        device_sn = sensorid
        for i in range(NUMBER_OF_SENSORS):
            advertise_device = ha_device(0, device_sn)
            config_payload = ha_device_config_payload(sensor_names[i].replace(' ','_'), device_sn, sensor_units[i], sensor_classes[i], advertise_device)
            topic = "homeassistant/sensor/{0}/{0}_{1}/config".format(device_sn, sensor_names[i])
            info = client1.publish(topic, json.dumps(config_payload), retain=True)
            print("{0} / {1}".format(topic, info))
        

    except:
        pass
    try:
        topic = "{0}/sensor/{1}".format(PWRMONMQTT_MQTT_TOPIC, device_sn)
        info = client1.publish(topic, json.dumps(payload), retain=False)
        print("{0} / {1} : {2}".format(topic, info, json.dumps(payload)))
    except:
        pass
