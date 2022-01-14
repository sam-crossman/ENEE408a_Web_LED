# import socket library

try:
    import usocket as socket
except:
    import socket

# import Wifi networking libraries
from machine import Pin
import network
# added from merge
from urequests import *
from time import *

# turn off vendor OS debug messages
import esp
esp.osdebug(None)

# garbage collector to manage memory
import gc 
gc.collect()

# network credentials
ssid = "Sams_iPhone"
password = "nysmd23iwjlp"

# set esp32 as a Wifi station
station = network.WLAN(network.STA_IF)
# activate station
station.active(True)
# connect to Wifi source/router
station.connect(ssid, password)

# ensure that Wifi is connected
while station.isconnected() == False:
    
    pass


print("Network Connection successful")
# prints ip address (esp?), netmask, gateway, DNS

print(station.ifconfig())
# GPIO2 pin of esp32
led = Pin(2, Pin.OUT)
