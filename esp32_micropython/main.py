import uart

cloud_api="https://api.thingspeak.com/channels/1600828/feeds.json?results=2"

def web_page(color):
    # get led state
    if led.value() == 1:
        gpio_state = "ON"
    else:
        gpio_state = "OFF"
    # generate html for the webpage
    html = """<html><head> <title>ESP32 Web Server</title> <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="icon" href="data:,"> <style>html{font-family: Helvetica; display:inline-block; margin: 0px auto; text-align: center;}
    h1{color: #0F3376; padding: 2vh;}p{font-size: 1.5rem;}.button{display: inline-block; background-color: #e7bd3b; border: none; 
    border-radius: 4px; color: white; padding: 16px 40px; text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}
    .button2{background-color: #4286f4;}</style></head><body> <h1>ESP Web Server</h1> 
    <p>GPIO state: <strong>""" + gpio_state + """</strong></p><p><a href="/?led=on"><button class="button">OFF</button></a></p> 
    <p>color: <strong>""" + color + """</strong></p><p><a href="/?led=on"><button class="button">ON</button></a></p>
    </body></html>"""
    return html

# create stream tcp socket 
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# bind with port 80
s.bind(('', 80))
# enable server to accept max of 5 connections
s.listen(5)

# is there fork() in python to service socket connections and control light?
# I SUPPOSE THE FLAG WAS TO SEE IF THE CLOUD API CHANGED...
# need to put above stuff back in loop???
# flag in if statement and whether to start new socket connection???
prev_on = 0
changed = 0
color = 'none'
while True:
    # accept incoming connection and save socket object, client address
    conn, addr = s.accept()
    print('Got a connection from %s' % str(addr))
    #request = conn.recv(1024)
    #request = str(request)

    # thingspeak value stuff
    on = get(cloud_api)
    on = on.text
    on = on.split('</br>')[0]
    on = on.split('"')[-2]
    print(on)
    
    if (on == '0' and prev_on != 0):
        led.value(0)
        print("LED OFF")
        prev_on = 0
    
    elif (on == '1' and prev_on != 1):
        led.value(1)
        #uart.uart_run()
        print("LED ON")
        prev_on = 1
     
    elif (on == '1' and prev_on == 1):
        color = on.split('"')[-1]

        print("color: " + color)
        uart.uart_run(color)

    response = web_page(color)
    conn.send('HTTP/1.1 200 OK\n')
    conn.send('Content-Type: text/html\n')
    conn.send('Connection: close\n\n')
    conn.sendall(response)
    conn.close()