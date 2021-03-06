import sensor, image, time, network, usocket, sys, json

SSID ='OPENMV_AP'    # Network SSID
KEY  ='1234567890'    # Network key (must be 10 chars)
HOST = ''           # Use first available interface
PORT = 8080         # Arbitrary non-privileged port

green_threshold   = (   0,   80,  -70,   -10,   -0,   30)

# Reset sensor
sensor.reset()
# Set sensor settings
sensor.set_contrast(1)
sensor.set_brightness(1)
sensor.set_saturation(1)
sensor.set_gainceiling(16)
sensor.set_framesize(sensor.QQVGA)
sensor.set_pixformat(sensor.GRAYSCALE)

# Init wlan module in AP mode.
wlan = network.WINC(mode=network.WINC.MODE_AP)
wlan.start_ap(SSID, key=KEY, security=wlan.WEP, channel=2)

# You can block waiting for client to connect
#print(wlan.wait_for_sta(10000))

def recv(s):
    print ('Waiting for connections..')
    client, addr = s.accept()
    # set client socket timeout to 2s
    client.settimeout(2.0)
    print ('Connected to ' + addr[0] + ':' + str(addr[1]))

    #while True:
    # Read request from client
    data = client.recv(1024)
    if data:
        # Should parse client request here
        print('recive:',data.decode()) #打印接收到的数据

    client.send("HTTP/1.1 200 OK\r\n" \
                "Server: OpenMV\r\n" \
                "Content-Type: application/json\r\n" \
                "Cache-Control: no-cache\r\n" \
                "Pragma: no-cache\r\n\r\n")
    client.send(data.decode())
    client.close()
    #s.close()


def response(s):
    print ('Waiting for connections..')
    client, addr = s.accept()
    # set client socket timeout to 2s
    client.settimeout(2.0)
    print ('Connected to ' + addr[0] + ':' + str(addr[1]))

    # Read request from client
    data = client.recv(1024)
    # Should parse client request here
    print('recive:',data.decode()) #打印接收到的数据

    # Send multipart header
    client.send("HTTP/1.1 200 OK\r\n" \
                "Server: OpenMV\r\n" \
                "Content-Type: application/json\r\n" \
                "Cache-Control: no-cache\r\n" \
                "Pragma: no-cache\r\n\r\n")

    # FPS clock
    clock = time.clock()

    # Start streaming images
    # NOTE: Disable IDE preview to increase streaming FPS.

    img = sensor.snapshot()
    blobs = img.find_blobs([green_threshold])
    if blobs:
        for b in blobs:
            img.draw_rectangle(b[0:4]) # rect
            img.draw_cross(b[5], b[6]) # cx, cy

    client.send(json.dumps(blobs))
    client.close()



while (True):
   # recv(s)
   # Create server socket
   s = usocket.socket(usocket.AF_INET, usocket.SOCK_STREAM)
   # Bind and listen
   s.bind([HOST, PORT])
   s.listen(5)
   try:
        # Set server socket timeout
        # NOTE: Due to a WINC FW bug, the server socket must be closed and reopened if
        # the client disconnects. Use a timeout here to close and re-create the socket.
        s.settimeout(3)
        recv(s)
   except OSError as e:
        s.close()
        print("socket error: ", e)
        #sys.print_exception(e)

