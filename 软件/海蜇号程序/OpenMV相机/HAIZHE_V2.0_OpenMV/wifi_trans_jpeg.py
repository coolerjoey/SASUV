import sensor, image, time, network, usocket, sys,pyb
from pyb import LED
from pyb import Pin, ExtInt

red_led   = LED(1)
green_led = LED(2)
blue_led  = LED(3)
ir_led    = LED(4)

SSID ='OPENMV_AP'    # Network SSID
KEY  ='1234567890'    # Network key (must be 10 chars)
HOST = ''           # Use first available interface
PORT = 8080         # Arbitrary non-privileged port


#初始化串口
uart_baudrate = 115200
uart = pyb.UART(3, uart_baudrate, timeout_char = 1000)

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

buf_recv_flag =0
#定义外部中断服务函数,注意：回调函数中不能有任何的内存申请！！
def callback(line):
    global buf_recv_flag
    #print("line =", line)
    buf_recv_flag = 1

#初始化外部中断
#https://docs.singtown.com/micropython/zh/latest/openmvcam/library/pyb.ExtInt.html#pyb-extint
extint = pyb.ExtInt('P9', pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, callback)


def response(s):
    print ('Waiting for connections..')
    client, addr = s.accept()
    # set client socket timeout to 2s
    client.settimeout(2.0)
    print ('Connected to ' + addr[0] + ':' + str(addr[1]))

    # Read request from client
    data = client.recv(1024)
    # Should parse client request here

    # Send multipart header
    client.send("HTTP/1.1 200 OK\r\n" \
                "Server: OpenMV\r\n" \
                "Content-Type: image/jpeg\r\n\r\n")

    # FPS clock
    clock = time.clock()

    # Start streaming images
    # NOTE: Disable IDE preview to increase streaming FPS.


    frame = sensor.snapshot()
    cframe = frame.compressed(quality=35)
    client.send(cframe)
    client.close()

#mav包格式:fe seq mid data ff
#mid=25:openmv工作模式设定    data:0-水面水下都不记录，1-水面记录，水下不记录，2-水面不记录，水下记录，3-都记录
MAV_MID_PARA = 6
para_OpenMV_CAM = 10
openmv_record=0 #默认记录种类为0
#openmv记录种类
openmv_none                     =0  #manual不实时传输，manual不记录，auto不记录
openmv_manual_rt                =1  #manual实时传输，manual不记录，auto不记录
openmv_manual_rec               =2  #manual不实时传输，manual记录，auto不记录
openmv_manual_rt_rec            =3  #manual实时传输，manual记录，auto不记录
openmv_auto_rec                 =4  #manual不实时传输，manual不记录，auto记录
openmv_manual_rt_auto_rec       =5  #manual实时传输，manual不记录，auto记录
openmv_manual_rec_auto_rec		=6  #manual不实时传输，manual记录，auto记录
openmv_manual_rt_rec_auto_rec	=7  #manual实时传输，manual记录，auto记录

global manual_mode_snap #水面视频发送使能标志位
manual_mode_snap = 0
global auto_mode_record #水下记录使能标志位
auto_mode_record = 0

def MAV_Parse(mavmsg_bytes):#海蜇号mav信号解析
    global openmv_record
    # bytes转字符串
    mavmsg_str = str(mavmsg_bytes, 'utf-8')
    #find()用于str查找元素位置,index()用于查找列表元素位置，
    #两者区别在于，遇到没有的元素时：find会返回-1，index会报错
    if(mavmsg_str.find('fe')==-1):return#查找'fe'包头,没有找到则返回
    if(mavmsg_str.find('ff')==-1):return#查找'ff'包头
    mavmsg = mavmsg_str.split(' ')
    #index第一个元素下标从0开始
    mavmsg_head_pos = mavmsg.index('fe')
    mavmsg_tail_pos = mavmsg.index('ff')
    mavmsg_seq = int(mavmsg[mavmsg_head_pos+1])
    mavmsg_mid = int(mavmsg[mavmsg_head_pos+2])
    mavmsg_para_list = int(mavmsg[mavmsg_head_pos+3])
    mavmsg_data = int(mavmsg[mavmsg_head_pos+4])
    openmv_record = mavmsg_data
    if mavmsg_mid==MAV_MID_PARA:
        #print("open_mv set \r\n")
        if mavmsg_data==0:
            green_led.off()
            red_led.off()
        elif mavmsg_data==1:
            green_led.off()
            red_led.on()
        elif mavmsg_data==2:
            green_led.on()
            red_led.off()
            print("auto record!\r\n")
            manual_mode_snap == 0;
        elif mavmsg_data==3:
            green_led.on()
            red_led.on()
            print("record all!\r\n")
            manual_mode_snap == 1;
    #uart.write(mavmsg_head_pos)
    #uart.write(mavmsg_mid)


while (True):


    if buf_recv_flag==1:
        buf_recv_flag = 0
        #if uart.any():
        buf = uart.readline()
        print(buf)
#        #uart.readinto(buf)
        MAV_Parse(buf)

    if openmv_record==openmv_none:
        pass
        #print("record none!\r\n")
    elif openmv_record==openmv_manual_rt:
        print("manual record!\r\n")
        try:
            # Create server socket
            s = usocket.socket(usocket.AF_INET, usocket.SOCK_STREAM)
            # Bind and listen
            s.bind([HOST, PORT])
            s.listen(5)

            # Set server socket timeout
            # NOTE: Due to a WINC FW bug, the server socket must be closed and reopened if
            # the client disconnects. Use a timeout here to close and re-create the socket.
            s.settimeout(3)
            response(s)
        except OSError as e:
            s.close()
            print("socket error: ", e)
            #sys.print_exception(e)




