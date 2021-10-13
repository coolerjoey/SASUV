import sensor, image, time, network, usocket, sys,pyb,os
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


###############################
#全局变量定义
###############################
buf_recv_flag =0    #接收中断标志
task_run=0  #默认不处于auto运行阶段
task_total_num=0    #记录总任务数
record_path=''  #记录路径
#data = openmv记录种类
openmv_none                     =0  #manual不实时传输，manual不记录，auto不记录
openmv_manual_rt                =1  #manual实时传输，manual不记录，auto不记录
openmv_manual_rec               =2  #manual不实时传输，manual记录，auto不记录
openmv_manual_rt_rec            =3  #manual实时传输，manual记录，auto不记录
openmv_auto_rec                 =4  #manual不实时传输，manual不记录，auto记录
openmv_manual_rt_auto_rec       =5  #manual实时传输，manual不记录，auto记录
openmv_manual_rec_auto_rec		=6  #manual不实时传输，manual记录，auto记录
openmv_manual_rt_rec_auto_rec	=7  #manual实时传输，manual记录，auto记录
record_data=[]   #建立一个空列表，每一项存储一个视频文件夹名字
video_name=[]   #video_name是二级列表，每一项存储多个同一日期的视频名
img_reader = '' #要播放的视频文件句柄
video_play_finish_flag = 0  #视频播放结束标志
openmv_record = openmv_auto_rec #openmv记录默认记录种类

###############################
#初始化
###############################
#~读取配置参数(上次记录日期)


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


#定义外部中断服务函数,注意：回调函数中不能有任何的内存申请！！
def callback(line):
    global buf_recv_flag
    #print("line =", line)
    buf_recv_flag = 1

#初始化外部中断
#https://docs.singtown.com/micropython/zh/latest/openmvcam/library/pyb.ExtInt.html#pyb-extint
extint = pyb.ExtInt('P9', pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, callback)


###############################
#函数定义
###############################

#wifi实时发送视频流
def start_streaming(s):
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
                "Content-Type: multipart/x-mixed-replace;boundary=openmv\r\n" \
                "Cache-Control: no-cache\r\n" \
                "Pragma: no-cache\r\n\r\n")
    # FPS clock
    clock = time.clock()

    # Start streaming images
    # NOTE: Disable IDE preview to increase streaming FPS.
    while (True):
        clock.tick() # Track elapsed milliseconds between snapshots().
        frame = sensor.snapshot()
        cframe = frame.compressed(quality=35)
        header = "\r\n--openmv\r\n" \
                 "Content-Type: image/jpeg\r\n"\
                 "Content-Length:"+str(cframe.size())+"\r\n\r\n"
        client.send(header)
        client.send(cframe)
        print(clock.fps())

#WiFi发送记录视频流
def record_streaming(s):
    global video_play_finish_flag
    global img_reader
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
                "Content-Type: multipart/x-mixed-replace;boundary=openmv\r\n" \
                "Cache-Control: no-cache\r\n" \
                "Pragma: no-cache\r\n\r\n")
    # FPS clock
    clock = time.clock()

    # Start streaming images
    # NOTE: Disable IDE preview to increase streaming FPS.
    frame = ''
    while (frame != None):
        clock.tick() # Track elapsed milliseconds between snapshots().
        frame = img_reader.next_frame(copy_to_fb=True, loop=False)
        #frame = sensor.snapshot()
        if frame != None:
            cframe = frame.compressed(quality=35)
            header = "\r\n--openmv\r\n" \
                     "Content-Type: image/jpeg\r\n"\
                     "Content-Length:"+str(cframe.size())+"\r\n\r\n"
            client.send(header)
            client.send(cframe)
            print(clock.fps())
    print('play finish')
    video_play_finish_flag = 1

#读取sd卡视频列表(记录日期，文件名，记录时间，记录时长)
def get_video_list():
    global record_data
    global video_name
    data_num = 0
    root_files= os.listdir('/') #得到文件夹下的所有文件名称
    for file in root_files: #遍历文件夹
        '''
        判断是否是记录的视频文件夹
        正规操作应该是os.path.isdir(file)，但是报错？
        '''
        if '-' in file: #判断是否是视频记录文件夹
            record_data.append(file)
            video_name.append([])   #添加二级列表
            sub_files = os.listdir("/"+file)
            for sub_file in sub_files:
                video_name[data_num].append(sub_file)#读取文件名
                #~读取文件大小
                #~读取文件创建时间
                print('record %d :' % data_num)
                print(record_data[data_num])
                print(video_name[data_num])
            data_num = data_num+1
            #print(sub_files)

#视频列表组合成json格式字符串并发送
def send_video_list_json():
    '''video_list示例
    obj ={
        "video_list":
        [
            {
                "record_data":2019-10-31,
                "video":
                [
                    {
                        "video_time":"16:26:20",
                        "video_name":"2019-10-31-task_1"
                    },
                    {
                        "video_time":"17:00:08",
                        "video_name":"2019-10-31-task_2"
                    },
                ]
            },
            {
                "record_data":2019-11-01,
                "video":
                [
                    {
                        "video_time":"12:00:01",
                        "video_name":"2019-11-01-task_1"
                    },
                    {
                        "video_time":"14:00:01",
                        "video_name":"2019-11-01-task_2"
                    }
                ]
            }
        ]
    }
    '''
    data_num = 0
    video_list = ''
    video_one_day = ''
    record_one_day = ''
    data_length = len(record_data)#总记录天数
    data_count = 0
    for data in record_data:
        data_count = data_count+1   #记录天数加1
        video_length = len(video_name[data_num])#一天视频记录的条数
        video_count = 0
        for video in video_name[data_num]:  #处理一天内记录的视频
            video_count = video_count+1 #一天内的视频个数累加
            if video_count<video_length:    #还没有到最后一个视频
                video_one_day = video_one_day+("{\"video_name\": %s},") % (video)    #构造一个video数组中的一个对象
            else:   #每天的最后一条末尾不能带有','
                video_one_day = video_one_day+("{\"video_name\": %s}") % (video)    #构造一个video数组中的一个对象
        if data_count<data_length:  #还没有到记录的最后一天
            record_one_day = record_one_day+("{\"record_data\": %s, \"video\": [%s]}, ") % (data,video_one_day)
        else:   #最后一天的末尾不能带有','
            record_one_day = record_one_day+("{\"record_data\": %s, \"video\": [%s]}") % (data,video_one_day)
        data_num = data_num+1
        video_one_day = ''
    video_list =("{\"video_list\": [%s]}") % record_one_day
    print(video_list)
    uart.write(video_list+'\r\n')   #视频列表发送给海蜇号

#播放视频
def play_video(video):
    global video_name
    global record_data
    global img_reader
    global video_play_finish_flag
    video_num=0 #视频个数
    for data_list in range(len(video_name)):
        for video_list in range(len(video_name[data_list])):
            video_num = video_num+1
            if video_num==video:
                video_paly_flag = 1
                video_play_name = video_name[data_list][video_list] #要播放视频的文件名
                video_play_path = record_data[data_list]    #要播放视频的文件夹
                #print(video_play_path)
                #print(video_play_name)
                video_path = ("/%s/%s") % (video_play_path,video_play_name) #要播放视频的完整路径
                print(video_path)
                img_reader = image.ImageReader(video_path)
                while (video_play_finish_flag==0):
                    # Create server socket
                    s = usocket.socket(usocket.AF_INET, usocket.SOCK_STREAM)
                    try:
                        # Bind and listen
                        s.bind([HOST, PORT])
                        s.listen(5)
                        # Set server socket timeout
                        # NOTE: Due to a WINC FW bug, the server socket must be closed and reopened if
                        # the client disconnects. Use a timeout here to close and re-create the socket.
                        s.settimeout(3)
                        record_streaming(s)
                    except OSError as e:
                        s.close()
                        print("socket error: ", e)
                        #sys.print_exception(e)

#海蜇号mav信号解析
def MAV_Parse(mavmsg_bytes):
    global openmv_record
    global task_run
    global record_start
    global record_finish
    global task_total_num
    global record_data
    global record_path
    global video_name
    global img_reader

    '''MAV变量定义
    '''
    #mav包格式:fe seq mid data ff
    MAV_MID_PARA = 6    #参数设定包ID
    MAV_MID_TASK_RUN = 14   #运行包ID
    MAV_MID_VIDEO_LIST = 25 #视频列表获取包ID
    MAV_MID_VIDEO_PLAY = 26 #播放视频包ID
    MAV_MID_VIDEO_PLAY_ON = 27  #视频开始播放包ID
    MAV_MID_TIME = 30   #日期包ID
    para_OpenMV_CAM = 10    #参数设定包中的openmv设置包sub-id

    # bytes转字符串
    mavmsg_str = str(mavmsg_bytes, 'utf-8')
    #find()用于str查找元素位置,index()用于查找列表元素位置，
    #两者区别在于，遇到没有的元素时：find会返回-1，index会报错
    if(mavmsg_str.find('fe')==-1):return#查找'fe'包头,没有找到则返回
    if(mavmsg_str.find('ff')==-1):return#查找'ff'包尾
    mavmsg = mavmsg_str.split(' ')
    #index第一个元素下标从0开始
    mavmsg_head_pos = mavmsg.index('fe')
    mavmsg_tail_pos = mavmsg.index('ff')
    mavmsg_seq = int(mavmsg[mavmsg_head_pos+1])
    mavmsg_mid = int(mavmsg[mavmsg_head_pos+2])

    if mavmsg_mid==MAV_MID_PARA:    #接收到参数设定
        mavmsg_para_list = int(mavmsg[mavmsg_head_pos+3])
        mavmsg_data = int(mavmsg[mavmsg_head_pos+4])
        openmv_record = mavmsg_data
        #print("open_mv set \r\n")
        if openmv_record==openmv_none:
            print("openmv_none \r\n")
            red_led.off()
            green_led.off()
            blue_led.off()
        elif mavmsg_data==openmv_manual_rt:
            print("openmv_manual_rt \r\n")
            red_led.on()
            green_led.off()
            blue_led.off()
        elif mavmsg_data==openmv_manual_rec:
            red_led.off()
            green_led.on()
            blue_led.off()
            print("openmv_manual_rec \r\n")
        elif mavmsg_data==openmv_manual_rt_rec:
            red_led.on()
            green_led.on()
            blue_led.off()
            print("openmv_manual_rt_rec \r\n")
        elif openmv_record==openmv_auto_rec:
            print("openmv_auto_rec \r\n")
            green_led.off()
            red_led.off()
            blue_led.on()
        elif openmv_record==openmv_manual_rt_auto_rec:
            print("openmv_manual_rt_auto_rec \r\n")
            green_led.on()
            red_led.off()
            blue_led.on()
        elif openmv_record==openmv_manual_rec_auto_rec:
            print("openmv_manual_rec_auto_rec \r\n")
            green_led.off()
            red_led.on()
            blue_led.on()
        elif openmv_record==openmv_manual_rt_rec_auto_rec:
            print("openmv_manual_rt_rec_auto_rec \r\n")
            green_led.on()
            red_led.on()
            blue_led.on()
        #uart.write(mavmsg_head_pos)
        #uart.write(mavmsg_mid)
    elif mavmsg_mid==MAV_MID_TASK_RUN:  #接收到运行包
        mavmsg_data = int(mavmsg[mavmsg_head_pos+3])
        task_total_num = mavmsg_data
        if task_total_num!=0: task_run=1
        else: task_run=0
        if task_run==1: #开始运行
            record_start=1
            record_finish = 0
            print("start run \r\n")
        elif   task_run==0: #停止运行
            print("stop run!\r\n")
    elif mavmsg_mid==MAV_MID_VIDEO_LIST:  #接收到视频列表获取包
        get_video_list()    #获取视频列表
        send_video_list_json()  #组合成json格式并发送
    elif mavmsg_mid==MAV_MID_VIDEO_PLAY: #接收到视频播放包
        mavmsg_data = int(mavmsg[mavmsg_head_pos+3])
        video_play = mavmsg_data;  #要播放的视频
        mav_msg_send = ("fe 0 %d 1 ff\r\n") % MAV_MID_VIDEO_PLAY_ON
        uart.write(mav_msg_send)   #向海蜇号回传已开始播放视频
        play_video(1)    #video_play
    elif mavmsg_mid==MAV_MID_TIME: #接收到日期包
        record_data = mavmsg[mavmsg_head_pos+3]
        print('当前日期:'+record_data)
        #~如果当前时间和上次记录时间不同，创建一个新的视频文件夹
        record_path = '/'+record_data
        try:
            os.mkdir(record_path)
        except OSError as e:
            print("os error: ", e)


###############################
#测试代码部分
###############################
#get_video_list()
#play_video(1)

###############################
#主程序
###############################
record_start=0 #一次记录开始标志
record_finish=1 #一次记录完成标志
while (True):
    #串口接收解析
    if buf_recv_flag==1:
        buf_recv_flag = 0
        #if uart.any():
        buf = uart.readline()
        #print(buf)
        MAV_Parse(buf)

    if openmv_record==openmv_none:
        pass
    elif openmv_record==openmv_manual_rt:

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
            start_streaming(s)
        except OSError as e:
            s.close()
            print("socket error: ", e)
            #sys.print_exception(e)
    elif openmv_record==openmv_manual_rec:pass
    elif openmv_record==openmv_auto_rec:
        #print("auto record \r\n")
        if record_finish != 1:
            if  record_start==1:
                print("start record \r\n")
                record_start=0
                #按照时间和任务数来记录视频文件
                global filename
                filename = ("%s/%s_task-%d.bin") % (record_path,record_data,task_total_num)
                img_writer = image.ImageWriter(filename)
            if task_run==1:  #开始记录
                img = sensor.snapshot()
                # Modify the image if you feel like here...
                img_writer.add_frame(img)
            elif task_run==0:
                print("stop record! \r\n")
                record_finish = 1    #停止记录
                img_writer.close()



