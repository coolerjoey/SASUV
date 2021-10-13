import sensor, image, time, network, usocket, sys,pyb,os
from pyb import LED
from pyb import Pin, ExtInt

red_led   = LED(1)
green_led = LED(2)
blue_led  = LED(3)
ir_led    = LED(4)

SSID ='Haizhe01_cam'    # Network SSID
KEY  ='1234567890'    # Network key (must be 10 chars)
HOST = ''           # Use first available interface
PORT = 8080         # Arbitrary non-privileged port

##############################
#led灯状态
#红色慢闪(1hz):不回传不记录，等待连接
#红灯快闪:auto模式正在记录
#蓝灯慢闪(3hz)：实时回传，等待连接
#蓝灯快闪：正在实时回传
#绿灯快闪：正在回放视频
##############################

###############################
#全局变量定义
###############################
#视频变量
record_data=[]   #建立一个空列表，每一项存储一个视频文件夹名字
video_name=[]   #video_name是二级列表，每一项存储多个同一日期的视频名
video_list=[]   #存储所有视频文件的路径
img_reader = '' #要播放的视频文件句柄
img_writer = '' #要写入的视频文件句柄
video_play_finish_flag = 0  #视频播放结束标志
record_path=''  #记录路径
rt_trans_flag=0   #实时播放标志
#任务变量
record_start=0 #一次记录开始标志
record_finish=1 #一次记录完成标志
buf_recv_flag =0    #接收中断标志
task_run=0  #默认不处于auto运行阶段
task_total_num=0    #一天内的总任务数
task_total_time=0   #一次auto任务总时长
run_time = ''   #auto运行开始时间
CtrlMode = 0    #海蜇号所处模式
mode_set_status = 0     #1-处于模式配置
#openmv记录种类
openmv_none                     =0  #manual不实时传输，manual不记录，auto不记录
openmv_manual_rt                =1  #manual实时传输，manual不记录，auto不记录
openmv_manual_rec               =2  #manual不实时传输，manual记录，auto不记录
openmv_manual_rt_rec            =3  #manual实时传输，manual记录，auto不记录
openmv_auto_rec                 =4  #manual不实时传输，manual不记录，auto记录
openmv_manual_rt_auto_rec       =5  #manual实时传输，manual不记录，auto记录
openmv_manual_rec_auto_rec		=6  #manual不实时传输，manual记录，auto记录
openmv_manual_rt_rec_auto_rec	=7  #manual实时传输，manual记录，auto记录
#记录模式变量
auto_mode = 1
openmv_record = openmv_manual_rt #openmv记录默认记录种类
#回传信息变量
mav_sequence = 0

###############################
#初始化
###############################
#~读取配置参数(上次记录日期)

'''
#定义外部中断服务函数,注意：回调函数中不能有任何的内存申请！！
def callback(line):
    global buf_recv_flag
    print("line =", line)
    buf_recv_flag = 1

#初始化外部中断
#https://docs.singtown.com/micropython/zh/latest/openmvcam/library/pyb.ExtInt.html#pyb-extint
extint = pyb.ExtInt('P9', pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, callback)
'''

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
sensor.set_pixformat(sensor.RGB565)
#sensor.set_hmirror(True)    #水平方向翻转
#sensor.set_vflip(True)  #垂直方向翻转

# Init wlan module in AP mode.
wlan = network.WINC(mode=network.WINC.MODE_AP)
wlan.start_ap(SSID, key=KEY, security=wlan.WEP, channel=2)

# You can block waiting for client to connect
#print(wlan.wait_for_sta(10000))





###############################
#函数定义
###############################

#串口接收解析
def uart_parse():
    s = ''
    c = ''
    if uart.any():
        buf = uart.readline()
        print(buf)
        if buf:
            MAV_Parse(s,c,buf)

#WiFi发送记录视频流
def record_streaming(s):
    global video_play_finish_flag
    global img_reader
    print ('Waiting play..')
    client, addr = s.accept()
    # set client socket timeout to 2s
    client.settimeout(2.0)
    print ('Connected to ' + addr[0] + ':' + str(addr[1]))

    # Read request from client
    #data = client.recv(1024)
    # Should parse client request here

    # Send multipart header
    client.send("HTTP/1.1 200 OK\r\n" \
                "Server: Haizhe01_cam\r\n" \
                "Content-Type: multipart/x-mixed-replace;boundary=Haizhe01_cam\r\n" \
                "Cache-Control: no-cache\r\n" \
                "Pragma: no-cache\r\n\r\n")
    # FPS clock
    clock = time.clock()
    red_led.off()
    green_led.off()
    blue_led.on()
    # Start streaming images
    # NOTE: Disable IDE preview to increase streaming FPS.
    frame = ''
    while (frame != None):
        led_toggle(green_led)
        clock.tick() # Track elapsed milliseconds between snapshots().
        frame = img_reader.next_frame(copy_to_fb=True, loop=False)
        #frame = sensor.snapshot()
        if frame != None:
            cframe = frame.compressed(quality=30)
            header = "\r\n--Haizhe01_cam\r\n" \
                     "Content-Type: image/jpeg\r\n"\
                     "Content-Length:"+str(cframe.size())+"\r\n\r\n"
            client.send(header)
            client.send(cframe)
            print(clock.fps())
    client.close()
    s.close()
    print('play finish')
    video_play_finish_flag = 1

#播放视频
def play_video(s,c,video):
    global video_name
    global record_data
    global img_reader
    global video_play_finish_flag
    global mav_sequence
    video_num=0 #视频个数
    video_play_finish_flag = 0
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
                print('ok')
                c.send("HTTP/1.1 200 OK\r\n" \
                        "Server: Haizhe01_cam\r\n" \
                        "Content-Type: application/json\r\n" \
                        "Cache-Control: no-cache\r\n" \
                        "Pragma: no-cache\r\n\r\n")
                mav_sequence = mav_sequence+1
                mav_msg_send = ("fe %d %d 1 ff \r\n") % (mav_sequence,27)
                c.send(mav_msg_send)
                c.close()
                s.close()
                while (video_play_finish_flag==0):
                    # Create server socket
                    s_video = usocket.socket(usocket.AF_INET, usocket.SOCK_STREAM)
                    try:
                        # Bind and listen
                        s_video.bind([HOST, PORT])
                        s_video.listen(5)
                        # Set server socket timeout
                        # NOTE: Due to a WINC FW bug, the server socket must be closed and reopened if
                        # the client disconnects. Use a timeout here to close and re-create the socket.
                        s_video.settimeout(1)
                        record_streaming(s_video)
                    except OSError as e:
                        red_led.off()
                        green_led.on()
                        blue_led.off()
                        s_video.close()
                        print("socket error: ", e)
                        #sys.print_exception(e)

#读取sd卡视频列表(记录日期，文件名，记录时间，记录时长)
def get_video_list():
    global record_data
    global video_name
    global video_list
    record_data = []
    video_name = []
    video_list = []
    data_num = 0
    root_files= os.listdir('/') #得到文件夹下的所有文件名称
    for video_file in root_files: #遍历文件夹
        '''
        判断是否是记录的视频文件夹
        正规操作应该是os.path.isdir(file)，但是报错？
        '''
        if '2019' in video_file: #判断是否是视频记录文件夹
            record_data.append(video_file)
            video_name.append([])   #添加二级列表
            sub_files = os.listdir("/"+video_file)
            for sub_file in sub_files:
                video_name[data_num].append(sub_file)#读取文件名
                video_path = ("/%s/%s") % (video_file,sub_file)
                video_list.append(video_path)
                #~读取文件大小(正规操作是os.path.getctime(filePath)，但是报错？)
                #~读取文件创建时间
            print('record %d :' % data_num)
            print(record_data[data_num])
            print(video_name[data_num])
            print(video_list)
            data_num = data_num+1
            #print(sub_files)

#视频列表组合成json格式字符串并发送
def send_video_list_json(c):
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
    #uart.write(video_list+'\r\n')   #视频列表发送给海蜇号
    # Send multipart header
    c.send("HTTP/1.1 200 OK\r\n" \
                "Server: Haizhe01_cam\r\n" \
                "Content-Type: application/json\r\n" \
                "Cache-Control: no-cache\r\n" \
                "Pragma: no-cache\r\n\r\n")
    c.send(video_list)
    #c.close()

#海蜇号mav信号解析
def MAV_Parse(s,c,mavmsg):
    global openmv_record
    global task_run
    global record_start
    global record_finish
    global task_total_num
    global task_total_time
    global record_data
    global record_path
    global video_name
    global img_reader
    global run_time
    global CtrlMode
    global mode_set_status
    global video_list
    global rt_trans_flag
    string = ''
    '''MAV变量定义
    '''
    #mav包格式:fe seq mid data ff
    MAV_MID_CtrlMode = 3    #模式包ID
    MAV_MID_PARA = 6    #参数设定包ID
    MAV_MID_TASK_RUN = 14   #运行包ID
    MAV_MID_VIDEO_LIST = 25 #视频列表获取包ID
    MAV_MID_VIDEO_PLAY = 26 #播放视频包ID
    MAV_MID_VIDEO_PLAY_ON = 27  #视频开始播放包ID
    MAV_MID_VIDEO_REMOVE = 28   #删除视频文件包ID
    MAV_MID_VIDEO_RT_TRANS = 29 #实时回传视频包ID
    MAV_MID_TIME = 30   #日期包ID
    para_OpenMV_CAM = 10    #参数设定包中的openmv设置包sub-id
    #bytes转字符串
    #print(mavmsg)
    if type(mavmsg)!=type(string):
        mavmsg_str = str(mavmsg, 'utf-8')
    else: mavmsg_str = mavmsg
    #find()用于str查找元素位置,index()用于查找列表元素位置，
    #两者区别在于，遇到没有的元素时：find会返回-1，index会报错
    if(mavmsg_str.find('fe')==-1):return#查找'fe'包头,没有找到则返回
    if(mavmsg_str.find('ff')==-1):return#查找'ff'包尾
    mavmsg = mavmsg_str.split(' ')  #以' '分割文本，返回列表
    #index第一个元素下标从0开始
    mavmsg_head_pos = mavmsg.index('fe')
    mavmsg_tail_pos = mavmsg.index('ff')
    mavmsg_seq = int(mavmsg[mavmsg_head_pos+1])
    mavmsg_mid = int(mavmsg[mavmsg_head_pos+2])
    print(mavmsg_mid)
    if mavmsg_mid==MAV_MID_TASK_RUN:  #接收到运行包
        mavmsg_data = int(mavmsg[mavmsg_head_pos+3])
        run_time = mavmsg[mavmsg_head_pos+4]#获取开始时间
        task_total_time = mavmsg[mavmsg_head_pos+5]#获取总运行时长
        task_total_num = mavmsg_data
        #print(mavmsg_data)
        #print(run_time)
        #print(task_total_time)
        if task_total_num!=0: #开始运行
            task_run=1
            record_start=1
            record_finish = 0
            print("start run \r\n")
        else: #停止运行
            task_run=0
            print("stop run!\r\n")
    elif mavmsg_mid==MAV_MID_VIDEO_LIST:  #接收到视频列表获取包
        print('get video list')
        get_video_list()    #获取视频列表
        send_video_list_json(c)  #组合成json格式并发送
    elif mavmsg_mid==MAV_MID_VIDEO_PLAY: #接收到视频播放包
        mavmsg_data = int(mavmsg[mavmsg_head_pos+3])
        video_play = mavmsg_data;  #要播放的视频
        #s.close()
        print('play video:',video_play)
        play_video(s,c,video_play)    #video_play
    elif mavmsg_mid==MAV_MID_VIDEO_RT_TRANS: #接收到视频实时回传包
        mavmsg_data = int(mavmsg[mavmsg_head_pos+3])
        rt_trans_flag = mavmsg_data
        if rt_trans_flag:print('实时播放')
    elif mavmsg_mid==MAV_MID_TIME: #接收到日期包
        record_data = mavmsg[mavmsg_head_pos+3]
        print('当前日期:'+record_data)
        #~如果当前时间和上次记录时间不同，创建一个新的视频文件夹
        record_path = '/'+record_data
        try:
            os.mkdir(record_path)
        except OSError as e:
            print("os error: ", e)
    elif mavmsg_mid==MAV_MID_CtrlMode: #接收到模式包
        mavmsg_data = int(mavmsg[mavmsg_head_pos+3])
        CtrlMode = mavmsg_data
    elif mavmsg_mid==MAV_MID_VIDEO_REMOVE: #接收到删除包
        #删除全部视频
        mavmsg_data = mavmsg[mavmsg_head_pos+3]
        if mavmsg_data=='1':
            print('删除全部视频')
            root_files= os.listdir('/')
            for video_file in root_files:
                if '2019' in video_file:
                    sub_files = os.listdir("/"+video_file)
                    for video in sub_files:
                        video_path = '/'+video_file+'/'+video
                        print(video_path)
                        os.remove(video_path)
                    os.rmdir(video_file)
        #删除部分视频
        else:
            data_head_pos = mavmsg_str.find('[')
            data_tail_pos = mavmsg_str.find(']')
            mavmsg_data = mavmsg_str[data_head_pos+1:data_tail_pos]#提取视频删除列表
            video_remove_list = eval(mavmsg_data)#字符串转列表
            #print(mavmsg_str)
            for i in range(len(video_remove_list)):
                if video_remove_list[i]==1:
                    #print('要删除视频：',i)
                    #print(video_list[i])
                    os.remove(video_list[i])
            #~返回删除成功消息
            #返回删除后的视频列表
            get_video_list()
            send_video_list_json(c)

#记录模式检测
def record_mode(s):
    global openmv_record
    global record_start
    global record_finish
    global filename
    global task_total_time
    global img_writer
    global CtrlMode
    global mode_set_status
    mode_set_status = 0

    #print("auto record \r\n")
    while record_finish!=1 and mode_set_status!=1:
        uart_parse()
        if  record_start==1:
            print("start record \r\n")
            record_start=0
            #按照时间和任务数来记录视频文件
            #文件名按照"记录日期-开始记录时间-总时长-当天第几个任务"来构造
            filename = ("%s/%s-%s-%s-%d.bin") % (record_path,record_data,run_time,task_total_time,task_total_num)
            img_writer = image.ImageWriter(filename)
            print(filename)
        if task_run==1:  #开始记录
            led_toggle(red_led)
            img = sensor.snapshot()
            # Modify the image if you feel like here...
            img_writer.add_frame(img)
        elif task_run==0:
            print("stop record! \r\n")
            record_finish = 1    #停止记录
            img_writer.close()

#等待app客户端连接
def wait_connect(s):
    print ('Waiting for connections..')
    client, addr = s.accept()
    # set client socket timeout to 2s
    client.settimeout(2.0)
    print ('Connected to ' + addr[0] + ':' + str(addr[1]))

    # Read request from client
    data = client.recv(1024)
    # Should parse client request here
    if data:
        buf = data.decode()
        #print('recive:',buf) #打印接收到的数据
        if(buf.find('ff')==-1):return   #防止误接收，不检测fe是因为误接收中也会有fe字符串
        head = buf.index('fe')
        #tail = buf.index('ff')
        buf = buf[head:]
        print('recive:',buf) #打印接收到的数据
        red_led.on()
        green_led.off()
        blue_led.off()
        MAV_Parse(s,client,buf)
    #client.close()

#led灯取反
x=0
def led_toggle(led):
    global x
    if x==0:
        x=1
        led.off()
    elif x==1:
        x=0
        led.on()
    if led==red_led:
        green_led.off()
        blue_led.off()
    elif led==green_led:
        red_led.off()
        blue_led.off()
    else:
        green_led.off()
        red_led.off()

###############################
#测试代码部分
###############################
#get_video_list()
#play_video(1)

###############################
#主程序
###############################
while (True):
    # Create server socket
    s = usocket.socket(usocket.AF_INET, usocket.SOCK_STREAM)
    try:
        # Bind and listen
        s.bind([HOST, PORT])
        s.listen(5)

        # Set server socket timeout
        # NOTE: Due to a WINC FW bug, the server socket must be closed and reopened if
        # the client disconnects. Use a timeout here to close and re-create the socket.
        s.settimeout(1)

        uart_parse()
        record_mode(s)
        wait_connect(s)
    except OSError as e:
        s.close()
        #print("socket error: ", e)
        led_toggle(red_led)
        #sys.print_exception(e)





