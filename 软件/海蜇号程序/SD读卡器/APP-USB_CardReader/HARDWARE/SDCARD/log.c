#include "log.h"
#include "uart.h"
#include "global_para.h"
#include "exfuns.h"
#include "cJSON_Utils.h"
#include "esp8266.h"
#include "malloc.h"

FATFS  FileSys; 			//file system object
FIL    File;					//File object
FRESULT  res; 	//File function return code (FRESULT)
UINT _bw ; 				//file number count 

#define sd_root "0:/"		//根目录
#define DIR_MAX_NUM 256	//根目录下最大的文件夹个数
#define FILE_MAX_NUM	256	//子目录下最大文件个数
char dir_name[DIR_MAX_NUM];		//存储根目录下文件夹的名称
char *file_in_one_dir_name[FILE_MAX_NUM];	//存储一个子文件夹下所有文件名称
char *file_all_name[DIR_MAX_NUM];		//存储所有子文件夹下所有文件名称
u8 file_num[DIR_MAX_NUM];			//存储每个子文件夹下所有文件个数

FRESULT scan_files (char* path){        /* Start node to be scanned (***also used as work area***) */
	FRESULT res;
	DIR dir;
	UINT i;
	static FILINFO fno;
	u8 num=0;
	
	res = f_opendir(&dir, path);                       /* Open the directory */
	if (res == FR_OK) {
			for (;;) {
					res = f_readdir(&dir, &fno);                   /* Read a directory item */
					if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
					if (fno.fattrib & AM_DIR) {                    /* It is a directory */
							dir_name[num] = fno.fname[0];						//保存根目录下的所有文件夹名
							i = strlen(path);
							sprintf(&path[i], "/%s", fno.fname);
							res = scan_files(path);                    /* Enter the directory */
							if (res != FR_OK) break;
							path[i] = 0;
					} else {                                       /* It is a file. */
							printf("%s/%s\n", path, fno.fname);
							file_in_one_dir_name[num] = fno.fname;						//保存当前子文件夹下的一个文件名
//							file_num[num] = ;	//保存当前子文件夹下的文件个数
							num++;
					}
			}
			f_closedir(&dir);
	}
	return res;
}

//sd卡记录函数
void logging(){
	if(!vp.task_run){	//还没有开始水下运行
		f_close(&File);	//关闭先前打开的文件
		return;
	}
	if(sys_flag.log_creat==true){	//水下运行一次创建一个log文件
		sys_flag.log_creat = false;
		char dir_name[32];
		char file_name[32];
		
		//创建文件夹，以year-month-day命名
		sprintf(dir_name,"0:/%4d%02d%02d",vp.sys_time.year,vp.sys_time.month,vp.sys_time.day);
		res = f_mkdir(dir_name);	
		if(res != FR_OK && res != FR_EXIST){	//文件夹创建不成功则退出
			printf("[error] Creat Folder Failed: %d \r\n",res);
			return;
		}
		if(res == FR_EXIST)
			printf("[ok] Folder %s already existed \r\n",dir_name);
		else printf("[ok] Creat Folder %s \r\n",dir_name);
		
		//创建文件，以year-month-day-hour-min-sec.log命名
		sprintf(file_name,"%s/%4d%02d%02d_%02d%02d%02d.txt",
											dir_name,
											vp.sys_time.year,vp.sys_time.month,vp.sys_time.day,
											vp.sys_time.hour,vp.sys_time.minute,vp.sys_time.second);
		res = f_open(&File,file_name,FA_OPEN_EXISTING);
		if(res==FR_OK){		//如果已存在此文件，则覆盖(先删除再创建)
			printf("[ok] File %s already exist,rewrite! \r\n",file_name);
			f_unlink(file_name);
		}
		res = f_open(&File,file_name,FA_OPEN_ALWAYS|FA_READ|FA_WRITE);
		if(res != FR_OK){	//文件创建不成功则退出
			printf("[error] Creat file Failed: %d \r\n",res);
			f_close(&File);
			return;
		}
		else printf("[ok] Creat File %s \r\n",file_name);
		
		//在文件头部写入任务列表，形如
		/*
		task:list	type	time	value
					1		1			10.0	5.00
					2		3			10.0	5.00
					...(写到任务为空的子任务)
		*/
//		f_lseek(&File,100);  //离文件起始100字节开始写入(每次上电从头写入)
		char *task_list = mymalloc(SRAMIN,1024);
		//任务列表整合
		//sprintf函数返回写入的字符总数，不包括字符串追加在字符串末尾的空字符。如果失败，则返回一个负数。
		int list_length = sprintf(task_list,"task: \t list \t type \t time \t value \r\n");
		for(u8 i=0;Task_List[i].task_list!=0;i++){	//写到任务为空的子任务
			list_length = sprintf(task_list,"%s \t %d \t %d \t %.1f \t %.2f \r\n",
							task_list,Task_List[i].task_num,Task_List[i].task_list,
							Task_List[i].task_beat,Task_List[i].task_val);
		}
		if(list_length<0){	//若任务列表数组初始化错误，退出。可能原因是maloc分配的字节不够
			printf("[error] Write task_list fails: %d \r\n",list_length);
			myfree(SRAMIN,task_list);
			return;
		}
		res = f_write(&File,task_list,list_length,&_bw);//往文件里面写数据.写100字节，bw实际返回写入字节数(用于检测文件已满)sizeof(task_list)/sizeof(char)
		printf("[ok] write task_list \r\n");
		myfree(SRAMIN,task_list);
		
		//写入保存的数据种类
		char *file_hearder = mymalloc(SRAMIN,1024);
		int type_length = sprintf(file_hearder,"acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,eular_x,eular_y,eular_z,pressure,depth,temperature,voltage,roll_in,pitch_in,yaw_in,throttle_in,motor1,motor2,motor3,motor4 \r\n");
		if(type_length<0){
			printf("[error] Write data type fails: %d \r\n",type_length);
			myfree(SRAMIN,file_hearder);
			return;
		}
		res = f_write(&File,file_hearder,type_length,&_bw);
		printf("[ok] write date type \r\n");
		f_sync(&File);	//保存文件记录，防止系统崩溃数据丢失
		f_close(&File);
//		File = (void*)0;	//文件关闭后，需要将文件指针指向空，这样做会防止出现游离指针，而对整个工程造成不必要的麻烦；如：fp = NULL;
		myfree(SRAMIN,file_hearder);
	}
	return;
	char *logging_data;
	sprintf(logging_data," \r\n\
					%.2f,%.2f,%.2f,\
					%.2f,%.2f,%.2f,\
					%.2f,%.2f,%.2f,\
					%.2f,%.2f,%.2f,\
					%.2f,%.2f,%.2f,%.2f,\
					%.2f,%.2f,%.2f,%.2f,\
					%.2f,%.2f,%.2f,%.2f,\
					\r\n",
					//传感器数据
					Sensor_latest.acc.x,Sensor_latest.acc.x,Sensor_latest.acc.x,
					Sensor_latest.gyro.x,Sensor_latest.gyro.x,Sensor_latest.gyro.x,
					Sensor_latest.mag.x,Sensor_latest.mag.x,Sensor_latest.mag.x,
					Sensor_latest.eluar.x,Sensor_latest.eluar.x,Sensor_latest.eluar.x,
					Sensor_latest.pressure,Sensor_latest.depth,Sensor_latest.temp,Sensor_latest.power,
					//电机数据
					roll_in,pitch_in,yaw_in,throttle_in,
					motor_out[0],motor_out[1],motor_out[2],motor_out[3]
	);
//	res = f_write(&File,logging_data,sizeof(logging_data)/sizeof(char),&_bw);
//	printf("file write result: %d \r\n",res);
//	f_sync(&File);
}

/*
json输出格式：
	{
		"sd_size":sd_size,			//sd容量
		"sd_used":sd_used,			//sd已使用容量
		"sd_size":sd_available,	//sd可用容量
		"record":[{
			"record_data":"20200225";
			"record_lists":[{
				"name":"20200225-095800.log",
				"duration":duration},{
				"name":"20200225-105800.log",
				"duration":duration}...
			]
		},{
			"record_data":"20200226";
			"record_lists":[{
				"name":"20200226-095800.log",
				"duration":duration},{
				"name":"20200226-105800.log",
				"duration":duration}...
			]
		}],
	}
*/

//https://www.jianshu.com/p/3b7c8aac2469
//sd卡返回记录列表
void send_loglist(){
	u8 record_dir_num=0;	//记录文件夹个数
//	u16 record_file_num=0;	//记录文件个数
	u16 sd_size=0;//sd容量
	u16 sd_used=0;//sd已使用容量
	u16 sd_available=0;//sd可用容量
	u16 i =0;
	memset(dir_name,0,sizeof(char)*DIR_MAX_NUM);
	
//	char record_dir[record_dir_num];	//存储所有文件夹名称	
//	char *record_file[record_dir_num];//存储指向每个文件夹下所有文件名的指针

	//遍历根目录下的文件夹，读取每个文件夹下的所有文件名
	res = scan_files(sd_root);
	
	//获取记录文件夹个数
	while(dir_name[i++]);
	record_dir_num = i;
	
	//使用json格式发送
	cJSON *root;
	root = cJSON_CreateObject();    //创建一个log json对象
	cJSON_AddNumberToObject(root, "sd_size", sd_size);
	cJSON_AddNumberToObject(root, "sd_used", sd_used);
	cJSON_AddNumberToObject(root, "sd_available", sd_available);
	
	cJSON *record[record_dir_num];	//创建一个record json对象
	cJSON *record_list;  						//创建一个record_list json对象
	for(u8 i=0;i<record_dir_num;i++){
		record[i] = cJSON_CreateObject();	//创建一个record的json对象
		cJSON_AddStringToObject(record[i],"record_data",&dir_name[i]);
		record_list = cJSON_CreateStringArray(file_all_name,file_num[i]);	//创建包含file_num[i]个字符串的字符串数组json对象
		cJSON_AddItemToObject(record[i],"record_lists",record_list);
	}
	
	printf("构建的JSON:\n%s\n", cJSON_Print(root));
	wifi_printf(cJSON_Print(root));	//WiFi回传记录列表
  cJSON_Delete(root);	//删除构建的json对象

	
//res = f_opendir(&dir, path); 
//res = f_readdir(&dir, &fno); 
//  while (((res = f_readdir(&dirs, &Finfo)) == FR_OK) && Finfo.fname[0]) 
//                {
//                    #if _USE_LFN
//                        fn = *Finfo.lfname ? Finfo.lfname : Finfo.fname;
//                    #else
//                        fn = Finfo.fname;
//                    #endif
//                }
	
//加入这一句代码 if (folderInfo.fattrib & AM_DIR)//判断是文件夹
/*************以上代码为 scan_files (char* path) 中的，实现打开某文件夹然后搜索该文件夹内的所有文件，文件名逐一传递给  fn，测试没问题。*/
}

//对记录的log文件进行保存/删除的操作
void log_update(){
	if(sys_flag.log_check){ //发送记录列表
		sys_flag.log_check = false;
		send_loglist();	
	}
	if(vp.log_return!=0){	//发送记录
		//使用ymodem发送log文件？
		//使用http发送log文件？
	}
	if(vp.log_delete!=0){	//删除记录
		
		//返回更新后的记录列表
		send_loglist();	
	}
		
}

