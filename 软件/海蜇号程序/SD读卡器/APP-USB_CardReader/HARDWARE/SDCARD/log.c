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

#define sd_root "0:/"		//��Ŀ¼
#define DIR_MAX_NUM 256	//��Ŀ¼�������ļ��и���
#define FILE_MAX_NUM	256	//��Ŀ¼������ļ�����
char dir_name[DIR_MAX_NUM];		//�洢��Ŀ¼���ļ��е�����
char *file_in_one_dir_name[FILE_MAX_NUM];	//�洢һ�����ļ����������ļ�����
char *file_all_name[DIR_MAX_NUM];		//�洢�������ļ����������ļ�����
u8 file_num[DIR_MAX_NUM];			//�洢ÿ�����ļ����������ļ�����

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
							dir_name[num] = fno.fname[0];						//�����Ŀ¼�µ������ļ�����
							i = strlen(path);
							sprintf(&path[i], "/%s", fno.fname);
							res = scan_files(path);                    /* Enter the directory */
							if (res != FR_OK) break;
							path[i] = 0;
					} else {                                       /* It is a file. */
							printf("%s/%s\n", path, fno.fname);
							file_in_one_dir_name[num] = fno.fname;						//���浱ǰ���ļ����µ�һ���ļ���
//							file_num[num] = ;	//���浱ǰ���ļ����µ��ļ�����
							num++;
					}
			}
			f_closedir(&dir);
	}
	return res;
}

//sd����¼����
void logging(){
	if(!vp.task_run){	//��û�п�ʼˮ������
		f_close(&File);	//�ر���ǰ�򿪵��ļ�
		return;
	}
	if(sys_flag.log_creat==true){	//ˮ������һ�δ���һ��log�ļ�
		sys_flag.log_creat = false;
		char dir_name[32];
		char file_name[32];
		
		//�����ļ��У���year-month-day����
		sprintf(dir_name,"0:/%4d%02d%02d",vp.sys_time.year,vp.sys_time.month,vp.sys_time.day);
		res = f_mkdir(dir_name);	
		if(res != FR_OK && res != FR_EXIST){	//�ļ��д������ɹ����˳�
			printf("[error] Creat Folder Failed: %d \r\n",res);
			return;
		}
		if(res == FR_EXIST)
			printf("[ok] Folder %s already existed \r\n",dir_name);
		else printf("[ok] Creat Folder %s \r\n",dir_name);
		
		//�����ļ�����year-month-day-hour-min-sec.log����
		sprintf(file_name,"%s/%4d%02d%02d_%02d%02d%02d.txt",
											dir_name,
											vp.sys_time.year,vp.sys_time.month,vp.sys_time.day,
											vp.sys_time.hour,vp.sys_time.minute,vp.sys_time.second);
		res = f_open(&File,file_name,FA_OPEN_EXISTING);
		if(res==FR_OK){		//����Ѵ��ڴ��ļ����򸲸�(��ɾ���ٴ���)
			printf("[ok] File %s already exist,rewrite! \r\n",file_name);
			f_unlink(file_name);
		}
		res = f_open(&File,file_name,FA_OPEN_ALWAYS|FA_READ|FA_WRITE);
		if(res != FR_OK){	//�ļ��������ɹ����˳�
			printf("[error] Creat file Failed: %d \r\n",res);
			f_close(&File);
			return;
		}
		else printf("[ok] Creat File %s \r\n",file_name);
		
		//���ļ�ͷ��д�������б�����
		/*
		task:list	type	time	value
					1		1			10.0	5.00
					2		3			10.0	5.00
					...(д������Ϊ�յ�������)
		*/
//		f_lseek(&File,100);  //���ļ���ʼ100�ֽڿ�ʼд��(ÿ���ϵ��ͷд��)
		char *task_list = mymalloc(SRAMIN,1024);
		//�����б�����
		//sprintf��������д����ַ��������������ַ���׷�����ַ���ĩβ�Ŀ��ַ������ʧ�ܣ��򷵻�һ��������
		int list_length = sprintf(task_list,"task: \t list \t type \t time \t value \r\n");
		for(u8 i=0;Task_List[i].task_list!=0;i++){	//д������Ϊ�յ�������
			list_length = sprintf(task_list,"%s \t %d \t %d \t %.1f \t %.2f \r\n",
							task_list,Task_List[i].task_num,Task_List[i].task_list,
							Task_List[i].task_beat,Task_List[i].task_val);
		}
		if(list_length<0){	//�������б������ʼ�������˳�������ԭ����maloc������ֽڲ���
			printf("[error] Write task_list fails: %d \r\n",list_length);
			myfree(SRAMIN,task_list);
			return;
		}
		res = f_write(&File,task_list,list_length,&_bw);//���ļ�����д����.д100�ֽڣ�bwʵ�ʷ���д���ֽ���(���ڼ���ļ�����)sizeof(task_list)/sizeof(char)
		printf("[ok] write task_list \r\n");
		myfree(SRAMIN,task_list);
		
		//д�뱣�����������
		char *file_hearder = mymalloc(SRAMIN,1024);
		int type_length = sprintf(file_hearder,"acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,eular_x,eular_y,eular_z,pressure,depth,temperature,voltage,roll_in,pitch_in,yaw_in,throttle_in,motor1,motor2,motor3,motor4 \r\n");
		if(type_length<0){
			printf("[error] Write data type fails: %d \r\n",type_length);
			myfree(SRAMIN,file_hearder);
			return;
		}
		res = f_write(&File,file_hearder,type_length,&_bw);
		printf("[ok] write date type \r\n");
		f_sync(&File);	//�����ļ���¼����ֹϵͳ�������ݶ�ʧ
		f_close(&File);
//		File = (void*)0;	//�ļ��رպ���Ҫ���ļ�ָ��ָ��գ����������ֹ��������ָ�룬��������������ɲ���Ҫ���鷳���磺fp = NULL;
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
					//����������
					Sensor_latest.acc.x,Sensor_latest.acc.x,Sensor_latest.acc.x,
					Sensor_latest.gyro.x,Sensor_latest.gyro.x,Sensor_latest.gyro.x,
					Sensor_latest.mag.x,Sensor_latest.mag.x,Sensor_latest.mag.x,
					Sensor_latest.eluar.x,Sensor_latest.eluar.x,Sensor_latest.eluar.x,
					Sensor_latest.pressure,Sensor_latest.depth,Sensor_latest.temp,Sensor_latest.power,
					//�������
					roll_in,pitch_in,yaw_in,throttle_in,
					motor_out[0],motor_out[1],motor_out[2],motor_out[3]
	);
//	res = f_write(&File,logging_data,sizeof(logging_data)/sizeof(char),&_bw);
//	printf("file write result: %d \r\n",res);
//	f_sync(&File);
}

/*
json�����ʽ��
	{
		"sd_size":sd_size,			//sd����
		"sd_used":sd_used,			//sd��ʹ������
		"sd_size":sd_available,	//sd��������
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
//sd�����ؼ�¼�б�
void send_loglist(){
	u8 record_dir_num=0;	//��¼�ļ��и���
//	u16 record_file_num=0;	//��¼�ļ�����
	u16 sd_size=0;//sd����
	u16 sd_used=0;//sd��ʹ������
	u16 sd_available=0;//sd��������
	u16 i =0;
	memset(dir_name,0,sizeof(char)*DIR_MAX_NUM);
	
//	char record_dir[record_dir_num];	//�洢�����ļ�������	
//	char *record_file[record_dir_num];//�洢ָ��ÿ���ļ����������ļ�����ָ��

	//������Ŀ¼�µ��ļ��У���ȡÿ���ļ����µ������ļ���
	res = scan_files(sd_root);
	
	//��ȡ��¼�ļ��и���
	while(dir_name[i++]);
	record_dir_num = i;
	
	//ʹ��json��ʽ����
	cJSON *root;
	root = cJSON_CreateObject();    //����һ��log json����
	cJSON_AddNumberToObject(root, "sd_size", sd_size);
	cJSON_AddNumberToObject(root, "sd_used", sd_used);
	cJSON_AddNumberToObject(root, "sd_available", sd_available);
	
	cJSON *record[record_dir_num];	//����һ��record json����
	cJSON *record_list;  						//����һ��record_list json����
	for(u8 i=0;i<record_dir_num;i++){
		record[i] = cJSON_CreateObject();	//����һ��record��json����
		cJSON_AddStringToObject(record[i],"record_data",&dir_name[i]);
		record_list = cJSON_CreateStringArray(file_all_name,file_num[i]);	//��������file_num[i]���ַ������ַ�������json����
		cJSON_AddItemToObject(record[i],"record_lists",record_list);
	}
	
	printf("������JSON:\n%s\n", cJSON_Print(root));
	wifi_printf(cJSON_Print(root));	//WiFi�ش���¼�б�
  cJSON_Delete(root);	//ɾ��������json����

	
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
	
//������һ����� if (folderInfo.fattrib & AM_DIR)//�ж����ļ���
/*************���ϴ���Ϊ scan_files (char* path) �еģ�ʵ�ִ�ĳ�ļ���Ȼ���������ļ����ڵ������ļ����ļ�����һ���ݸ�  fn������û���⡣*/
}

//�Լ�¼��log�ļ����б���/ɾ���Ĳ���
void log_update(){
	if(sys_flag.log_check){ //���ͼ�¼�б�
		sys_flag.log_check = false;
		send_loglist();	
	}
	if(vp.log_return!=0){	//���ͼ�¼
		//ʹ��ymodem����log�ļ���
		//ʹ��http����log�ļ���
	}
	if(vp.log_delete!=0){	//ɾ����¼
		
		//���ظ��º�ļ�¼�б�
		send_loglist();	
	}
		
}

