#include "Adaptive.h"
#include <pthread.h>
#include "main.h"
#include <semaphore.h>
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
Calibration_Date Calibration_Message = {0};
extern  CalibrationParaS CalibrationPara;
extern  Message_VehicleMsgS Message_VehicleMsg;
extern  RadarParaS RadarPara;
CALIBRATION_MODE CAL_MODE = CALIBRATION_INIT;
const char *Split_symbol = ",";
#define ang_to_rad  PI/180.0f
#define Filter_Angle_Output_File_PATH "C:\\Users\\zhujunnan\\Desktop\\MuGITHUB\\2024_7_1_master_51test\\Filter_Angle_output.txt"

#define Frame_number 0
#define Serial_number 1
#define Range  2
#define Doppler  3
#define Azimuth  4
#define Snr  7

#define LIST_NUM  15
#define line_NUM  200

sem_t semaphore,semaphore1;
/* 函数名: void Output_file_clearing(char *output_filename)
 * 描述：文件清空函数，用于在开始新的一轮数据读取前清空输出文件
 * 返回值:NA
 */
void Output_file_clearing(char *output_filename)
{
    if (truncate(output_filename, 0) == -1) 
    {
        perror("Error truncating file");
        return ;
    }
}

/* 函数名: Tag_write()
 * 描述：标签写入函数，用于在输出文件中写入标签
 * 返回值:NA
 */
void Tag_write()
{
    FILE *output_fp = fopen(Filter_Angle_Output_File_PATH,"a+");
    if (NULL == output_fp)
    {
        perror("open_output_file error");
        return;
    }
    fprintf(output_fp,"%s\t","XD");
    fprintf(output_fp,"%s\n","YD");
    fflush(output_fp);
    fclose(output_fp);
}
/* 函数名: Calibration_Screening_Angle()
 * 描述：XD YD标签写入函数，用于在输出文件中写入标签
 * 返回值:NA
 */
void Calibration_Screening_Angle()
{
    Output_file_clearing(Filter_Angle_Output_File_PATH);
    Tag_write();
}

/* 函数名: Calibration_Screening_Angle()
 * 描述 ：YD XD数据写入函数，用于在输出文件中写入标签
 * 返回值:NA
 */

void YD_XD_writing(float YD,float XD)
{
    FILE *output_fp = fopen(Filter_Angle_Output_File_PATH,"a+");
    if (NULL == output_fp)
    {
        perror("open_output_file error");
        return;
    }
    fprintf(output_fp,"%f\t%f\n",YD,XD);
    fflush(output_fp);
    fclose(output_fp);
}


void Calibration_Required_data()
{
    /*********Message_VehicleMsgS********/
    //Calibration_Message.Message_VehicleMsgS.Velocity = 25.0f;
    Calibration_Message.Message_VehicleMsgS.YawRate  = 0.3f;
    Calibration_Message.Message_VehicleMsgS.SteeringAngle = 1.0f;
    Calibration_Message.Message_VehicleMsgS.CurveRadius = 400.0f;
    /*********Message_VehicleMsgS********/
    Calibration_Message.RadarParaS.InstallAngle = 0.0f;
    //memcpy(&CalibrationPara,&Calibration_Message.CalibrationParaS,sizeof(CalibrationPara));
    memcpy(&Message_VehicleMsg,&Calibration_Message.Message_VehicleMsgS,sizeof(Message_VehicleMsg));
    memcpy(&RadarPara,&Calibration_Message.RadarParaS,sizeof(RadarPara));
}

void FILE_Read(void)
{
    char buffer[2048];
    int line = 0;
    int list = 0;
    bool flag = false;
    int point_id;
    int frame_num_temp = 0x01;
    float32_t speed_temp = 0.0f;

    FILE *output_fp = fopen(FILE_PATH,"r");
    if (NULL == output_fp)
    {
        perror("open_output_file error");
        return;
    }
    sem_wait(&semaphore); //wait for the semaphore
    while (fgets(buffer, sizeof(buffer), output_fp)) //this is line
    {
        if(CAL_MODE == DATA_READ_EXIT||CAL_MODE == CALIBRATION_EXIT) 
        {
            printf("FILE_RAND_task_exit\n");
            sem_post(&semaphore1);
            return;
        }
        else
        {
            if (flag == false) //Title filtering
            {
                flag = true;
                continue;
            }
            char *token;
            // 分割每行数据
            token = strtok(buffer, Split_symbol);
            while (token != NULL) //this is list
            {   
                if(list > LIST_NUM) //Only the first 15 columns are read
                {
                    break;
                }
                if((list == Frame_number)||(list == Serial_number)||(list == Range)|| (list == Doppler)||(list == Azimuth)||(list == Snr))
                {
                    if((list == Range)&&(strcmp(token,"0") == 0))
                    {
                        break;
                    }else if (list == Serial_number)
                    {
                        if(atoi(token) != 0)
                        {
                            point_id = atoi(token);
                        }
                    }
    
                    switch (list)
                    {
                        case Frame_number:
                            if(frame_num_temp != atoi(token))
                            {
                                frame_num_temp = atoi(token);
                                Calibration_Message.or_point_cloud_format_t.point_count = point_id + 1;
                                point_id = 0;
                                speed_temp = 0.0f;
                                printf("Release semaphore\n");
                                sem_post(&semaphore1);
                                sem_wait(&semaphore);
                            }else{
                                //This is Single frame valid data
                            }
                        break;
                        case Range:
                            Calibration_Message.or_point_cloud_format_t.term[point_id].range = (float)atof(token);
                        break;
                        case Doppler:
                            Calibration_Message.or_point_cloud_format_t.term[point_id].doppler = (float)atof(token);
                        /*****************Simulated speed******************/
                            speed_temp += (float)atof(token);
                            Calibration_Message.Message_VehicleMsgS.Velocity = fabs((speed_temp/point_id)*3.6f)+6.0f;
                        /*****************Simulated speed******************/
                        break;
                        case Azimuth:
                            Calibration_Message.or_point_cloud_format_t.term[point_id].azimuth = ((float)atof(token))*ang_to_rad;
                        break;
                        case Snr:
                            Calibration_Message.or_point_cloud_format_t.term[point_id].snr = (float)atof(token);
                        break;
                    }
                    token = strtok(NULL, Split_symbol);
                }else
                {
                    token = strtok(NULL, Split_symbol);
                }
                ++list;
            }
            line++;
            list = 0;
        }
    }
    fclose(output_fp);
    CAL_MODE = DATA_READ_EXIT;   //File read complete
}

void Data_reading_task(void)
{ 
        FILE_Read();
}

void Calibration_runing_task(void)
{
    while (1)
    {
        sem_wait(&semaphore1); //wait for the semaphore
        Calibration_Required_data();
        //printf("point_count = %d\n",Calibration_Message.or_point_cloud_format_t.point_count);
        // for(int i = 0; i < Calibration_Message.or_point_cloud_format_t.point_count ; i++)
        //printf("rang[%d] = %f, doppler[%d] = %f, azimuth[%d] = %f, snr[%d] = %f\n",i,Calibration_Message.or_point_cloud_format_t.term[i].range,i,Calibration_Message.or_point_cloud_format_t.term[i].doppler,i,Calibration_Message.or_point_cloud_format_t.term[i].azimuth,i,Calibration_Message.or_point_cloud_format_t.term[i].snr);

            switch(CAL_MODE)
            {
                case CALIBRATION_INIT:
                    AdaptiveCalStart();
                    break;
                case CALIBRATION_RUNING:
                    Adaptive_Calibration(&Calibration_Message.or_point_cloud_format_t);
                break;
                default:
    
                    break;
            }
            if(CAL_MODE == DATA_READ_EXIT||CAL_MODE == CALIBRATION_EXIT) 
            {
                sem_post(&semaphore);
                printf("Calibration_runing_task_exit\n");
                return;
            }
            sem_post(&semaphore);
    }
}

void main(int argc, char**argv)
{
    Calibration_Screening_Angle();
    if(sem_init(&semaphore,0,1) != 0){
        perror("sem_init error");
        exit(EXIT_FAILURE);
    }

    if(sem_init(&semaphore1,0,0) != 0){
        perror("sem_init error");
        exit(EXIT_FAILURE);
    }
    pthread_t Data_reading_thread,Calibration_thread;
    pthread_create(&Data_reading_thread, NULL, (void*)Data_reading_task, NULL);
    pthread_create(&Calibration_thread, NULL, (void*)Calibration_runing_task,NULL);
    
    pthread_join(Data_reading_thread, NULL);  
    pthread_join(Calibration_thread, NULL);  
}