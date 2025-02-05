#include "QX_Adaptive.h"
#include <pthread.h>
#include "main.h"
#include <semaphore.h>

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
Calibration_Date Calibration_Message = {0};
extern  CalibrationParaS CalibrationPara;
extern  Message_VehicleMsgS Message_VehicleMsg;
extern  RadarParaS RadarPara;
extern  GTRACK_measurementPoint Peakpoint[GTRACK_NUM_POINTS_MAX] ;
CALIBRATION_MODE CAL_MODE = CALIBRATION_INIT;
const char *Split_symbol = ",";
#define ang_to_rad  PI/180.0f

#define Frame_number 0
#define Serial_number 1
#define Range  2
#define Doppler  3
#define Snr  4
#define Azimuth  7
#define Vel  10
#define Yaw 13
#define Steering 14
#define Cur 15

#define LIST_NUM  15
#define line_NUM  200

sem_t semaphore,semaphore1;

void Data_Writing(float Data,int list,char *Title)
{
    
}

void Calibration_Required_data()
{
    /*********Message_VehicleMsgS********/
//     //Calibration_Message.Message_VehicleMsgS.Velocity = 25.0f;
//     Calibration_Message.Message_VehicleMsgS.YawRate  = 0.3f;
//     Calibration_Message.Message_VehicleMsgS.SteeringAngle = 1.0f;
//     Calibration_Message.Message_VehicleMsgS.CurveRadius = 400.0f;
//     /*********Message_VehicleMsgS********/
//     Calibration_Message.RadarParaS.InstallAngle = 0.0f;
//     //memcpy(&CalibrationPara,&Calibration_Message.CalibrationParaS,sizeof(CalibrationPara));
//     memcpy(&Message_VehicleMsg,&Calibration_Message.Message_VehicleMsgS,sizeof(Message_VehicleMsg));
//     memcpy(&RadarPara,&Calibration_Message.RadarParaS,sizeof(RadarPara));
        //RadarPara.InstallAngle = RadarInstallAngle;
        RadarPara.InstallPosition = INSTALL_LEFT_BACK;

        for(int i = 0;i < Calibration_Message.or_point_cloud_format_t.point_count;i++)
        {
            Peakpoint[i].vector.azimuth = Calibration_Message.or_point_cloud_format_t.term[i].azimuth;
            Peakpoint[i].vector.range = Calibration_Message.or_point_cloud_format_t.term[i].range;
            Peakpoint[i].vector.doppler = Calibration_Message.or_point_cloud_format_t.term[i].doppler;
            Peakpoint[i].snr = Calibration_Message.or_point_cloud_format_t.term[i].snr;
        }
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
            if(CAL_MODE == DATA_READ_EXIT)
            {
                printf("FILE_RAND_task_exit\n");
            }else
            {
                printf("Calibration_runing_task_exit\n");
            }
            fclose(output_fp);
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
                if((list == Frame_number)||(list == Serial_number)||(list == Range)|| (list == Doppler)||(list == Azimuth)||(list == Snr)||(list == Vel)||(list == Yaw)||(list == Steering)||(list == Cur))
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
                                printf("frame_num_temp = %d\n",frame_num_temp);
                                Calibration_Message.or_point_cloud_format_t.point_count = point_id + 1;

                                point_id = 0;
                                //speed_temp = 0.0f;
                                //printf("Release semaphore\n");
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
                            // speed_temp += (float)atof(token);
                            // Calibration_Message.Message_VehicleMsgS.Velocity = fabs((speed_temp/point_id)*3.6f)+6.0f;
                        /*****************Simulated speed******************/
                        break;
                        case Azimuth:
                            Calibration_Message.or_point_cloud_format_t.term[point_id].azimuth = ((float)atof(token))*ang_to_rad;//
                        break;
                        case Snr:
                            Calibration_Message.or_point_cloud_format_t.term[point_id].snr = (float)atof(token);
                            //printf("SNR: %f\n", (float)atof(token));
                            //sleep(1);
                        break;
                        case Vel:
                            Message_VehicleMsg.Velocity = (float)atof(token);
                        break;
                        case Yaw:
                            Message_VehicleMsg.YawRate = (float)atof(token);
                        break;
                        case Steering:
                            Message_VehicleMsg.SteeringAngle = (float)atof(token);
                        break;
                        case Cur:
                            Message_VehicleMsg.CurveRadius = (float)atof(token);
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
    sem_post(&semaphore1); 
}

void Data_reading_task(void)
{ 
        FILE_Read();
}

void Calibration_runing_task(void)
{
    while (1)
    {
       // printf("CAL_MODE = %d\n",CAL_MODE); //wait for the semaphore
         sem_wait(&semaphore1); //wait for the semaphore
        //printf("point_count = %d\n",Calibration_Message.or_point_cloud_format_t.point_count);
        Calibration_Required_data();
        // for(int i = 0; i < Calibration_Message.or_point_cloud_format_t.point_count ; i++)
        //     printf("rang[%d] = %f, doppler[%d] = %f, azimuth[%d] = %f, snr[%d] = %f\n",i,Peakpoint[i].vector.range,i,Peakpoint[i].vector.doppler,i,Peakpoint[i].vector.azimuth,i,Peakpoint[i].snr);
        //     printf("Message_VehicleMsg.Velocity = %f,Message_VehicleMsg.YawRate = %f,Message_VehicleMsg.SteeringAngle = %f\n",Message_VehicleMsg.Velocity,Message_VehicleMsg.YawRate,Message_VehicleMsg.SteeringAngle);
        //     sleep(1);
            switch(CAL_MODE)
            {
                case CALIBRATION_INIT:
                    //Adaptive_CalibrationInit();
                    AdaptiveCalStart();
                    break;
                case CALIBRATION_RUNING:
                printf("CALIBRATION_RUNING\n");
                //usleep(1000000);
                    Calibration_Required_data();
                    Adaptive_Calibration(Calibration_Message.or_point_cloud_format_t.point_count,Peakpoint);
                break;
                default:
    
                    break;
            }

            if(CAL_MODE == DATA_READ_EXIT||CAL_MODE == CALIBRATION_EXIT) 
            {
                sem_post(&semaphore);
                if(CAL_MODE == DATA_READ_EXIT)
                {
                    printf("FILE_RAND_task_exit\n");
                }else
                {
                    printf("Calibration_runing_task_exit\n");
                }
                return;
            }
            sem_post(&semaphore);
        
    }
}

void main(int argc, char**argv)
{
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

    if (sem_destroy(&semaphore) != 0) {
    perror("sem_destroy");
    exit(EXIT_FAILURE);
    }
    if (sem_destroy(&semaphore1) != 0) {
    perror("sem_destroy");
    exit(EXIT_FAILURE);
    }

    //graphics();
}