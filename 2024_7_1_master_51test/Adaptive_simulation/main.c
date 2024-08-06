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
int NO_DATA_NUM = 0;
#define Frame_number 0
#define Serial_number 1
#define Range  2
#define Doppler  3
#define Azimuth  4
#define Snr  7

#define LIST_NUM  15
#define line_NUM  200

sem_t semaphore,semaphore1,semaphore2; 

// void Single_frame_data_truncation(int num_list, char *token)
// {
//     if((num_list == Serial_number)&&(strcmp(token,"0") == 0))
//     {
//         NO_DATA_NUM++;
//     }

// }
void Calibration_Required_data()
{
    /*********Message_VehicleMsgS********/
    Calibration_Message.Message_VehicleMsgS.Velocity = 30.0f;
    Calibration_Message.Message_VehicleMsgS.YawRate  = 0.3f;
    Calibration_Message.Message_VehicleMsgS.SteeringAngle = 1.0f;
    Calibration_Message.Message_VehicleMsgS.CurveRadius = 400.0f;
    /*********Message_VehicleMsgS********/

    Calibration_Message.RadarParaS.InstallAngle = 0.0f;

    memcpy(&CalibrationPara,&Calibration_Message.CalibrationParaS,sizeof(CalibrationPara));
    memcpy(&Message_VehicleMsg,&Calibration_Message.Message_VehicleMsgS,sizeof(Message_VehicleMsg));
    memcpy(&RadarPara,&Calibration_Message.RadarParaS,sizeof(RadarPara));
}

void FILE_RAND(void)
{
    char buffer[2048];
    int line = 0;
    int list = 0;
    bool flag = false;
    //int i = 0;
    int point_id;
    int frame_num = 0;
    int frame_num_temp = 0xff;

    FILE *output_fp = fopen(FILE_PATH,"r");
    if (NULL == output_fp)
    {
        perror("open_output_file error");
        return;
    }
    sem_wait(&semaphore); //wait for the semaphore
    while (fgets(buffer, sizeof(buffer), output_fp)) //this is line
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
            if((list == Frame_number)||(list == Range)|| (list == Doppler)||(list == Azimuth)||(list == Snr))
            {
                if((list == Range)&&(strcmp(token,"0") == 0))
                {
                    break;
                }
                switch (list)
                {
                    case Frame_number:
                        //frame_num  = atoi(token);  
                        if(frame_num_temp != atoi(token))
                        {
                            frame_num_temp = atoi(token);
                            point_id = 0;
                            printf("Release semaphore\n");
                            //sem_post(&semaphore1);  //Release semaphore
                            
                            //sem_wait(&semaphore2); //wait for the semaphore

                            sem_post(&semaphore1);
                            sem_wait(&semaphore);
                            

                        }else{
                            point_id++;
                            Calibration_Message.or_point_cloud_format_t.point_count = point_id;
                            printf("point_id = %d\n", point_id);
                        }

                    break;
                    case Range:
                        Calibration_Message.or_point_cloud_format_t.term[point_id].range = (float)atof(token);
                        //printf(" FILE_RAND: range[%d] = %f",point_id,Calibration_Message.or_point_cloud_format_t.term[point_id].range);
                    break;
                    case Doppler:
                        Calibration_Message.or_point_cloud_format_t.term[point_id].doppler = (float)atof(token);
                        //printf(" doppler[%d] = %f",point_id,Calibration_Message.or_point_cloud_format_t.term[point_id].doppler);
                    break;
                    case Azimuth:
                        Calibration_Message.or_point_cloud_format_t.term[point_id].azimuth = (float)atof(token);
                        //printf(" azimuth[%d] = %f",point_id,Calibration_Message.or_point_cloud_format_t.term[point_id].azimuth);
                    break;
                    case Snr:
                        Calibration_Message.or_point_cloud_format_t.term[point_id].snr = (float)atof(token);
                         //printf(" snr[%d] = %f",point_id,Calibration_Message.or_point_cloud_format_t.term[point_id].snr);
                         
                    break;
                }
                //printf("token = %s,list=%d", token,list);
                
                //printf(" ");
                //printf("\n");
                token = strtok(NULL, Split_symbol);
            }else
            {
                token = strtok(NULL, Split_symbol);
            }
            ++list;
        }
        /*********************** */
        line++;
        //printf("line = %d\n",line);
        // if (line > 250)
        // {
        //     break;
        // }
        /*********************** */
        list = 0;
        //point_id++;
        //printf("\n");
    }
    fclose(output_fp);
}

void Data_reading_task(void)
{ 
    // while (1)
    // {
         printf("/**********FILE_RAND_start************/\n");
        
        FILE_RAND();
         printf("/**********FILE_RAND_END*************/\n");
       
    // }
}

void Calibration_runing_task(void)
{
    while (1)
    {
        // usleep(2000);
        // printf("/**********Calibration_runing_start************/\n");
        sem_wait(&semaphore1); //wait for the semaphore
        
         Calibration_Required_data();

        // for(int i = 0; i < Calibration_Message.or_point_cloud_format_t.point_count ; i++)
        //     printf("rang[%d] = %f, doppler[%d] = %f, azimuth[%d] = %f, snr[%d] = %f\n",i,Calibration_Message.or_point_cloud_format_t.term[i].range,i,Calibration_Message.or_point_cloud_format_t.term[i].doppler,i,Calibration_Message.or_point_cloud_format_t.term[i].azimuth,i,Calibration_Message.or_point_cloud_format_t.term[i].snr);

        // swit1ch (CAL_MODE)
        // {
        //     case CALIBRATION_INIT:
        //         AdaptiveCalStart();
        //         break;

        //     case CALIBRATION_RUNING:
        //         Adaptive_Calibration(&Calibration_Message.or_point_cloud_format_t);
        //     break;
            
        //     default:
        //         pthread_exit(NULL);
        //         break;
        // }
        
        

        //sem_post(&semaphore2);  //Release semaphore
        sem_post(&semaphore);
        printf("/**********Calibration_runing_END************/\n");
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

    if(sem_init(&semaphore2,0,0) != 0){
        perror("sem_init error");
        exit(EXIT_FAILURE);
    }

    pthread_t Data_reading_thread,Calibration_thread;
    //sem_post(&semaphore); 
    pthread_create(&Data_reading_thread, NULL, (void*)Data_reading_task, NULL);
    pthread_create(&Calibration_thread, NULL, (void*)Calibration_runing_task,NULL);
    
    pthread_join(Data_reading_thread, NULL);  
    pthread_join(Calibration_thread, NULL);  
}