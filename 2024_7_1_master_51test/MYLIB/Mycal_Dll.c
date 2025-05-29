#include "Mycal_Dll.h"

//RadarParaS RadarPara = {0};
__declspec(dllexport) SIMULATION_DATA sim_data;
__declspec(dllexport) CALIBRATION_MODE CAL_MODE = {0};

#ifdef FILTERED_POINTS
/* 函数名: void Output_file_clearing(char *output_filename)
 * 描述：文件清空函数，用于在开始新的一轮数据读取前清空输出文件
 * 返回值:NA
 */
__declspec(dllexport) void Output_file_clearing(char *output_filename)
{
    if (truncate(output_filename, 0) == -1) 
    {
        perror("Error truncating file");
        return ;
    }
}


/* 函数名: Tag_write()
 * 描述：标签写入函数，用于在输出文件中写入标签
 * 参数：output_filename：输出文件名
 * 返回值:NA
 */
__declspec(dllexport) void Tag_write(char *output_filename)
{
    FILE *output_fp = fopen(output_filename,"a+");
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
__declspec(dllexport) void Calibration_Screening_Angle(char *output_filename)
{
    Output_file_clearing(output_filename);
    Tag_write(output_filename);
}
#endif

/* 函数名: Calibration_Screening_Angle()
 * 描述 ：YD XD数据写入函数，用于在输出文件中写入标签
 * 返回值:NA
 */
__declspec(dllexport) void YD_XD_writing(float YD,float XD,char *output_filename)
{
    FILE *output_fp = fopen(output_filename,"a+");
    if (NULL == output_fp)
    {
        perror("open_output_file error");
        return;
    }
    fprintf(output_fp,"%f\t%f\n",YD,XD);
    fflush(output_fp);
    fclose(output_fp);
}

/* 函数名: Function_param_init()
 * 描述 ：参数初始化函数，用于在程序开始前初始化参数
 * 返回值:NA
 */
__declspec(dllexport) void Function_param_init()
{
    sim_data.Input_filename = "C:\\Users\\zhujunnan\\Desktop\\simulation_data\\P2341_Peak.csv";
    sim_data.output_filename = "C:\\Users\\zhujunnan\\Desktop\\MuGITHUB\\2024_7_1_master_51test\\Filter_Angle_output.txt";
    sim_data.LIST_NUM = 16;
    sim_data.line_NUM = 200;
    sim_data.Split_symbol = ",";  //分隔符
    sim_data.Frame_number = 0;    //帧列
    sim_data.Serial_number = 1;   //序列号
    sim_data.Range = 2;           //距离列
    sim_data.Doppler = 3;         //速度列
    sim_data.Snr = 4;             //信噪比列
    sim_data.Azimuth = 7;         //方位角列
    sim_data.Vel = 10;            //车速列
    sim_data.Yaw = 13;            //横摆角列
    sim_data.Steer = 14;          //转向角列
    sim_data.Cur = 15;            //曲率半径列
 
    Calibration_Screening_Angle(sim_data.output_filename);


    if(sem_init(&sim_data.sem,0,1) != 0){   //信号量初始化
        perror("sem_init error");
        exit(EXIT_FAILURE);
    }


    if(sem_init(&sim_data.sem2,0,0) != 0){  //信号量初始化
        perror("sem_init error");
        exit(EXIT_FAILURE);
    }                           

    CAL_MODE = CALIBRATION_INIT;
}

/* 函数名: Function_param_join()
 * 描述 ：功能运行结束，资源释放
 * 返回值:NA
 */
__declspec(dllexport) void Function_param_join()
{
    pthread_join(sim_data.Data_reading_thread, NULL);  
    pthread_join(sim_data.Data_reading_thread, NULL);  

    if (sem_destroy(&sim_data.sem) != 0) {
    perror("sem_destroy");
    exit(EXIT_FAILURE);
    }
    if (sem_destroy(&sim_data.sem2) != 0) {
    perror("sem_destroy");
    exit(EXIT_FAILURE);
    }
}





/* 函数名: FILE_Read()
 * 描述 ：原始点获取
 * 返回值:NA
 */
__declspec(dllexport) void FILE_Read()
{
    char buffer[2048];
    int line = 0;
    int list = 0;
    bool flag = false;
    int point_id;
    int frame_num_temp = 0x01;
    float32_t speed_temp = 0.0f;

    FILE *output_fp = fopen(sim_data.Input_filename,"r");
    if (NULL == output_fp)
    {
        perror("open_output_file error");
        return;
    }
    sem_wait(&sim_data.sem); //wait for the semaphore
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
            }///
            fclose(output_fp);
            sem_post(&sim_data.sem2);
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
            token = strtok(buffer, sim_data.Split_symbol);

            while (token != NULL) //this is list
            {   
                if(list > sim_data.LIST_NUM) //Only the first 15 columns are read
                {
                    break;
                }
                if((list == sim_data.Frame_number)||(list == sim_data.Serial_number)||(list == sim_data.Range)|| (list == sim_data.Doppler)||(list == sim_data.Azimuth)||(list == sim_data.Snr)||(list == sim_data.Vel)||(list == sim_data.Yaw)||(list == sim_data.Steer)||(list == sim_data.Cur))
                {
                    if((list == sim_data.Range)&&(strcmp(token,"0") == 0))
                    {
                        break;
                    }else if (list == sim_data.Serial_number)
                    {
                        if(atoi(token) != 0)
                        {
                            point_id = atoi(token);
                        }
                    }/************Simulated speed******************/
                    else if (list == sim_data.Frame_number)
                    {       
                        if(frame_num_temp != atoi(token))
                        {
                            frame_num_temp = atoi(token);
                            sim_data.point_num = point_id + 1;
                            point_id = 0;
                            speed_temp = 0.0f;
                            printf("Frame_num_temp:%d\n",frame_num_temp);
                            sem_post(&sim_data.sem2);
                            sem_wait(&sim_data.sem);
                        }else{
                            //This is Single frame valid data
                        }
                    }
                    else if (list == sim_data.Range)
                    {
                        sim_data.Cal_data[point_id].range = (float)atof(token);
                    }
                    else if (list == sim_data.Doppler)
                    {
                        sim_data.Cal_data[point_id].doppler = (float)atof(token);
                    }
                    else if (list == sim_data.Azimuth)
                    {
                        sim_data.Cal_data[point_id].azimuth = ((float)atof(token))*ang_to_rad;
                    }
                    else if (list == sim_data.Snr)
                    {
                        sim_data.Cal_data[point_id].snr = (float)atof(token);
                    }
                    else if (list == sim_data.Vel)
                    {
                        sim_data.Message_VehicleMsg.Velocity = (float)atof(token);
                        //printf("Velocity:%f\n",sim_data.Message_VehicleMsg.Velocity);
                    }
                    else if (list == sim_data.Yaw)
                    {
                        sim_data.Message_VehicleMsg.YawRate = (float)atof(token);
                    }
                    else if (list == sim_data.Steer)
                    {
                        sim_data.Message_VehicleMsg.SteeringAngle = (float)atof(token);
                    }
                    else if (list == sim_data.Cur)
                    {
                        sim_data.Message_VehicleMsg.CurveRadius = (float)atof(token);
                    }
                    
                    token = strtok(NULL, sim_data.Split_symbol);
                }else
                {
                    token = strtok(NULL, sim_data.Split_symbol);
                }
                
                ++list;
            }
            line++;
            list = 0;
            //printf("line:%d\n",line);
        }
    }
    fclose(output_fp);
    CAL_MODE = DATA_READ_EXIT;   //File read complete
    sem_post(&sim_data.sem2); 
}
    