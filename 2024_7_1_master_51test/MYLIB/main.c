#include "Calibration.h"
#include "Mycal_Dll.h"
extern SIMULATION_DATA sim_data;
extern CALIBRATION_MODE CAL_MODE;

extern GTRACK_measurementPoint Peakpoint[PEAK_NUM_POINTS_MAX];
extern Message_VehicleMsgS Message_VehicleMsg;

void Func_Parame_Init()
{
    Function_param_init();
}

void Calibration_Required_data()
{
    for(uint8_t i = 0; i < sim_data.point_num; i++)
    {
        Peakpoint[i].vector.range = sim_data.Cal_data[i].range;        
        Peakpoint[i].vector.azimuth = sim_data.Cal_data[i].azimuth;
        Peakpoint[i].vector.elev = sim_data.Cal_data[i].elevation;
        Peakpoint[i].vector.doppler = sim_data.Cal_data[i].doppler;
        Peakpoint[i].snr = sim_data.Cal_data[i].snr;
    }
    
    Message_VehicleMsg.CurveRadius = sim_data.Message_VehicleMsg.CurveRadius;
    Message_VehicleMsg.Velocity = sim_data.Message_VehicleMsg.Velocity;
    Message_VehicleMsg.YawRate = sim_data.Message_VehicleMsg.YawRate;
    Message_VehicleMsg.SteeringAngle = sim_data.Message_VehicleMsg.SteeringAngle;
}

void Calibration_runing_task(void)
{
    int value;
    while (1)
    {    
         sem_wait(&sim_data.sem2); //wait for the semaphore
         Calibration_Required_data();
            switch(CAL_MODE)
            {
                case CALIBRATION_INIT:
                    Adaptive_CalibrationInit();
                    break;
                case CALIBRATION_RUNING:
                    Calibration_Required_data();
                    Adaptive_Calibration(sim_data.point_num,Peakpoint);
                break;
                default:
                    break;
            }
            if(CAL_MODE == DATA_READ_EXIT||CAL_MODE == CALIBRATION_EXIT) 
            {
                sem_post(&sim_data.sem);
                if(CAL_MODE == DATA_READ_EXIT)
                {
                    printf("FILE_RAND_task_exit\n");
                }else
                {
                    printf("Calibration_runing_task_exit\n");
                }
                return;
            }
            sem_post(&sim_data.sem);
    }
}


void Data_reading_task(void)
{ 
    printf("Data_reading_task_start\n");
    FILE_Read();
}


void main()
{
    Func_Parame_Init();

    pthread_create(&sim_data.Data_reading_thread, NULL, (void*)Data_reading_task, NULL);
    pthread_create(&sim_data.Calibration_thread, NULL, (void*)Calibration_runing_task,NULL);

    Function_param_join();
}