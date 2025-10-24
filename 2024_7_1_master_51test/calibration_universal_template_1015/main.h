#ifndef __MAIN_H
#define __MIAN_H
#include <unistd.h>
#include "adaptive_calibration.h"
#include "1014calibration.h"
#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1

#define FILE_PATH "C:\\Users\\zhujunnan\\Desktop\\qianpeak\\10_49_07_803\\Peak.csv"
//"C:\\Users\\zhujunnan\\Desktop\\qianpeak\\install024\\Peak.csv"

void Output_file_clearing(char *output_filename);
void Tag_write();

void Calibration_Screening_Angle();
void YD_XD_writing(float YD,float XD);
void ang_dopp_rang_snr_vel(float32_t angle,float32_t doppler,float32_t range,float32_t snr,float32_t vel);
#endif