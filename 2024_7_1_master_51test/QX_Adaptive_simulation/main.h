#ifndef __MAIN_H
#define __MIAN_H
#define FILE_PATH "C:\\Users\\zhujunnan\\Desktop\\xingrui\\Peak2.csv"
#define Filter_Angle_Output_File_PATH "C:\\Users\\zhujunnan\\Desktop\\MuGITHUB\\2024_7_1_master_51test\\Filter_Angle_output.txt"

//#include "graphics_for_cal.h"
void Output_file_clearing(char *output_filename);
void Tag_write(void);
void YD_XD_writing(float YD,float XD);
void Calibration_Screening_Angle(void);
#endif