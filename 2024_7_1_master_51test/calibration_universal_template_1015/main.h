#ifndef __MAIN_H
#define __MIAN_H
#include <unistd.h>
#include "adaptive_calibration.h"
#include "1014calibration.h"
#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1

#define FILE_PATH "C:\\Users\\zhujunnan\\Desktop\\ICAI05\\installangle2_96\\Peak.csv"

void Output_file_clearing(char *output_filename);
void Tag_write();

void Calibration_Screening_Angle();
void YD_XD_writing(float YD,float XD);

#endif