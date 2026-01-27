#ifndef __MAIN_H
#define __MIAN_H
#include <unistd.h>
#include "adaptive_calibration.h"
//#define FILE_PATH "C:\\Users\\zhujunnan\\Desktop\\QIRUI\\Peak.csv"
// #include "stdio.h"
// #include "stdlib.h"
// #include "string.h"
// #include "math.h"
// #include "time.h"
// #include "windows.h"
// #include "process.h"
// #include "direct.h"
// #include "io.h"
// #include "sys/stat.h"
// #include "sys/types.h"
// #include "sys/timeb.h"
// #include "sys/types.h"

#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1

#define FILE_PATH "C:\\Users\\zhujunnan\\Desktop\\JETOUR\\17_47_39_025\\Peak.csv"

void Output_file_clearing(char *output_filename);
void Tag_write();

void Calibration_Screening_Angle();
void YD_XD_writing(float YD,float XD);

#endif