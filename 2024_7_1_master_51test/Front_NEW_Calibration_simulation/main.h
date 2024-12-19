#ifndef __MAIN_H
#define __MIAN_H
//#define FILE_PATH "C:\\Users\\zhujunnan\\Desktop\\QIRUI\\Peak.csv"
#define FILE_PATH "C:\\Users\\zhujunnan\\Desktop\\NEW_CALIBRATION_temp_FILE\\17_43_12_580\\17_43_12_580\\Peak.csv"

void Output_file_clearing(char *output_filename);
void Tag_write();

void Calibration_Screening_Angle();
void YD_XD_writing(float YD,float XD);

#endif