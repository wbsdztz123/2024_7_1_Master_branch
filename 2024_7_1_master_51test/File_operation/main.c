#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <stdbool.h>
#include <unistd.h>

#define Destination_file_path "C:\\Users\\zhujunnan\\Desktop\\QIRUI\\Angle_merging.txt"
#define Horizontal_angle "horizontal_angle"
#define Vertical_angle "vertical_angle"

//C:\Users\zhujunnan\Desktop\QIRUI\Chery_Calibration_pressure2024_0731_2\Chery_Calibration_pressure2024_0731_2\LogData\OK\2024\8_1 输入
//C:\Users\zhujunnan\Desktop\QIRUI\Angle_merging.txt  输出
//C:\Users\zhujunnan\Desktop\QIRUI\7_31 输入
//C:/Users/zhujunnan/Desktop/QIRUI/7_31 输入
void Tag_write()
{
    FILE *output_fp = fopen(Destination_file_path,"a+");
    if (NULL == output_fp)
    {
        perror("open_output_file error");
        return;
    }
    fprintf(output_fp,"%s\t",Horizontal_angle);
    fprintf(output_fp,"%s\n",Vertical_angle);
    fflush(output_fp);
    fclose(output_fp);
}
void Output_file_clearing(char *output_filename)
{
    if (truncate(output_filename, 0) == -1) {
        perror("Error truncating file");
        return ;
    }
}

void Horizontal_Angle_writing(char *Horizontal_Angle)
{
    FILE *output_fp = fopen(Destination_file_path,"a+");
    if (NULL == output_fp)
    {
        perror("open_output_file error");
        return;
    }
    if(strlen(Horizontal_Angle)!=0) fprintf(output_fp,"%s\t",Horizontal_Angle);

    fflush(output_fp);
    fclose(output_fp);
}
void elev_Angle_written(char *elev_Angle)
{
    FILE *output_fp = fopen(Destination_file_path,"a+");
    if (NULL == output_fp)
    {
        perror("open_output_file error");
        return;
    }
    if(strlen(elev_Angle) != 0)  fprintf(output_fp,"%s\n",elev_Angle);
    fflush(output_fp);
    fclose(output_fp);
}
void Three_level_path_file_operation(char *second_level_path)
{
    char Three_level_path[200] = {0};
    char angle_buffer[20] = {0};
    char file_char;
    int i = 0;
    bool Angle_jud = false;
    sprintf(Three_level_path,"%s\\result.txt",second_level_path);
    FILE *fp = fopen(Three_level_path,"r");
    if (NULL == fp)
    {
        perror("ERROR opening file");
        return;
    }
        while ((file_char = fgetc(fp)) != EOF) {
            if(file_char == ':') 
            {
                Angle_jud = true;
                continue;
            }
            if(Angle_jud == true)
            {
                if ((0 < file_char)&&(file_char < 127))
                {
                    angle_buffer[i++] = file_char;
                }
                else
                {
                    Angle_jud = false;
                    //水平角度写入
                    Horizontal_Angle_writing(angle_buffer);
                    memset(angle_buffer, 0,sizeof(angle_buffer));
                    i = 0;
                }
            }
    }
    if (file_char == EOF)
    {   //俯仰角度写入
        elev_Angle_written(angle_buffer);
    }
    
    fclose(fp);
    
}
void dir_second_level_operation(char *File_path,char *dir_path)
{
    
    char second_level_path[200];
    sprintf(second_level_path,"%s\\%s",File_path,dir_path);
    DIR *dir_second_level = opendir(second_level_path);
    if(NULL == dir_second_level)
    {
        printf("dir_second_level == NULL\n");
    }
    struct dirent *second_entry;
    while((second_entry = readdir(dir_second_level)) != NULL)
    {
        if (0 == strcmp(second_entry->d_name, ".") || 0 == strcmp(second_entry->d_name, "..")) {
            continue;
        }
        if(0 == strcmp(second_entry->d_name,"result.txt"))
        {
            //三级路径下文件操作
            Three_level_path_file_operation(second_level_path);
        }else
        {
            //log.txt  The current file is not the destination file
        }
    }
    closedir(dir_second_level);
}
int main(int argc,char **argv)
{
    printf("software_version:08.01.1\n");
    Output_file_clearing(Destination_file_path);
    Tag_write();
    struct dirent *entry; 
    DIR *dir = opendir(argv[1]); 
    if(argc != 2)
    {
        fprintf(stderr,"Parameter input error\n");
        return -1;
    }
    if(dir == NULL)
    {
        perror("opendir_error");
        return -1;
    }
    while((entry = readdir(dir)) != NULL)
    {
        if (0 == strcmp(entry->d_name, ".") || 0 == strcmp(entry->d_name, "..")) {
            continue;
        }
        dir_second_level_operation(argv[1],entry->d_name);
    }
     closedir(dir);
    return 0;
}