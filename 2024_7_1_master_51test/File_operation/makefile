CC = gcc
CFLAGS = -Wall -O2
LDFLAGS =
TARGET = myprogram
SRC = main.c

all: $(TARGET)

$(TARGET): $(SRC:.c=.o)
	$(CC) $(LDFLAGS) $^ -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(TARGET) $(SRC:.c=.o) 
run:
	./myprogram.exe C:\Users\zhujunnan\Desktop\QIRUI\Chery_Calibration_pressure2024_0731_2\Chery_Calibration_pressure2024_0731_2\LogData\OK\2024\8_1