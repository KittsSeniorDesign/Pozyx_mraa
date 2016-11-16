CC = g++
CFLAGS = -std=c++11 

TARGET = Pozyx

SRCS = $(TARGET)_core.cpp $(TARGET)_lib.cpp Wire.cpp

ready_to_localize: examples/ready_to_localize/ready_to_localize.cpp
	$(CC) $(CFLAGS) -o ready_to_localize examples/ready_to_localize/ready_to_localize.cpp $(SRCS)

ready_to_range: ready_to_range.cpp
	$(CC) $(CFLAGS) -o ready_to_range examples/ready_to_range/ready_to_range.cpp $(SRCS)

$(TARGET): $(TARGET)_main.cpp
	$(CC) $(CFLAGS) -o $(TARGET) $(TARGET)_main.cpp 

