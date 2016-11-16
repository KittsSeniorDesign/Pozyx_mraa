CC = g++

TARGET = Pozyx

SRCS = $(TARGET)_core.cpp $(TARGET)_lib.cpp Wire.cpp

ready_to_localize: ready_to_localize.cpp
	$(CC) -o ready_to_localize examples/ready_to_localize/ready_to_localize.cpp $(SRCS)

ready_to_range: ready_to_range.cpp
	$(CC) -o ready_to_range examples/ready_to_range/ready_to_range.cpp $(SRCS)

$(TARGET): $(TARGET)_main.cpp
	$(CC) -o $(TARGET) $(TARGET)_main.cpp 

