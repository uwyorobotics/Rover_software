CXX = g++
CXXFLAGS = -Iinclude -Wall -Wextra `sdl2-config --cflags` -std=c++17 -O2
LDFLAGS = `sdl2-config --libs`

SRC = Main.cpp src/robot_comms.cpp \
		src/motor.cpp src/ak_motor.cpp src/can_control.cpp 

OBJ = $(SRC:.cpp=.o)
TARGET = motor_control

.PHONY: all clean $(TARGET)

all: clean $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(OBJ) -o $(TARGET) $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)
