CXX = g++
CXXFLAGS = -Iinclude -Wall -Wextra

SRC = src/two_main.cpp src/motor.cpp src/ak_motor.cpp src/can_control.cpp
OBJ = $(SRC:.cpp=.o)
TARGET = motor_control

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(OBJ) -o $(TARGET)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

.PHONY: all clean

