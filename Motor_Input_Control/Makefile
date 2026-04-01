CXX = g++
CXXFLAGS = -Iinclude -Wall -Wextra `sdl2-config --cflags`
LDFLAGS = `sdl2-config --libs`

SRC = src/Main.cpp src/motor.cpp src/ak_motor.cpp src/can_control.cpp
OBJ = $(SRC:.cpp=.o)
TARGET = motor_control

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(OBJ) -o $(TARGET) $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

.PHONY: all clean
