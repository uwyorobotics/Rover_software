CXX = g++
CXXFLAGS = -Iinclude -Wall -Wextra

SRC = src/main.cpp src/motor.cpp src/ak_motor.cpp
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

