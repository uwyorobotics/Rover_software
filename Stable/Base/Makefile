CXX = g++
CXXFLAGS = -Iinclude -Wall -Wextra `sdl2-config --cflags`
LDFLAGS = `sdl2-config --libs`

SRC = ./Main.cpp src/base_comms.cpp src/Joystick.cpp
OBJ = $(SRC:.cpp=.o) 
TARGET = baseStation

.PHONY: all clean $(TARGET)

all: clean $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(OBJ) -o $(TARGET) $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)
