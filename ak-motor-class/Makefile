CC = gcc
CFLAGS = -Iinclude -Wall -Wextra
SRC = src/main.c src/motor.c src/ak_motor.c
OBJ = $(SRC:.c=.o)
TARGET = motor_control

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

.PHONY: all clean