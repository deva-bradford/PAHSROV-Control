SRC = joy.cpp updates.cpp control.cpp
OBJ = $(SRC:.cpp=.o)

CC = g++
LINK = g++

CFLAGS = -g -O3 -Wall -Wextra -Wpedantic -std=c++11 -lwiringPi -lwiringPiPca9685
CXXFLAGS = $(CFLAGS)

TARGET = control

all: $(TARGET)

$(TARGET): $(OBJ)
	$(LINK) -o $@ $^ $(CFLAGS)

.cpp:
	$(CC) -o $@ $@.cpp

clean:
	rm -rf *.o *.so *.gch *.orig $(TARGET)

format:
	astyle --indent=spaces --style=allman *.h *.cpp
