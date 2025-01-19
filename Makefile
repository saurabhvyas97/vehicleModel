CC = gcc
CFLAGS = -Iinclude -lm
SRC = src/tireMode.c main/vehicleModel.c
OBJ = $(SRC:.c=.o)
TARGET = build/vehicleModel

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS)

clean:
	rm -f $(OBJ) $(TARGET)