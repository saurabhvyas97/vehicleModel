CC = gcc
CFLAGS = -Iinclude -lm
SRC = src/tireModel.c main/vehicleModel.c
OBJ = $(patsubst %.c, build/%.o, $(SRC))
TARGET = build/vehicleModel

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

build/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) -c -o $@ $< $(CFLAGS)

clean:
	rm -rf build/*
