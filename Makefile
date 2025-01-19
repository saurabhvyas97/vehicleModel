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

test: test/tireModel_test

test/tireModel_test: test/tireModel_test.c src/tireModel.c include/tireModel.h
	$(CC) -o $@ test/tireModel_test.c src/tireModel.c -Iinclude -lcunit -lm

clean:
	rm -rf build/*
	rm -f $(OBJ) $(TARGET) test/tireModel_test