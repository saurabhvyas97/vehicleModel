CC = gcc
CFLAGS = -Iinclude -lm
SRC_DIR = src
MAIN_DIR = main
BUILD_DIR = build
TEST_DIR = test

SRC = $(shell find $(SRC_DIR) -name "*.c") $(shell find $(MAIN_DIR) -name "*.c")
OBJ = $(patsubst $(SRC_DIR)/%, $(BUILD_DIR)/%, $(SRC:.c=.o))
TARGET = $(BUILD_DIR)/vehicleModel
TEST_TARGET = $(TEST_DIR)/tireModel_test

all: $(TARGET)

$(TARGET): $(OBJ)
	@mkdir -p $(BUILD_DIR)
	$(CC) -o $@ $^ $(CFLAGS)

#Build object files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(@D)
	$(CC) -c -o $@ $< $(CFLAGS)

#Test build
test: $(TEST_TARGET)

$(TEST_TARGET): $(TEST_DIR)/tireModel_test.c $(SRC_DIR)/tireModel.c $(INCLUDE_DIR)/tireModel.h
	$(CC) -o $@ $^ -I$(INCLUDE_DIR) -lcunit -lm

clean:
	rm -rf $(BUILD_DIR)/*
	rm -f $(OBJ) $(TARGET) $(TEST_TARGET)