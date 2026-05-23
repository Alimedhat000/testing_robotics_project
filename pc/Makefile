CC      = gcc
CFLAGS  = -Wall -Wextra -O2
TARGET  = test_full
BUILD   = build
OBJS    = $(BUILD)/test_full.o $(BUILD)/calibrate.o $(BUILD)/color_detect.o $(BUILD)/kinematics.o $(BUILD)/camera.o

$(BUILD)/$(TARGET): $(OBJS)
	mkdir -p $(BUILD)
	$(CC) -o $@ $^ -lm

$(BUILD)/test_full.o: test_full.c calibrate.h color_detect.h kinematics.h camera.h config.h types.h stb_image.h
	mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -c -o $@ test_full.c

$(BUILD)/calibrate.o: calibrate.c calibrate.h config.h types.h
	mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -c -o $@ calibrate.c

$(BUILD)/color_detect.o: color_detect.c color_detect.h config.h types.h
	mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -c -o $@ color_detect.c

$(BUILD)/kinematics.o: kinematics.c kinematics.h config.h
	mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -c -o $@ kinematics.c

$(BUILD)/camera.o: camera.c camera.h stb_image.h stb_image_write.h
	mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -c -o $@ camera.c

clean:
	rm -rf $(BUILD)

run: $(BUILD)/$(TARGET)
	./$(BUILD)/$(TARGET)

.PHONY: clean run
