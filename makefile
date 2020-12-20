BUILD_DIR := build

ifeq ($(OS),Windows_NT)
CMAKE_GENERATOR := "MinGW Makefiles"
else
CMAKE_GENERATOR := "Unix Makefiles"
endif

all:
	cmake . -B$(BUILD_DIR) -G $(CMAKE_GENERATOR)
	make -C $(BUILD_DIR) -j --no-print-directory

clean:
ifeq ($(OS),Windows_NT)
	@if exist $(BUILD_DIR) rmdir /s /q $(BUILD_DIR)
else
	@rm -rf $(BUILD_DIR)
endif

flash erase reset debug:
	make -C $(BUILD_DIR) --no-print-directory $@
