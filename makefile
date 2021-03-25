BUILD_DIR ?= build
BUILD_TYPE ?= Debug

ifeq ($(OS),Windows_NT)
CMAKE_GENERATOR ?= "MinGW Makefiles"
else
CMAKE_GENERATOR ?= "Unix Makefiles"
NUMBER_OF_PROCESSORS = $(shell grep -c ^processor /proc/cpuinfo)
endif

all:
	cmake . -B$(BUILD_DIR) -G $(CMAKE_GENERATOR) -DCMAKE_BUILD_TYPE=$(BUILD_TYPE)
	cmake --build $(BUILD_DIR) -j $(NUMBER_OF_PROCESSORS)

clean:
ifeq ($(OS),Windows_NT)
	@if exist $(BUILD_DIR) rmdir /s /q $(BUILD_DIR)
else
	@rm -rf $(BUILD_DIR)
endif

flash erase reset debug:
	cmake --build $(BUILD_DIR) --target $@
