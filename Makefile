TARGET ?= hik_capture
BUILD_DIR ?= build
SRCS := src/main.cpp src/hik_capture.cpp src/app_options.cpp src/debug_draw.cpp src/vision_pipeline.cpp src/armor_preprocessor.cpp src/light_bar_filter.cpp src/armor_matcher.cpp src/pnp_solver.cpp
HEADERS := include/hik_capture.hpp include/app_options.hpp include/debug_draw.hpp include/vision_pipeline.hpp include/armor_preprocessor.hpp include/armor_types.hpp include/light_bar_filter.hpp include/armor_matcher.hpp include/pnp_solver.hpp

CXX ?= g++
UNAME_M := $(shell uname -m)

MVCAM_ROOT ?= /opt/MVS
MVCAM_INCLUDE ?= $(MVCAM_ROOT)/include
ifeq ($(strip $(MVCAM_COMMON_RUNENV)),)
MVCAM_LIB_DIR ?= $(MVCAM_ROOT)/lib/$(UNAME_M)
else
MVCAM_LIB_DIR ?= $(MVCAM_COMMON_RUNENV)/$(UNAME_M)
endif

OPENCV_CFLAGS ?= $(shell pkg-config --cflags opencv4 2>/dev/null)
OPENCV_LIBS ?= $(shell pkg-config --libs opencv4 2>/dev/null)

CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic
CPPFLAGS += -Iinclude -I$(MVCAM_INCLUDE) $(OPENCV_CFLAGS)
LDFLAGS += -L$(MVCAM_LIB_DIR) -Wl,-rpath,$(MVCAM_LIB_DIR)
LDLIBS += -lMvCameraControl $(OPENCV_LIBS)

.PHONY: all clean print-config

all: $(BUILD_DIR)/$(TARGET)

$(BUILD_DIR)/$(TARGET): $(SRCS) $(HEADERS)
	mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(SRCS) -o $@ $(LDFLAGS) $(LDLIBS)

print-config:
	@echo "CXX=$(CXX)"
	@echo "MVCAM_INCLUDE=$(MVCAM_INCLUDE)"
	@echo "MVCAM_LIB_DIR=$(MVCAM_LIB_DIR)"
	@echo "OPENCV_CFLAGS=$(OPENCV_CFLAGS)"
	@echo "OPENCV_LIBS=$(OPENCV_LIBS)"

clean:
	rm -rf $(BUILD_DIR)
