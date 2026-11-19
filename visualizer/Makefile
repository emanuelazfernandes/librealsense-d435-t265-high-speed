CXX = g++
CXXFLAGS = -std=c++17 -O3 -Wall -Wno-deprecated-copy -Wno-ignored-qualifiers -Wno-sign-compare -pthread

LIBREALSENSE_ROOT = ..
INCLUDES = -I$(LIBREALSENSE_ROOT)/include $$(pkg-config --cflags opencv4)

LIBS = $(LIBREALSENSE_ROOT)/build/librealsense2.a \
       $(LIBREALSENSE_ROOT)/build/common/fw/libfw.a \
       $(LIBREALSENSE_ROOT)/build/third-party/realsense-file/librealsense-file.a \
       $$(pkg-config --libs opencv4) \
       -lusb-1.0 -ludev -lpthread

TARGET = visualizer
SRC = visualizer.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(SRC) -o $(TARGET) $(LIBS)

clean:
	rm -f $(TARGET)

.PHONY: all clean
