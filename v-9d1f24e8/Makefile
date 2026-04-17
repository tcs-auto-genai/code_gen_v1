# ProbeApp — portable desktop build (g++/clang++)
# From project root: make
# Use forward slashes in sources; works with mingw/msys and WSL.

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -pedantic -I. -ICanConsumer -IProbeComm -IProbeCommVariant -IProbeApp -Iruntime
# Objects first, then libs — required on some platforms; use g++ (not gcc) for the link step.
LDFLAGS  ?= -lstdc++

SRCS := \
	main.cpp \
	ProbeApp/ProbeApp.cpp \
	ProbeComm/ProbeComm.cpp \
	ProbeCommVariant/ProbeCommVariant.cpp \
	CanConsumer/CanConsumer.cpp \
	PlatformStubs.cpp \
	SharedMemStubs.cpp

OBJS := $(SRCS:.cpp=.o)
TARGET := ProbeApp.out

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $(OBJS) $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)
