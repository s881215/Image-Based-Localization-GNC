CXX ?= g++
CFLAGS =`pkg-config --cflags opencv4`
LIBS = `pkg-config --libs opencv4`
INCLUDE= -I../../include

SRCS7 := defRefPtStreamV3.cpp DIP_FUN.cpp findMultiPath_FUN.cpp
OBJS7 := $(SRCS7:.cpp=.o)


all: defRefPtStreamV3

defRefPtStreamV3: $(OBJS7)
	$(CXX) $(CFLAGS)  -o $@ $(OBJS7) $(LIBS)   $(INCLUDE)   -lMVSDK

%.o: %.cpp DIP_FUN.h
	$(CXX) $(CFLAGS) $(INCLUDE) -c $< -o $@

%.o: %.cpp findMultiPath_FUN.h
	$(CXX) $(CFLAGS) $(INCLUDE) -c $< -o $@

clean:
	rm -f *.o 
	rm -f defRefPtStreamV3
