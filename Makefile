SRCS := $(wildcard *.cpp)
OBJS := $(patsubst %.cpp, %.o, $(SRCS))

CXXFLAGS += -stdlib=libc++ -std=c++11 -g -O0
#-Wl,-L/Users/tigran/waffles/lib,-lGClasses

CXX := clang++

EXEC := alphabeta

all: $(EXEC)

$(EXEC) : $(OBJS)
		@echo Building parallel, $(CXX) ...
		$(CXX) $(CXXFLAGS) $(OBJS) -o $@

%.o : %.cpp
		$(CXX) -c $(CXXFLAGS) $< -o $@
		@echo Building .o from .cpp $(CXX)

clean:
		rm $(EXEC) *.o

