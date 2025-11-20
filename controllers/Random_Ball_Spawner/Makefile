# Try-to-run Makefile: link libController at runtime, allow undefined lookup
CXX = clang++
CXXFLAGS = -std=c++17 -O2 -Wall

WEBOTS_HOME ?= /Applications/Webots.app

INCLUDE = -I"$(WEBOTS_HOME)/Contents/include/controller/cpp"
LIBDIR  = -L"$(WEBOTS_HOME)/Contents/lib/controller"
RPATH   = -Wl,-rpath,"$(WEBOTS_HOME)/Contents/lib/controller"

# allow unresolved at link time but still link the libController symbol table
LDFLAGS = -Wl,-undefined,dynamic_lookup $(RPATH)

TARGET = random_ball_spawner

all:
	$(CXX) $(CXXFLAGS) $(INCLUDE) random_ball_spawner.cpp $(LIBDIR) -lController $(LDFLAGS) -o $(TARGET)

clean:
	rm -f $(TARGET)