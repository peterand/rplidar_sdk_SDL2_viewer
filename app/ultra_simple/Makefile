SDK_ROOT := ../..

SDK_LIB  := $(SDK_ROOT)/output/Linux/Release/libsl_lidar_sdk.a

PKG_CONFIG ?= pkg-config
HAVE_SDL2  := $(shell $(PKG_CONFIG) --exists sdl2 && echo 1 || echo 0)
SDL_CFLAGS := $(shell $(PKG_CONFIG) --cflags sdl2 2>/dev/null)
SDL_LIBS   := $(shell $(PKG_CONFIG) --libs sdl2 2>/dev/null)

CXX      := g++
CXXFLAGS := -std=c++11 -O2 \
            -I$(SDK_ROOT)/sdk/include \
            -I$(SDK_ROOT)/sdk/src

ifeq ($(HAVE_SDL2),1)
    CXXFLAGS += $(SDL_CFLAGS) -DHAVE_SDL2=1
    SDL_LDFLAGS := $(SDL_LIBS)
else
    CXXFLAGS += -DHAVE_SDL2=0
    SDL_LDFLAGS :=
endif

LDFLAGS  := $(SDK_LIB) $(SDL_LDFLAGS) -lm -lpthread

TARGET   := ultra_simple
SRC      := main.cpp
OBJ      := $(SRC:.cpp=.o)
.PHONY: all clean sdk

all: $(TARGET)

$(TARGET): $(SDK_LIB) $(OBJ)
	$(CXX) -o $@ $(OBJ) $(LDFLAGS)

$(SDK_LIB):
	$(MAKE) -C $(SDK_ROOT)/sdk

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(TARGET) $(OBJ)

