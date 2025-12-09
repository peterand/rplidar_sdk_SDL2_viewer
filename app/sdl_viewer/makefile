#
# Sample SDL2 viewer for RPLIDAR
#
HOME_TREE := ../../

MODULE_NAME := $(notdir $(CURDIR))

include $(HOME_TREE)/mak_def.inc

SDL2_CFLAGS ?= $(shell pkg-config --cflags sdl2 2>/dev/null)
SDL2_LIBS ?= $(shell pkg-config --libs sdl2 2>/dev/null)

CXXSRC += main.cpp
C_INCLUDES += -I$(CURDIR)/../../sdk/include -I$(CURDIR)/../../sdk/src $(SDL2_CFLAGS)
CXXFLAGS += $(SDL2_CFLAGS)

LD_LIBS += -lstdc++ -lpthread -lm $(SDL2_LIBS)

all: build_app

include $(HOME_TREE)/mak_common.inc

clean: clean_app
