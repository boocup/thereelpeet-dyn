RACK_DIR ?= /Users/peterbuchhop/vcv-dev/Rack

FLAGS +=

SOURCES += src/plugin.cpp
SOURCES += src/TheReelPeet.cpp
SOURCES += src/TheReelPeetEvo.cpp

DISTRIBUTABLES += res
# DISTRIBUTABLES += presets
# DISTRIBUTABLES += selections

# Include the VCV Rack plugin Makefile framework
include $(RACK_DIR)/plugin.mk
