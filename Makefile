##아직 수정중
#README.md 참고

##########################
# MACRO
##########################

program := sim

sources := $(wildcard *.cpp)
objects := $(subst .cpp,.o,$(sources))

dram_library := ./DRAMSim2/libdramsim.so
LDLIBS := -ldramsim

CXXFLAGS += -std=c++11 -Wall -g
LDFLAGS += -L$(dir $(dram_library))
include_dir := -soname

CXX := g++
RM := rm -f

###########################
# TEST MACRO
###########################

test := run
test_sources := IniParser.cpp DataReader.cpp run.cpp BufferInterface.cpp#write down test sources
test_objects := $(subst .cpp,.o,$(test_sources))


DRAMSim_FLAGS := -ldramsim \
				-L$(dir $(dram_library)) \
				-I$(dir $(dram_library)) \
				-Wl,-rpath=$(dir $(dram_library))

nclude_dir := -I$(dir $(dram_library))

#########################
# rules
#########################

.PHONY: all
all: $(program)

$(test): $(dram_library) $(test_objects)
	$(LINK.cpp) -o $@ $(test_objects) $(LDLIBS) -I$(dir $(dram_library)) -Wl,-rpath=$(dir $(dram_library))

$(program): $(dram_library) $(objects)
	$(LINK.cc) -o $@ $(objects) $(LDLIBS) -I$(dir $(dram_library)) -Wl,-rpath=$(dir $(dram_library))

$(dram_library):
	$(MAKE) --directory=$(dir $@) $(notdir $@)

%.o: %.cpp
	$(COMPILE.cpp) $< -I$(dir $(dram_library))

.PHONY: clean
clean:
	$(MAKE) --directory=$(dir $(dram_library)) clean
	$(RM) $(objects) $(program)
	$(RM) $(test_objects) $(test)


#############################
# 참고
#############################
# LINK.o   = $(CC) $(LDFLAGS) $(TARGET_ARCH)
# LINK.cc  = $(CXX) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(TARGET_ARCH)
# LINK.C   = $(LINK.cc)
# LINK.cpp = $(LINK.cc)

# Or for compiling:

# COMPILE.cc  = $(CXX) $(CXXFLAGS) $(CPPFLAGS) $(TARGET_ARCH) -c
# COMPILE.C   = $(COMPILE.cc)
# COMPILE.cpp = $(COMPILE.cc)
