include ../common.mk

# flags for building the gtk library
CXXFLAGS = $(CXXFLAGS_STD) \
	$(CFLAGS_MATH) \
	$(CFLAGS_COMMON) \
	$(CFLAGS_LCMTYPES) \
	$(CFLAGS_LCM) \
	$(CFLAGS_GTK) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_GSL) \
	-O3

LDFLAGS = $(LDFLAGS_STD) \
      $(LDFLAGS_MATH) \
      $(LDFLAGS_COMMON) \
      $(LDFLAGS_LCMTYPES) \
      $(LDFLAGS_LCM) \
      $(LDFLAGS_GSL) \
      $(LDFLAGS_MAPPING)

LIBDEPS = $(call libdeps, $(LDFLAGS))

LIB_MAPPING = $(LIB_PATH)/libmapping.a
LIBMAPPING_OBJS = occupancy_grid.o

BIN_SLAM = $(BIN_PATH)/slam

BIN_TEST = $(BIN_PATH)/test

ALL = $(LIB_MAPPING) $(BIN_SLAM)
TEST = $(BIN_TEST)

all: $(ALL)

test: $(TEST)

$(BIN_SLAM): slam_main.o moving_laser_scan.o mapping.o slam.o action_model.o particle_filter.o sensor_model.o occupancy_grid_utils.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS) $(CXXFLAGS)

$(LIB_MAPPING): $(LIBMAPPING_OBJS)
	@echo "    $@"
	@ar rc $@ $^

$(BIN_TEST): occupancy_grid_utils.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS) $(CXXFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
