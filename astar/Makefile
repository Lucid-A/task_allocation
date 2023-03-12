# 1950053 自动化 赵浪舟

CC := g++ -g 

CPP_STD_VER := c++17

SRC_DIR := ./src
TEST_DIR := ./test
INCLUDE_DIR := ./include

TEMP_DIR := ./temp
EXEC_DIR := ./exec

OPENCV_INCLUDE := `pkg-config opencv4 --cflags`
OPENCV_LIBS := `pkg-config opencv4 --libs`

OBJS :=
EXES :=
TEST_OBJS :=
TEST_EXES :=
CLEAN_LIST = $(OBJS) $(EXES) $(TEST_OBJS) $(TEST_EXES)



.PHONY: default_target
default_target: test_opencv

NAME := pgm
DEPEND := 

TEST_SRC := $(TEST_DIR)/$(NAME:%=test_%.cpp)
TEST_OBJ := $(TEMP_DIR)/$(NAME:%=test_%.o)
TEST_EXE := $(EXEC_DIR)/$(NAME:%=test_%)

TEST_EXES += $(TEST_EXE)
TEST_OBJS += $(TEST_OBJ)

$(TEST_OBJ) : $(TEST_SRC) $(DEPEND)
	$(CC) $(OPENCV_INCLUDE) -c $^ -o $@

$(TEST_EXE) : $(TEST_OBJ)
	$(CC) $^ $(OPENCV_LIBS) -o $@

NAME := astar
DEPEND := 

SRC := $(SRC_DIR)/$(NAME:%=%.cpp)
OBJ := $(TEMP_DIR)/$(NAME:%=%.o)

OBJS += $(TEST_OBJ)

$(OBJ) : $(SRC) $(DEPEND)
	$(CC) -I$(INCLUDE_DIR) $(OPENCV_INCLUDE) -c $^ -o $@

NAME := SA
DEPEND := 

SRC := $(SRC_DIR)/$(NAME:%=%.cpp)
OBJ := $(TEMP_DIR)/$(NAME:%=%.o)

TEST_SRC := $(TEST_DIR)/$(NAME:%=test_%.cpp)
TEST_OBJ := $(TEMP_DIR)/$(NAME:%=test_%.o)
TEST_EXE := $(EXEC_DIR)/$(NAME:%=test_%)

TEST_EXES += $(TEST_EXE)
TEST_OBJS += $(TEST_OBJ)

$(OBJ) : $(SRC) $(DEPEND)
	$(CC) -I$(INCLUDE_DIR) $(OPENCV_INCLUDE) -c $^ -o $@

$(TEST_OBJ) : $(TEST_SRC) $(DEPEND)
	$(CC) -I$(INCLUDE_DIR) $(OPENCV_INCLUDE) -c $^ -o $@

$(TEST_EXE) : $(TEST_OBJ) $(OBJ) $(TEMP_DIR)/astar.o
	$(CC) $^ $(OPENCV_LIBS) -o $@



test_opencv.o  : test_opencv.cpp
	g++ `pkg-config opencv4 --cflags` -c $^ -o $@

test_opencv    : test_opencv.o
	g++ $^ `pkg-config opencv4 --libs` -o $@

.PHONY  : clean
clean   :
	rm $(CLEAN_LIST) test_opencv.o test_opencv
