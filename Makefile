CXX := g++
CXXFLAGS := -std=c++0x -Wall

INC_DIR = inc
SRC_DIR = src
OBJ_DIR = obj

TARGET      := main
SOURCES_CXX := $(wildcard $(SRC_DIR)/*.cpp)
OBJS_CXX    := $(SOURCES_CXX:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
INCS        := $(wildcard $(INC_DIR)/*.h)

$(TARGET): $(OBJS_CXX)
	$(CXX) -o $(TARGET) $(OBJS_CXX) 

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp $(INCS)
	$(CXX) -c -o $@ $< -I$(INC_DIR) $(CXXFLAGS)

clean:
	$(RM) $(TARGET) $(OBJS_CXX) 

run:
	./$(TARGET)

