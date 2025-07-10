# --- Compiler and Flags ---
export PATH := /mingw64/bin:/usr/bin:/bin:$(PATH)

CXX := g++
CXXFLAGS := -std=c++20 -Wall -Wextra -o3 \
            -I./ \
            -Iinclude \
            -Ilib/imgui \
			-Ilib/imgui-sfml \
            -I/mingw64/include/eigen3 \

# --- Linker and Libraries ---
LDFLAGS := -lsfml-window -lsfml-graphics -lsfml-system -lopengl32 -lyaml-cpp

# --- Files and Directories ---
TARGET := Fx2D.exe
BUILD_DIR := build

# Application's source files.
APP_SRCS := src/main.cpp \
            src/core.cpp \
			src/renderer.cpp \
			src/yaml_utils.cpp \
			

# Define the ImGui core and backend source files.
IMGUI_SRCS := lib/imgui/imgui.cpp \
			  lib/imgui/imgui_draw.cpp \
			  lib/imgui/imgui_tables.cpp \
			  lib/imgui/imgui_widgets.cpp \
			  lib/imgui-sfml/imgui-SFML.cpp \


# Combine all source files to be compiled.
SRCS := $(APP_SRCS) $(IMGUI_SRCS)
# Automatically generate object file names from source files.
OBJS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(SRCS))
OBJDIRS := $(sort $(dir $(OBJS)))

# --- Build Rules ---
# Default target: Build the final executable.
all: init $(TARGET)

init:
	@echo "Creating build directories...$(OBJDIRS)"
	@mkdir -p build/lib
	@mkdir -p build/src
	@mkdir -p $(OBJDIRS)

$(TARGET): $(OBJS)
	@echo "Linking..."
	$(CXX) $(OBJS) -o $(TARGET) $(LDFLAGS)
	@echo "Build finished: $(TARGET)"

# Pattern rule for object files.
$(BUILD_DIR)/%.o: %.cpp
	@echo "Creating directory for $@"
	@mkdir -p $(dir $@)
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -c $< -o $@

# --- Cleanup ---
# Removes the final executable and the build directory with all the object files.
clean:
	@echo "Cleaning up..."
	rm -f $(TARGET)
	rm -rf $(BUILD_DIR)

# --- Rebuild Target ---
rebuild: clean all

# --- Debug Build ---
debug: CXXFLAGS += -g -DDEBUG
debug: clean all

.PHONY: all clean
