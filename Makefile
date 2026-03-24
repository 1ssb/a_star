CXX      := /opt/homebrew/bin/g++-14
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Wpedantic -D_Alignof=alignof
PYTHON   := $(HOME)/miniconda3/bin/python3
TARGET   := astar

.PHONY: all run clean

all: $(TARGET)

$(TARGET): planner.cc
	$(CXX) $(CXXFLAGS) -o $@ $<

# Single command: build, launch visualizer in background, run planner
run: $(TARGET)
	@$(PYTHON) visualize.py &
	@sleep 0.5
	./astar

clean:
	rm -f $(TARGET) plan_viz.png
