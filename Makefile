CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wpedantic -g

.PHONY: all clean planner

all: planner

planner:
	$(MAKE) -C planner

clean:
	$(MAKE) -C planner clean
	rm -f plan2exec waypoint2ctrl ctrl_runner *.o compile_commands.json
