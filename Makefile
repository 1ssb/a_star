CXX      ?= g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Wpedantic
PYTHON   ?= python3
TARGET   := astar

.PHONY: all run clean test sanitize viz

all: $(TARGET)

$(TARGET): planner.cc
	$(CXX) $(CXXFLAGS) -o $@ $<

# Run planner, then visualize from output files
run: $(TARGET)
	./astar
	@$(PYTHON) visualize.py

# Visualize from existing output files
viz:
	@$(PYTHON) visualize.py --save-only

# Run all 10 gallery test scenarios
test: $(TARGET)
	@bash test_gallery.sh

# Build with AddressSanitizer + UBSan + LeakSan and run tests
sanitize: planner.cc
	$(CXX) -std=c++17 -O1 -g -fsanitize=address,undefined,leak -fno-omit-frame-pointer -Wall -Wextra -Wpedantic -o astar_asan $<
	@echo "--- Running gallery tests under sanitizers ---"
	@for q in gallery/*.cfg; do \
		name=$$(basename "$$q" .cfg); \
		[ "$$name" = "planner" ] && continue; \
		printf "  %-30s " "$$name"; \
		if ./astar_asan --config gallery/planner.cfg --query "$$q" 2>&1 | grep -q "^Path found:"; then \
			echo "CLEAN"; \
		else \
			echo "ISSUE"; \
		fi; \
	done
	@rm -f astar_asan

clean:
	rm -f $(TARGET) astar_asan plan_viz.png controls.txt se2_waypoints.txt
