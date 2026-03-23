CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wpedantic -g

SRCS     := plan2exec.cc waypoint2ctrl.cc ctrl_runner.cc
TARGETS  := plan2exec waypoint2ctrl ctrl_runner
OBJS     := $(SRCS:.cc=.o)

.PHONY: all clean

all: $(TARGETS) compile_commands.json

# ── Link targets ────────────────────────────────────────────
plan2exec: plan2exec.o
	$(CXX) $(CXXFLAGS) -o $@ $<

waypoint2ctrl: waypoint2ctrl.o
	$(CXX) $(CXXFLAGS) -o $@ $<

ctrl_runner: ctrl_runner.o
	$(CXX) $(CXXFLAGS) -o $@ $<

# ── Compile rule ────────────────────────────────────────────
%.o: %.cc
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ── Auto-generate compile_commands.json for clangd ─────────
compile_commands.json: $(SRCS)
	@printf '[\n' > $@
	@sep=""; \
	for src in $(SRCS); do \
	  printf '%s  {\n    "directory": "%s",\n    "file": "%s/%s",\n    "command": "%s %s -c -o %s %s"\n  }' \
	    "$$sep" "$(CURDIR)" "$(CURDIR)" "$$src" \
	    "$(CXX)" "$(CXXFLAGS)" "$${src%.cc}.o" "$$src" >> $@; \
	  sep=",\n"; \
	done
	@printf '\n]\n' >> $@

clean:
	rm -f $(OBJS) $(TARGETS) compile_commands.json
