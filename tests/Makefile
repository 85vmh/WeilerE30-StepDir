# Functional tests for the realtime components.
#
#   make            build and run everything
#   make clean
#
# No LinuxCNC runtime is needed: the C body of each .comp is extracted and
# compiled into a plain test binary.

BUILD   := build
CFLAGS  := -std=gnu99 -Wall -Wextra -Wno-unused-parameter -O0 -g -I$(BUILD) -Istubs

COMPS   := program_state weiler_spindle weiler_startup
BINS    := $(addprefix $(BUILD)/test_,$(COMPS))

.PHONY: all binaries clean $(COMPS)
all: $(BINS)
	@for t in $(BINS); do echo "== $$t"; $$t || exit 1; done

# Build the binaries without running them (used by run-tests.sh).
# Not named "build": that is the output directory.
binaries: $(BINS)

# Run a single component's suite, e.g.:  make program_state
$(COMPS): %: $(BUILD)/test_%
	$(BUILD)/test_$@

# Everything after the ';;' line of a .comp is plain C
$(BUILD)/%_body.inc: ../components/%.comp | $(BUILD)
	sed '1,/^;;$$/d' $< > $@

.PRECIOUS: $(BUILD)/%_body.inc

$(BUILD)/test_%: test_%.c $(BUILD)/%_body.inc harness.h | $(BUILD)
	$(CC) $(CFLAGS) -o $@ $<

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
