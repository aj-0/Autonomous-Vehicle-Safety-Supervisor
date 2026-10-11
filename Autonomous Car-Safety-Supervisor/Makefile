# AV Safety Supervisor build
#
#   make                 Raspberry Pi 5 / Pi 4 (QNX aarch64le)   -> build/aarch64le/
#   make ARCH=x86_64     QNX x86_64 virtual machine              -> build/x86_64/
#   make test            host unit tests of the decision logic (gcc, no QNX needed)
#   make clean
#
# Run "source ~/qnx800/qnxsdp-env.sh" (Linux) or qnxsdp-env.bat (Windows) first, so qcc is found.

ARCH ?= aarch64le
ifeq ($(ARCH),x86_64)
  QCCV := gcc_ntox86_64
else
  QCCV := gcc_ntoaarch64le
endif

CC      := qcc -V$(QCCV)
CFLAGS  := -std=gnu11 -O2 -g -Wall -Wextra -Ishared -Isafety_supervisor $(EXTRA_CFLAGS)
LDLIBS  := -lm
OUT     := build/$(ARCH)
HOSTCC  ?= gcc

HDRS        := $(wildcard shared/*.h safety_supervisor/*.h)
SHARED      := shared/settings.c shared/vehicle_physics.c shared/sensor_health.c
SHARED_QNX  := shared/qnx_helpers.c shared/event_buffer.c shared/access_control.c
DECISION    := safety_supervisor/decision_core.c safety_supervisor/check_sensors.c                safety_supervisor/check_nav_command.c safety_supervisor/check_obstacle_camera.c                safety_supervisor/watchdog.c safety_supervisor/safety_states.c                safety_supervisor/final_command.c safety_supervisor/timing_stats.c
SUP_QNX     := safety_supervisor/supervisor_main.c safety_supervisor/send_output.c                safety_supervisor/leds_buzzer.c

PROGRAMS := safety_supervisor vehicle_simulator navigation recovery_manager event_logger             operator_cli cpu_stress

all: $(addprefix $(OUT)/,$(PROGRAMS)) $(OUT)/start_all.sh $(OUT)/stop_all.sh

$(OUT):
	mkdir -p $@

$(OUT)/safety_supervisor: $(SUP_QNX) $(DECISION) $(SHARED) $(SHARED_QNX) $(HDRS) | $(OUT)
	$(CC) $(CFLAGS) -o $@ $(filter %.c,$^) $(LDLIBS)

$(OUT)/cpu_stress: tools/cpu_stress.c | $(OUT)
	$(CC) $(CFLAGS) -o $@ $< $(LDLIBS)

# every other program is one folder with one .c file of the same name
define ONE_FILE_PROGRAM
$(OUT)/$(1): $(1)/$(1).c $(SHARED) $(SHARED_QNX) $(HDRS) | $(OUT)
	$$(CC) $$(CFLAGS) -o $$@ $$(filter %.c,$$^) $$(LDLIBS)
endef
$(foreach p,vehicle_simulator navigation recovery_manager event_logger operator_cli,  $(eval $(call ONE_FILE_PROGRAM,$(p))))

$(OUT)/%.sh: tools/%.sh | $(OUT)
	cp $< $@

test:
	mkdir -p build/host
	$(HOSTCC) -std=gnu11 -O0 -g -Wall -Wextra -Ishared -Isafety_supervisor -o build/host/unit_tests 		tests/unit_tests.c $(DECISION) $(SHARED) -lm
	./build/host/unit_tests

clean:
	rm -rf build

.PHONY: all test clean
