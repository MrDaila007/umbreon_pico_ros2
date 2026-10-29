DC := docker compose -f docker/compose.yml run --rm fw-build
export UID := $(shell id -u)
export GID := $(shell id -g)
HOST_BUILD := build/host
FW_BUILD := build/fw

.PHONY: test coverage fw microros image

image:
	docker compose -f docker/compose.yml build fw-build

test:
	$(DC) bash -c 'set -e; \
	  cmake -S tests/host -B $(HOST_BUILD) -G Ninja >/dev/null; \
	  cmake --build $(HOST_BUILD); \
	  find $(HOST_BUILD) -name "*.gcda" -delete; \
	  rc=0; n=0; \
	  for t in $(HOST_BUILD)/test_*; do \
	    [ -f "$$t" ] && [ -x "$$t" ] || continue; \
	    n=$$((n+1)); echo "== $$t"; "$$t" || rc=1; \
	  done; \
	  [ $$n -gt 0 ] || { echo "no host tests found" >&2; exit 1; }; \
	  exit $$rc'

coverage: test
	$(DC) bash -c 'if ls firmware/core/*.c >/dev/null 2>&1; then \
	  gcovr --root . --filter "firmware/core/.*\.c" --object-directory $(HOST_BUILD) \
	    --txt --fail-under-line 80; \
	  else echo "coverage: no firmware/core/*.c yet, nothing to measure"; fi'

fw:
	$(DC) bash -c 'set -e; \
	  cmake -S firmware -B $(FW_BUILD) -G Ninja >/dev/null; \
	  cmake --build $(FW_BUILD); \
	  test -f $(FW_BUILD)/umbreon_pico_ros2.uf2; \
	  echo "built $(FW_BUILD)/umbreon_pico_ros2.uf2"'

microros:
	./micro_ros/build.sh
