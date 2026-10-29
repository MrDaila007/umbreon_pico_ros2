DC := docker compose -f docker/compose.yml run --rm fw-build
export UID := $(shell id -u)
export GID := $(shell id -g)
HOST_BUILD := build/host

.PHONY: test coverage fw microros image

image:
	docker compose -f docker/compose.yml build fw-build

test:
	$(DC) bash -c 'cmake -S tests/host -B $(HOST_BUILD) -G Ninja >/dev/null \
	  && cmake --build $(HOST_BUILD) \
	  && find $(HOST_BUILD) -name "*.gcda" -delete \
	  && rc=0; for t in $(HOST_BUILD)/test_*; do [ -x "$$t" ] && { echo "== $$t"; "$$t" || rc=1; }; done; exit $$rc'

coverage: test
	$(DC) bash -c 'if ls firmware/core/*.c >/dev/null 2>&1; then \
	  gcovr --root . --filter "firmware/core/.*\.c" --object-directory $(HOST_BUILD) \
	    --txt --fail-under-line 80; \
	  else echo "coverage: no firmware/core/*.c yet, nothing to measure"; fi'

fw:
	@echo "make fw: not implemented yet (Task 12)"; exit 1

microros:
	./micro_ros/build.sh
