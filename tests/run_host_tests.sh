#!/bin/sh
set -eu
repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
test_dir=$(mktemp -d "${TMPDIR:-/tmp}/car-host-tests.XXXXXX")
trap 'rm -rf "$test_dir"' EXIT HUP INT TERM
cc -std=c11 -Wall -Wextra -I "$repo_dir/tests/stubs" -I "$repo_dir/Core/code" \
  "$repo_dir/tests/track_nav_host_test.c" -lm -o "$test_dir/track_nav_test"
"$test_dir/track_nav_test"
cc -std=c11 -Wall -Wextra -I "$repo_dir/tests/stubs" -I "$repo_dir/Core/code" \
  "$repo_dir/tests/pid_helper_host_test.c" "$repo_dir/Core/code/pid_helper_app.c" \
  "$repo_dir/Core/code/pid_helper_protocol.c" -lm -o "$test_dir/pid_helper_test"
"$test_dir/pid_helper_test"
cc -std=c11 -Wall -Wextra -I "$repo_dir/tests/stubs" \
  "$repo_dir/tests/emergency_stop_test.c" -o "$test_dir/emergency_stop_test"
"$test_dir/emergency_stop_test"
