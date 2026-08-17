#!/bin/bash
# Build and run every component test suite.
#
#   ./run-tests.sh            all suites
#   ./run-tests.sh program_state weiler_spindle
#
# Unlike "make", this keeps going after a failing suite and prints a summary at
# the end, so one broken component does not hide the others. Exit code is 0 only
# when every suite passes.

set -uo pipefail
cd "$(dirname "$0")" || exit 1

if [ -t 1 ]; then
    RED=$'\033[31m'; GREEN=$'\033[32m'; BOLD=$'\033[1m'; OFF=$'\033[0m'
else
    RED=''; GREEN=''; BOLD=''; OFF=''
fi

# Suites to run: the ones named on the command line, otherwise everything the
# Makefile knows about.
if [ $# -gt 0 ]; then
    suites=("$@")
else
    mapfile -t suites < <(sed -n 's/^COMPS  *:= *//p' Makefile | tr ' ' '\n' | grep -v '^$')
fi

echo "${BOLD}Building${OFF}"
if ! make -s binaries; then
    echo "${RED}build failed${OFF}"
    exit 1
fi

failed_suites=()
total_pass=0
total_fail=0

for suite in "${suites[@]}"; do
    bin="build/test_${suite}"
    if [ ! -x "$bin" ]; then
        echo "${RED}no such suite: ${suite}${OFF}"
        failed_suites+=("$suite")
        continue
    fi

    echo
    echo "${BOLD}${suite}${OFF}"
    output=$("$bin")
    status=$?
    echo "$output"

    counts=$(echo "$output" | sed -n 's/^\([0-9]*\) passed, \([0-9]*\) failed.*/\1 \2/p')
    total_pass=$((total_pass + ${counts% *}))
    total_fail=$((total_fail + ${counts#* }))

    [ $status -ne 0 ] && failed_suites+=("$suite")
done

echo
if [ ${#failed_suites[@]} -eq 0 ]; then
    echo "${GREEN}${BOLD}ALL GREEN${OFF} - ${total_pass} scenarios in ${#suites[@]} suites"
    exit 0
fi

echo "${RED}${BOLD}FAILED${OFF} - ${total_fail} scenario(s) in: ${failed_suites[*]}"
echo "(${total_pass} passed)"
exit 1
