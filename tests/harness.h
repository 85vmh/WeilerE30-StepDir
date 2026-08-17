/* Shared plumbing for the component tests.
 *
 * A test file includes this header, declares the component's pins and params as
 * globals with the same names, then includes the C body extracted from the .comp
 * (everything after the ';;' line). The real logic is compiled into the test, so
 * the tests cannot drift from the component.
 *
 * fperiod is fixed at 1 ms, so a "tick" is one servo cycle and every scenario is
 * deterministic. Each scenario runs in its own forked process, because the
 * components keep their state in file-static variables that cannot be reset from
 * the outside.
 */

#ifndef HARNESS_H
#define HARNESS_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>

/* ===== HAL types the component bodies expect ===== */
typedef int   hal_bit_t;
typedef int   hal_s32_t;
typedef float hal_float_t;

#define FUNCTION(name) void comp_run(void)
void comp_run(void);

static double fperiod = 0.001;   /* 1 ms servo period */

static int failures;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            printf("    line %-4d FAIL  %s\n", __LINE__, #cond);               \
            failures++;                                                        \
        }                                                                      \
    } while (0)

static inline void tick(int n)
{
    while (n-- > 0) comp_run();
}

struct scenario {
    const char *name;
    void      (*fn)(void);
};

static inline int run_scenarios(const struct scenario *list, int count)
{
    int passed = 0, failed = 0;

    for (int i = 0; i < count; i++) {
        printf("  %-42s ", list[i].name);
        fflush(stdout);

        pid_t pid = fork();
        if (pid == 0) {
            printf("\n");
            failures = 0;
            list[i].fn();
            fflush(stdout);          /* _exit does not flush stdio */
            _exit(failures ? 1 : 0);
        }

        int status = 0;
        waitpid(pid, &status, 0);
        if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
            printf("ok\n");
            passed++;
        } else {
            printf("    -> FAILED\n");
            failed++;
        }
    }

    printf("\n%d passed, %d failed, %d total\n", passed, failed, count);
    return failed ? 1 : 0;
}

#define RUN_SCENARIOS(list) \
    run_scenarios((list), (int)(sizeof(list) / sizeof((list)[0])))

#endif /* HARNESS_H */
