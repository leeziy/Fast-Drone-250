#pragma once

#include <sys/syscall.h>
#include <unistd.h>

namespace ego_planner
{
class TraceMarkerScope
{
public:
    explicit TraceMarkerScope(int start) : _start(start) { emit(_start); }
    ~TraceMarkerScope() { emit(_start + 1); }

    TraceMarkerScope(const TraceMarkerScope &) = delete;
    TraceMarkerScope &operator=(const TraceMarkerScope &) = delete;

private:
    static void emit(int marker)
    {
        syscall(SYS_kill, marker, 0);
    }

    const int _start;
};
} // namespace ego_planner
