#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <time.h>

// Observation-only kinematics from ONE complete motor report. A zero ID/time
// means unavailable. This never supplies any control or freshness decisions.
struct FeedbackSample {
    float position = 0;
    float velocity = 0;
    uint64_t received_mono_ns = 0;
    uint64_t sample_id = 0;

    void capture(float q, float dq) {
        timespec ts{};
        if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return;
        position = q;
        velocity = dq;
        received_mono_ns = static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
        ++sample_id;
    }
    FeedbackSample relative(float zero, int direction, float wrap, bool absolute) const {
        auto out = *this;
        const float q = (position - zero) * direction + wrap;
        out.position = absolute ? std::fabs(q) : q;
        out.velocity = velocity * direction * (absolute && q < 0 ? -1 : 1);
        return out;
    }
    FeedbackSample normalized(float minimum, float maximum, bool open_at_min) const {
        auto out = *this;
        const float range = maximum - minimum;
        if (!(range > 0)) return {};
        out.position = std::clamp((position - minimum) / range, 0.0f, 1.0f);
        out.velocity = position < minimum || position > maximum ? 0 : velocity / range;
        if (open_at_min) {
            out.position = 1.0f - out.position;
            out.velocity = -out.velocity;
        }
        return out;
    }
};
static_assert(sizeof(FeedbackSample) == 24, "feedback snapshot ABI changed");
