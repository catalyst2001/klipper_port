#define NOMINMAX
#include "trapq.h"
#include <algorithm>
#include <cmath>

// ========== Move ==========

Move::Move(const Vec3& startPos, const Vec3& endPos,
           double spd, double acc, double maxJV2)
    : start_pos(startPos), end_pos(endPos), speed(spd), accel(acc),
      max_junction_v2(maxJV2)
{
    axes_d = end_pos - start_pos;
    move_d = axes_d.length();

    max_cruise_v2 = speed * speed;

    // Can't start/end faster than what accel allows over the distance
    max_start_v2 = max_cruise_v2;
    max_smoothed_v2 = max_cruise_v2;

    is_kinematic_move = (move_d > 0.000000001);
}

void Move::calcJunctionV2(const Move* prevMove, double junctionDeviation) {
    if (!prevMove || !prevMove->is_kinematic_move || !is_kinematic_move) {
        max_junction_v2 = 0.0;
        return;
    }

    // Junction velocity from junction deviation model (Klipper's approach):
    // v² = junction_deviation * accel * (sin(theta/2) / (1 - sin(theta/2)))
    // Using the dot product of unit vectors to get cos(theta)
    Vec3 prevUnit = prevMove->axes_d * (1.0 / prevMove->move_d);
    Vec3 curUnit = axes_d * (1.0 / move_d);

    double cosTheta = -(prevUnit.dot(curUnit));
    // cos(theta) ranges from -1 (same dir) to 1 (reversal)
    // cos(theta/2) = sqrt((1 + cos(theta))/2)
    // sin(theta/2) = sqrt((1 - cos(theta))/2)

    if (cosTheta >= 0.9999) {
        // Nearly 180° reversal: junction velocity = 0
        max_junction_v2 = 0.0;
        return;
    }

    cosTheta = std::max(-1.0, std::min(1.0, cosTheta));
    double sinThetaD2 = std::sqrt(0.5 * (1.0 - cosTheta));
    double r = junctionDeviation * sinThetaD2 / (1.0 - sinThetaD2);

    // v² = R * accel
    double jv2 = r * accel;

    // Clamp to min of both moves' cruise speeds  
    jv2 = std::min(jv2, max_cruise_v2);
    jv2 = std::min(jv2, prevMove->max_cruise_v2);

    max_junction_v2 = std::max(0.0, jv2);
}

void Move::setJunction(double startV2, double cruiseV2, double endV2) {
    start_v = std::sqrt(startV2);
    cruise_v = std::sqrt(cruiseV2);
    end_v = std::sqrt(endV2);

    // Acceleration phase: v² = v0² + 2*a*d → d = (v² - v0²) / (2*a)
    double inv2a = 1.0 / (2.0 * accel);

    accel_d = (cruiseV2 - startV2) * inv2a;
    decel_d = (cruiseV2 - endV2) * inv2a;
    cruise_d = move_d - accel_d - decel_d;

    // Handle triangle profile (cruise_d < 0)
    if (cruise_d < 0.0) {
        // Reduce cruise velocity so accel_d + decel_d = move_d
        // v_peak² = (v0² + v1² + 2*a*d) / 2
        double peakV2 = (startV2 + endV2 + 2.0 * accel * move_d) * 0.5;
        cruise_v = std::sqrt(std::max(0.0, peakV2));
        accel_d = (peakV2 - startV2) * inv2a;
        decel_d = (peakV2 - endV2) * inv2a;
        cruise_d = 0.0;
    }

    // Compute timing
    // accel: d = v0*t + 0.5*a*t² → t = (v - v0) / a
    accel_t = (accel > 0 && accel_d > 0) ? (cruise_v - start_v) / accel : 0;
    cruise_t = (cruise_v > 0) ? cruise_d / cruise_v : 0;
    decel_t = (accel > 0 && decel_d > 0) ? (cruise_v - end_v) / accel : 0;
}

std::vector<TrapMove> Move::toTrapMoves() const {
    std::vector<TrapMove> result;
    if (move_d < 0.000000001) return result;

    Vec3 axes_r = axes_d * (1.0 / move_d);
    double t = print_time;
    double dist = 0.0;

    // Acceleration phase
    if (accel_t > 0.000000001) {
        TrapMove tm;
        tm.print_time = t;
        tm.move_t = accel_t;
        tm.start_v = start_v;
        tm.half_accel = 0.5 * accel;
        tm.start_pos = {start_pos.x + axes_r.x * dist,
                        start_pos.y + axes_r.y * dist,
                        start_pos.z + axes_r.z * dist};
        tm.axes_r = axes_r;
        result.push_back(tm);
        t += accel_t;
        dist += accel_d;
    }

    // Cruise phase
    if (cruise_t > 0.000000001) {
        TrapMove tm;
        tm.print_time = t;
        tm.move_t = cruise_t;
        tm.start_v = cruise_v;
        tm.half_accel = 0.0;
        tm.start_pos = {start_pos.x + axes_r.x * dist,
                        start_pos.y + axes_r.y * dist,
                        start_pos.z + axes_r.z * dist};
        tm.axes_r = axes_r;
        result.push_back(tm);
        t += cruise_t;
        dist += cruise_d;
    }

    // Deceleration phase
    if (decel_t > 0.000000001) {
        TrapMove tm;
        tm.print_time = t;
        tm.move_t = decel_t;
        tm.start_v = cruise_v;
        tm.half_accel = -0.5 * accel;
        tm.start_pos = {start_pos.x + axes_r.x * dist,
                        start_pos.y + axes_r.y * dist,
                        start_pos.z + axes_r.z * dist};
        tm.axes_r = axes_r;
        result.push_back(tm);
    }

    return result;
}

// ========== TrapQ ==========

void TrapQ::append(const std::vector<TrapMove>& moves) {
    std::lock_guard<std::mutex> lk(m_mutex);
    m_moves.insert(m_moves.end(), moves.begin(), moves.end());
}

std::vector<TrapMove> TrapQ::getAndClear() {
    std::lock_guard<std::mutex> lk(m_mutex);
    std::vector<TrapMove> result;
    std::swap(result, m_moves);
    return result;
}

void TrapQ::purge(double cutoffTime) {
    std::lock_guard<std::mutex> lk(m_mutex);
    m_moves.erase(
        std::remove_if(m_moves.begin(), m_moves.end(),
                        [cutoffTime](const TrapMove& tm) {
                            return (tm.print_time + tm.move_t) < cutoffTime;
                        }),
        m_moves.end());
}
