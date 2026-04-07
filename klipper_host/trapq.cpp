#define NOMINMAX
#include "trapq.h"
#include <algorithm>
#include <cmath>

// ========== Move ==========

Move::Move(const Vec3& startPos, const Vec3& endPos,
           double spd, double acc, double junctionDev, double mcrPseudoAccel)
    : start_pos(startPos), end_pos(endPos), speed(spd), accel(acc),
      junction_deviation(junctionDev)
{
    axes_d = end_pos - start_pos;
    move_d = axes_d.length();

    max_cruise_v2 = speed * speed;
    delta_v2 = 2.0 * move_d * accel;
    mcr_delta_v2 = 2.0 * move_d * mcrPseudoAccel;

    max_start_v2 = 0.0;
    max_mcr_start_v2 = 0.0;

    is_kinematic_move = (move_d > 0.000000001);
    min_move_t = is_kinematic_move ? (move_d / speed) : 0.0;
}

// Port of Klipper's Move.calc_junction() from toolhead.py
// Computes max_start_v2 using junction deviation + centripetal velocity model
void Move::calcJunction(const Move* prevMove) {
    if (!prevMove || !prevMove->is_kinematic_move || !is_kinematic_move) {
        max_start_v2 = 0.0;
        max_mcr_start_v2 = 0.0;
        return;
    }

    // Initial constraint: can't exceed either move's cruise speed,
    // previous move's next_junction limit, or what prev could accelerate to
    double max_start = std::min({max_cruise_v2,
                                 prevMove->max_cruise_v2,
                                 prevMove->next_junction_v2,
                                 prevMove->max_start_v2 + prevMove->delta_v2});

    // Find max velocity using "approximated centripetal velocity"
    // See Klipper's toolhead.py Move.calc_junction() for derivation
    Vec3 prevUnit = prevMove->axes_d * (1.0 / prevMove->move_d);
    Vec3 curUnit = axes_d * (1.0 / move_d);

    // junction_cos_theta = -(dot product of direction vectors)
    double junction_cos_theta = -(curUnit.dot(prevUnit));

    double sin_theta_d2 = std::sqrt(std::max(0.5 * (1.0 - junction_cos_theta), 0.0));
    double cos_theta_d2 = std::sqrt(std::max(0.5 * (1.0 + junction_cos_theta), 0.0));
    double one_minus_sin_theta_d2 = 1.0 - sin_theta_d2;

    if (one_minus_sin_theta_d2 > 0.0 && cos_theta_d2 > 0.0) {
        double R_jd = sin_theta_d2 / one_minus_sin_theta_d2;

        // Junction deviation velocity for this move and previous move
        double move_jd_v2 = R_jd * junction_deviation * accel;
        double pmove_jd_v2 = R_jd * prevMove->junction_deviation * prevMove->accel;

        // Centripetal velocity limit — approximated circle must contact
        // moves no further than mid-move
        double quarter_tan_theta_d2 = 0.25 * sin_theta_d2 / cos_theta_d2;
        double move_centripetal_v2 = delta_v2 * quarter_tan_theta_d2;
        double pmove_centripetal_v2 = prevMove->delta_v2 * quarter_tan_theta_d2;

        // Apply all 4 limits
        max_start = std::min({max_start,
                              move_jd_v2, pmove_jd_v2,
                              move_centripetal_v2, pmove_centripetal_v2});
    }
    // else: colinear or ~180° — no geometric constraint, max_start
    // stays at cruise/reachability limit from above

    max_start_v2 = std::max(0.0, max_start);

    // MCR tracking
    max_mcr_start_v2 = std::min(max_start_v2,
        prevMove->max_mcr_start_v2 + prevMove->mcr_delta_v2);
}

void Move::setJunction(double startV2, double cruiseV2, double endV2) {
    // Port of Klipper's Move.set_junction() from toolhead.py
    // Determine accel, cruise, and decel portions of the move distance
    double half_inv_accel = 0.5 / accel;

    accel_d = (cruiseV2 - startV2) * half_inv_accel;
    decel_d = (cruiseV2 - endV2) * half_inv_accel;
    cruise_d = move_d - accel_d - decel_d;

    // Determine move velocities
    start_v = std::sqrt(startV2);
    cruise_v = std::sqrt(cruiseV2);
    end_v = std::sqrt(endV2);

    // Determine time spent in each portion of move (time is the
    // distance divided by average velocity) — matches Klipper exactly
    accel_t = (start_v + cruise_v > 0.0) ? accel_d / ((start_v + cruise_v) * 0.5) : 0.0;
    cruise_t = (cruise_v > 0.0) ? cruise_d / cruise_v : 0.0;
    decel_t = (end_v + cruise_v > 0.0) ? decel_d / ((end_v + cruise_v) * 0.5) : 0.0;
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

void TrapQ::prepend(const std::vector<TrapMove>& moves) {
    std::lock_guard<std::mutex> lk(m_mutex);
    m_moves.insert(m_moves.begin(), moves.begin(), moves.end());
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
