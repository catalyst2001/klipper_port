#pragma once

#include <vector>
#include <array>
#include <cstdint>
#include <cmath>
#include <mutex>

// 3D position
struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    double length() const { return std::sqrt(x * x + y * y + z * z); }
};

// TrapMove: a single sub-move (acceleration, cruise, or deceleration phase).
// Represents a segment of constant acceleration where position is:
//   pos(t) = start_pos + axes_r * (start_v * t + half_accel * t^2)
// for t in [0, move_t].
struct TrapMove {
    double print_time = 0;     // start time (in print_time domain)
    double move_t = 0;         // duration of this sub-move
    double start_v = 0;        // velocity at start of sub-move
    double half_accel = 0;     // 0.5 * acceleration
    Vec3 start_pos;            // position at t=0
    Vec3 axes_r;               // unit direction (axes_d / move_d)

    // Get position at time offset dt from print_time
    Vec3 getPos(double dt) const {
        double d = start_v * dt + half_accel * dt * dt;
        return {start_pos.x + axes_r.x * d,
                start_pos.y + axes_r.y * d,
                start_pos.z + axes_r.z * d};
    }

    // Get scalar distance at time offset dt
    double getDist(double dt) const {
        return start_v * dt + half_accel * dt * dt;
    }

    // Get velocity at time offset dt
    double getVelocity(double dt) const {
        return start_v + 2.0 * half_accel * dt;
    }
};

// Move: a planned motion from start_pos to end_pos.
// After junction calculation + lookahead, it gets split into TrapMoves.
// Port of Klipper's Move class from toolhead.py.
class Move {
public:
    Move() = default;
    Move(const Vec3& startPos, const Vec3& endPos,
         double speed, double accel, double junctionDev = 0.0,
         double mcrPseudoAccel = 0.0);

    Vec3 start_pos;
    Vec3 end_pos;
    Vec3 axes_d;               // end_pos - start_pos
    double move_d = 0;         // total distance
    double speed = 0;          // cruise speed (mm/s)
    double accel = 0;          // acceleration (mm/s^2)
    double junction_deviation = 0; // per-move junction deviation

    // Velocity squared tracking (Klipper convention)
    double max_start_v2 = 0;   // max allowed start velocity^2
    double max_cruise_v2 = 0;  // speed^2
    double delta_v2 = 0;       // 2 * move_d * accel (max v^2 change)
    double next_junction_v2 = 999999999.9; // limit on next move's junction

    // Minimum cruise ratio (MCR) tracking
    double max_mcr_start_v2 = 0;
    double mcr_delta_v2 = 0;   // 2 * move_d * mcr_pseudo_accel

    double min_move_t = 0;     // move_d / velocity (minimum time at cruise)

    // Junction results (filled by lookahead flush)
    double start_v = 0;
    double cruise_v = 0;
    double end_v = 0;

    // Trapezoid timing (filled by set_junction)
    double accel_t = 0;
    double cruise_t = 0;
    double decel_t = 0;
    double accel_d = 0;        // distance during accel
    double cruise_d = 0;       // distance during cruise
    double decel_d = 0;        // distance during decel

    // Timing in print_time domain
    double print_time = 0;

    bool is_kinematic_move = false;

    // Calculate junction velocity using centripetal + junction deviation model
    // Port of Klipper's Move.calc_junction()
    void calcJunction(const Move* prevMove);

    // Set start/cruise/end velocities and compute trapezoid timing
    void setJunction(double startV2, double cruiseV2, double endV2);

    // Split this move into TrapMoves (accel + cruise + decel)
    std::vector<TrapMove> toTrapMoves() const;
};

// Lookahead constants (from Klipper's toolhead.py)
constexpr double LOOKAHEAD_FLUSH_TIME = 0.150; // seconds

// TrapQ: queue of TrapMoves for step generation.
// Steppers consume TrapMoves to generate step times.
class TrapQ {
public:
    TrapQ() = default;

    // Append sub-moves from a processed Move
    void append(const std::vector<TrapMove>& moves);

    // Get all pending trap moves (for step generation)
    std::vector<TrapMove> getAndClear();

    // Peek at moves without clearing
    const std::vector<TrapMove>& peek() const { return m_moves; }

    // Remove moves older than cutoff_time
    void purge(double cutoffTime);

    bool empty() const { return m_moves.empty(); }
    size_t size() const { return m_moves.size(); }

private:
    std::vector<TrapMove> m_moves;
    mutable std::mutex m_mutex;
};
