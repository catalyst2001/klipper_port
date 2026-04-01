#include "input_shaper.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static constexpr double SHAPER_VIBRATION_REDUCTION = 20.0;

const std::vector<ShaperPulse> InputShaper::s_emptyPulses;

// ========== Public API ==========

void InputShaper::setAxisShaper(int axis, ShaperType type, double freq, double dampingRatio) {
    if (axis < 0 || axis > 2) return;

    auto& ax = m_axes[axis];
    ax.type = type;
    ax.freq = freq;
    ax.dampingRatio = dampingRatio;

    if (type == ShaperType::None || freq <= 0.0) {
        ax.active = false;
        ax.pulses.clear();
        return;
    }

    std::vector<double> A, T;
    computeRawPulses(type, freq, dampingRatio, A, T);
    ax.pulses = initPulses(A, T);
    ax.active = !ax.pulses.empty();
}

bool InputShaper::isAxisShaped(int axis) const {
    if (axis < 0 || axis > 2) return false;
    return m_axes[axis].active;
}

const std::vector<ShaperPulse>& InputShaper::getAxisPulses(int axis) const {
    if (axis < 0 || axis > 2) return s_emptyPulses;
    return m_axes[axis].pulses;
}

int InputShaper::getNumPulses(int axis) const {
    if (axis < 0 || axis > 2) return 0;
    return static_cast<int>(m_axes[axis].pulses.size());
}

double InputShaper::getPreActiveTime(int axis) const {
    if (axis < 0 || axis > 2 || !m_axes[axis].active) return 0.0;
    const auto& pulses = m_axes[axis].pulses;
    // Last pulse has the most positive t (pre_active = look ahead)
    return pulses.back().t;
}

double InputShaper::getPostActiveTime(int axis) const {
    if (axis < 0 || axis > 2 || !m_axes[axis].active) return 0.0;
    const auto& pulses = m_axes[axis].pulses;
    // First pulse has the most negative t (post_active = look behind)
    return -pulses.front().t;
}

ShaperType InputShaper::parseType(const std::string& name) {
    std::string lower = name;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    if (lower == "zv")          return ShaperType::ZV;
    if (lower == "mzv")         return ShaperType::MZV;
    if (lower == "zvd")         return ShaperType::ZVD;
    if (lower == "ei")          return ShaperType::EI;
    if (lower == "2hump_ei")    return ShaperType::TWO_HUMP_EI;
    if (lower == "3hump_ei")    return ShaperType::THREE_HUMP_EI;
    return ShaperType::None;
}

std::string InputShaper::typeName(ShaperType type) {
    switch (type) {
        case ShaperType::ZV:           return "zv";
        case ShaperType::MZV:          return "mzv";
        case ShaperType::ZVD:          return "zvd";
        case ShaperType::EI:           return "ei";
        case ShaperType::TWO_HUMP_EI:  return "2hump_ei";
        case ShaperType::THREE_HUMP_EI:return "3hump_ei";
        default:                       return "none";
    }
}

// ========== Raw pulse computation (matching Klipper's shaper_defs.py) ==========

void InputShaper::computeRawPulses(ShaperType type, double freq, double dampingRatio,
                                   std::vector<double>& A, std::vector<double>& T) {
    double df = std::sqrt(1.0 - dampingRatio * dampingRatio);
    double K = std::exp(-dampingRatio * M_PI / df);
    double t_d = 1.0 / (freq * df);

    switch (type) {
    case ShaperType::ZV: {
        A = {1.0, K};
        T = {0.0, 0.5 * t_d};
        break;
    }
    case ShaperType::ZVD: {
        A = {1.0, 2.0 * K, K * K};
        T = {0.0, 0.5 * t_d, t_d};
        break;
    }
    case ShaperType::MZV: {
        double Km = std::exp(-0.75 * dampingRatio * M_PI / df);
        double t_dm = 1.0 / (freq * df);
        double a1 = 1.0 - 1.0 / std::sqrt(2.0);
        double a2 = (std::sqrt(2.0) - 1.0) * Km;
        double a3 = a1 * Km * Km;
        A = {a1, a2, a3};
        T = {0.0, 0.375 * t_dm, 0.75 * t_dm};
        break;
    }
    case ShaperType::EI: {
        double v_tol = 1.0 / SHAPER_VIBRATION_REDUCTION;
        double dr = dampingRatio;

        double a1 = (0.24968 + 0.24961 * v_tol) +
                     ((0.80008 + 1.23328 * v_tol) +
                      (0.49599 + 3.17316 * v_tol) * dr) * dr;
        double a3 = (0.25149 + 0.21474 * v_tol) +
                     ((-0.83249 + 1.41498 * v_tol) +
                      (0.85181 - 4.90094 * v_tol) * dr) * dr;
        double a2 = 1.0 - a1 - a3;

        double t2 = 0.4999 +
                     (((0.46159 + 8.57843 * v_tol) * v_tol) +
                      (((4.26169 - 108.644 * v_tol) * v_tol) +
                       ((1.75601 + 336.989 * v_tol) * v_tol) * dr) * dr) * dr;

        A = {a1, a2, a3};
        T = {0.0, t2 * t_d, t_d};
        break;
    }
    case ShaperType::TWO_HUMP_EI: {
        std::vector<std::vector<double>> tc = {
            {0.0, 0.0, 0.0, 0.0},
            {0.49890, 0.16270, -0.54262, 6.16180},
            {0.99748, 0.18382, -1.58270, 8.17120},
            {1.49920, -0.09297, -0.28338, 1.85710}
        };
        std::vector<std::vector<double>> ac = {
            {0.16054, 0.76699, 2.26560, -1.22750},
            {0.33911, 0.45081, -2.58080, 1.73650},
            {0.34089, -0.61533, -0.68765, 0.42261},
            {0.15997, -0.60246, 1.00280, -0.93145}
        };
        getFromExpansionCoeffs(freq, dampingRatio, tc, ac, A, T);
        break;
    }
    case ShaperType::THREE_HUMP_EI: {
        std::vector<std::vector<double>> tc = {
            {0.0, 0.0, 0.0, 0.0},
            {0.49974, 0.23834, 0.44559, 12.4720},
            {0.99849, 0.29808, -2.36460, 23.3990},
            {1.49870, 0.10306, -2.01390, 17.0320},
            {1.99960, -0.28231, 0.61536, 5.40450}
        };
        std::vector<std::vector<double>> ac = {
            {0.11275, 0.76632, 3.29160, -1.44380},
            {0.23698, 0.61164, -2.57850, 4.85220},
            {0.30008, -0.19062, -2.14560, 0.13744},
            {0.23775, -0.73297, 0.46885, -2.08650},
            {0.11244, -0.45439, 0.96382, -1.46000}
        };
        getFromExpansionCoeffs(freq, dampingRatio, tc, ac, A, T);
        break;
    }
    default:
        A.clear();
        T.clear();
        break;
    }
}

// ========== Pulse initialization (matching Klipper's init_shaper + shift_pulses) ==========

std::vector<ShaperPulse> InputShaper::initPulses(const std::vector<double>& A,
                                                  const std::vector<double>& T) {
    int n = static_cast<int>(A.size());
    if (n <= 0 || n > 5) return {};

    // Normalize amplitudes
    double sumA = 0.0;
    for (double a : A) sumA += a;
    if (std::abs(sumA) < 1e-10) return {};
    double invA = 1.0 / sumA;

    // Reverse pulse order and negate times (matching Klipper's init_shaper)
    std::vector<ShaperPulse> pulses(n);
    for (int i = 0; i < n; i++) {
        pulses[n - i - 1].a = A[i] * invA;
        pulses[n - i - 1].t = -T[i];
    }

    // Shift pulses so that Σ(a_i * t_i) = 0 (identity for constant-speed motion)
    double ts = 0.0;
    for (const auto& p : pulses)
        ts += p.a * p.t;
    for (auto& p : pulses)
        p.t -= ts;

    return pulses;
}

// ========== Polynomial expansion helper for 2HUMP_EI / 3HUMP_EI ==========

void InputShaper::getFromExpansionCoeffs(double freq, double dampingRatio,
                                          const std::vector<std::vector<double>>& tCoeffs,
                                          const std::vector<std::vector<double>>& aCoeffs,
                                          std::vector<double>& A, std::vector<double>& T) {
    double tau = 1.0 / freq;
    int n = static_cast<int>(tCoeffs.size());
    int k = static_cast<int>(tCoeffs[0].size());

    A.resize(n);
    T.resize(n);

    for (int i = 0; i < n; i++) {
        // Horner's method: evaluate polynomial in damping_ratio
        double u = tCoeffs[i][k - 1];
        double v = aCoeffs[i][k - 1];
        for (int j = k - 2; j >= 0; j--) {
            u = u * dampingRatio + tCoeffs[i][j];
            v = v * dampingRatio + aCoeffs[i][j];
        }
        T[i] = u * tau;
        A[i] = v;
    }
}
