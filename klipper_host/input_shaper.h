#pragma once

#include <vector>
#include <string>
#include <cmath>

// Shaper pulse: a single (time_offset, amplitude) pair
struct ShaperPulse {
    double t;  // time offset (seconds, typically <= 0)
    double a;  // amplitude weight
};

// Supported input shaper types
enum class ShaperType {
    None,
    ZV,
    MZV,
    ZVD,
    EI,
    TWO_HUMP_EI,
    THREE_HUMP_EI
};

// Per-axis input shaper configuration and computed pulses
class InputShaper {
public:
    InputShaper() = default;

    // Configure shaper for an axis (0=X, 1=Y, 2=Z)
    void setAxisShaper(int axis, ShaperType type, double freq, double dampingRatio = 0.1);

    // Check if an axis has shaping enabled
    bool isAxisShaped(int axis) const;

    // Get the computed & initialized pulses for an axis
    const std::vector<ShaperPulse>& getAxisPulses(int axis) const;

    // Number of pulses for an axis
    int getNumPulses(int axis) const;

    // Get current axis shaper configuration values.
    ShaperType getAxisType(int axis) const {
        return (axis >= 0 && axis <= 2) ? m_axes[axis].type : ShaperType::None;
    }
    double getAxisFrequency(int axis) const {
        return (axis >= 0 && axis <= 2) ? m_axes[axis].freq : 0.0;
    }
    double getAxisDampingRatio(int axis) const {
        return (axis >= 0 && axis <= 2) ? m_axes[axis].dampingRatio : 0.1;
    }

    // Get the time window the shaper needs to look into the past (positive value)
    double getPreActiveTime(int axis) const;

    // Get the time window the shaper needs to look into the future (positive value)
    double getPostActiveTime(int axis) const;

    // Parse shaper type from string name (e.g., "mzv", "ei", "2hump_ei")
    static ShaperType parseType(const std::string& name);

    // Get string name from shaper type
    static std::string typeName(ShaperType type);

private:
    struct AxisShaper {
        ShaperType type = ShaperType::None;
        double freq = 0.0;
        double dampingRatio = 0.1;
        std::vector<ShaperPulse> pulses;
        bool active = false;
    };

    AxisShaper m_axes[3];
    static const std::vector<ShaperPulse> s_emptyPulses;

    // Compute raw (A[], T[]) coefficients for a shaper type
    static void computeRawPulses(ShaperType type, double freq, double dampingRatio,
                                 std::vector<double>& A, std::vector<double>& T);

    // Initialize pulses: normalize, reverse, shift (matching Klipper's init_shaper + shift_pulses)
    static std::vector<ShaperPulse> initPulses(const std::vector<double>& A,
                                               const std::vector<double>& T);

    // Helper for 2HUMP_EI and 3HUMP_EI polynomial expansion
    static void getFromExpansionCoeffs(double freq, double dampingRatio,
                                       const std::vector<std::vector<double>>& tCoeffs,
                                       const std::vector<std::vector<double>>& aCoeffs,
                                       std::vector<double>& A, std::vector<double>& T);
};
