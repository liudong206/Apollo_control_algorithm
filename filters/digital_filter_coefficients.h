#ifndef DIGITAL_FILTER_COEFFICIENTS_H
#define DIGITAL_FILTER_COEFFICIENTS_H

#include <vector>

/**
 * @file digital_filter_coefficients.h
 * @brief Functions to generate coefficients for digital filter.
 */
namespace common {

/**
 * @brief Get low-pass coefficients for digital filter.
 * @param ts Time interval between signals.
 * @param cutoff_freq Cutoff of frequency to filter high-frequency signals out.
 * @param denominators Denominator coefficients for digital filter.
 * @param numerators Numerator coefficients for digital filter.
 */
void LpfCoefficients(const double ts, const double cutoff_freq,
                     std::vector<double> *denominators,
                     std::vector<double> *numerators);

}  // namespace common

#endif // DIGITAL_FILTER_COEFFICIENTS_H
