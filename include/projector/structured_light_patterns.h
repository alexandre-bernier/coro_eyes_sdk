/** @file structured_light_patterns.h
 *  @brief Utility class to generate structured light patterns.
 *  @copyright BSD-3-Clause License
 */
#ifndef _STRUCTURED_LIGHT_PATTERNS_H
#define _STRUCTURED_LIGHT_PATTERNS_H

#include <vector>
#include <opencv2/structured_light.hpp>
#include "common/pattern/pattern.hpp"

class StructuredLightPatterns
{
public:
    StructuredLightPatterns(unsigned int proj_width = 912, unsigned int proj_height = 1140);
    ~StructuredLightPatterns();

    void generate_gray_code_patterns();
    void generate_sinusoidal_patterns();
    unsigned int get_nb_patterns() {return _images.size();};

    void visualize_patterns();

    dlp::Pattern::Sequence *get_dlp_patterns() {return &_dlp_pattern_sequence;};

private:
    enum _PatternType
    {
        None,
        GrayCode,
        Sinusoidal
    };

    _PatternType _patterns_generated = _PatternType::None;

    cv::Ptr<cv::structured_light::GrayCodePattern> _cv_graycode_patterns = nullptr;

    cv::Ptr<cv::structured_light::SinusoidalPattern> _cv_sinusoidal_patterns = nullptr;
    cv::Ptr<cv::structured_light::SinusoidalPattern::Params> _cv_sinusoidal_patterns_params = nullptr;

    std::vector<cv::Mat> _images;

    dlp::Pattern::Sequence _dlp_pattern_sequence;

    unsigned int width = 0;
    unsigned int height = 0;

    void _convert_cv_patterns_to_dlp();
};

#endif // _STRUCTURED_LIGHT_PATTERNS_H
