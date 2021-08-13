/** @file structured_light_patterns.cpp
 *  @brief Utility class to generate structured light patterns.
 *  @copyright 2021 Alexandre Bernier
 */

#include "structured_light_patterns.h"
#include <opencv2/highgui.hpp>

StructuredLightPatterns::StructuredLightPatterns(unsigned int proj_width, unsigned int proj_height) :
    width(proj_width),
    height(proj_height)
{
    // Create sinusoidal patterns parameter structure
    cv::structured_light::SinusoidalPattern::Params params;
    params.width = width;
    params.height = height;
    params.nbrOfPeriods = 20;
    params.setMarkers = true;
    params.horizontal = false;
    params.methodId = cv::structured_light::FTP;
    _cv_sinusoidal_patterns_params = cv::makePtr<cv::structured_light::SinusoidalPattern::Params>(params);
}

StructuredLightPatterns::~StructuredLightPatterns()
{
    _cv_graycode_patterns.release();
    _cv_sinusoidal_patterns.release();
}

/**
 * @brief Generate gray code structured light patterns using OpenCV.
 * @link https://docs.opencv.org/4.5.2/d1/dec/classcv_1_1structured__light_1_1GrayCodePattern.html
 */
void StructuredLightPatterns::generate_gray_code_patterns()
{
    // Clean old patterns
    _cv_graycode_patterns.release();
    _cv_sinusoidal_patterns.release();

    // Create new instance
    _cv_graycode_patterns = cv::structured_light::GrayCodePattern::create(width, height);
    _patterns_generated = _PatternType::GrayCode;

    // Generate patterns
    _cv_graycode_patterns->generate(_images);

    // Convert the patterns into the dlp equivalent
    _convert_cv_patterns_to_dlp();
}

/**
 * @brief Generate sinusoidal structured light patterns using OpenCV.
 * @link https://docs.opencv.org/4.5.2/d6/d96/classcv_1_1structured__light_1_1SinusoidalPattern.html
 */
void StructuredLightPatterns::generate_sinusoidal_patterns()
{
    // Clean old patterns
    _cv_graycode_patterns.release();
    _cv_sinusoidal_patterns.release();

    // Create new instance
    _cv_sinusoidal_patterns = cv::structured_light::SinusoidalPattern::create(_cv_sinusoidal_patterns_params);
    _patterns_generated = _PatternType::Sinusoidal;

    // Generate patterns
    _cv_sinusoidal_patterns->generate(_images);

    // Convert the patterns into the dlp equivalent
    _convert_cv_patterns_to_dlp();
}

/**
 * @brief Show all generated structured light pattern images.
 */
void StructuredLightPatterns::visualize_patterns()
{
    // Exit if no patterns were generated
    if(_patterns_generated == _PatternType::None)
        return;

    // Windows
    std::string window_name;
    if(_patterns_generated == _PatternType::GrayCode)
        window_name = "Gray Code Patterns";
    else if(_patterns_generated == _PatternType::Sinusoidal)
        window_name = "Sinusoidal Patterns";

    // Display patterns one at a time
    for(unsigned int i=0; i<get_nb_patterns(); i++) {
        cv::imshow(window_name, _images[i]);
        cv::waitKey(0);
    }

    // Close window
    cv::destroyAllWindows();
}

/**
 * @brief Create a DLP pattern sequence from each structured light pattern images
 *        generated previously.
 */
void StructuredLightPatterns::_convert_cv_patterns_to_dlp()
{
    // Exit if no patterns were generated
    if(_patterns_generated == _PatternType::None)
        return;

    // Clear the old sequence
    _dlp_pattern_sequence.Clear();

    // Variable holding a single structured light pattern
    dlp::Pattern dlp_pattern;
    dlp_pattern.color = dlp::Pattern::Color::WHITE;
    dlp_pattern.data_type = dlp::Pattern::DataType::IMAGE_DATA;
    if(_patterns_generated ==  _PatternType::GrayCode)
        dlp_pattern.bitdepth = dlp::Pattern::Bitdepth::MONO_1BPP;
    else if(_patterns_generated ==  _PatternType::Sinusoidal)
        dlp_pattern.bitdepth = dlp::Pattern::Bitdepth::MONO_8BPP;

    // Create the DLP pattern sequence
    for(unsigned int i=0; i<get_nb_patterns(); i++) {
        dlp_pattern.image_data.Clear();
        dlp_pattern.image_data.Create(_images[i]);
        _dlp_pattern_sequence.Add(dlp_pattern);
    }
}
