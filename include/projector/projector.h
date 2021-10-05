/** @file projector.h
 *  @brief Main header for the projector api.
 *  @copyright BSD-3-Clause License
 */
#ifndef _PROJECTOR_H
#define _PROJECTOR_H

#include <projector/dlp/common/returncode.hpp>
#include <projector/dlp/dlp_platforms/lightcrafter_4500/lcr4500.hpp>

/**
 * @brief Create a DLP pattern sequence from a series of gray code OpenCV patterns.
 */
dlp::Pattern::Sequence convert_gray_code_cv_patterns_to_dlp(const std::vector<cv::Mat>& images)
{
    dlp::Pattern::Sequence dlp_pattern_sequence;

    // Variable holding a single structured light pattern
    dlp::Pattern dlp_pattern;
    dlp_pattern.color = dlp::Pattern::Color::WHITE;
    dlp_pattern.data_type = dlp::Pattern::DataType::IMAGE_DATA;
    dlp_pattern.bitdepth = dlp::Pattern::Bitdepth::MONO_1BPP;

    // Create the DLP pattern sequence
    for(unsigned int i=0; i<images.size(); i++) {
        dlp_pattern.image_data.Clear();
        dlp_pattern.image_data.Create(images[i]);
        dlp_pattern_sequence.Add(dlp_pattern);
    }

    return dlp_pattern_sequence;
}

/**
 * @brief Create a DLP pattern sequence from a series of sinusoidal OpenCV patterns.
 */
dlp::Pattern::Sequence convert_sinusoidal_cv_patterns_to_dlp(const std::vector<cv::Mat>& images)
{
    dlp::Pattern::Sequence dlp_pattern_sequence;

    // Variable holding a single structured light pattern
    dlp::Pattern dlp_pattern;
    dlp_pattern.color = dlp::Pattern::Color::WHITE;
    dlp_pattern.data_type = dlp::Pattern::DataType::IMAGE_DATA;
    dlp_pattern.bitdepth = dlp::Pattern::Bitdepth::MONO_8BPP;

    // Create the DLP pattern sequence
    for(unsigned int i=0; i<images.size(); i++) {
        dlp_pattern.image_data.Clear();
        dlp_pattern.image_data.Create(images[i]);
        dlp_pattern_sequence.Add(dlp_pattern);
    }

    return dlp_pattern_sequence;
}

#endif // _PROJECTOR_H
