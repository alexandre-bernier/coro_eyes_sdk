/** @file structured_light.h
 *  @brief Main header for the structured light api.
 *  @copyright BSD-3-Clause License
 */
#ifndef _STRUCTURED_LIGHT_H
#define _STRUCTURED_LIGHT_H

#include <vector>
#include <opencv2/structured_light.hpp>

class StructuredLight
{
public:
    enum PatternType {
        None,
        GrayCode,
        Sinusoidal
    };

    StructuredLight(unsigned int proj_height = 1140, unsigned int proj_width = 912, unsigned int num_cameras = 2);
    ~StructuredLight();

    void generate_gray_code_patterns();
    void generate_sinusoidal_patterns();

    inline PatternType get_pattern_type() {return _pattern_type_generated;};
    inline unsigned int get_nb_patterns() {return _pattern_images.size();};
    inline std::vector<cv::Mat> get_pattern_images() {return _pattern_images;};

    void visualize_patterns();

    bool extract_pattern_images(std::vector<cv::Mat> captured_images, std::vector<cv::Mat>& pattern_images);

    bool compute_disparity_map(std::vector<std::vector<cv::Mat> > captured_images, cv::Mat& disparity_map);
    cv::Mat apply_color_to_disparity_map(const cv::Mat &disparity_map);

    std::vector<cv::Point3f> compute_point_cloud(cv::Mat disparity_map, const cv::Mat& Q);
    cv::Mat compute_depth_map(cv::Mat disparity_map, const cv::Mat& Q);

private:
    PatternType _pattern_type_generated = PatternType::None;

    cv::Ptr<cv::structured_light::GrayCodePattern> _cv_graycode_patterns = nullptr;

    cv::Ptr<cv::structured_light::SinusoidalPattern> _cv_sinusoidal_patterns = nullptr;
    cv::Ptr<cv::structured_light::SinusoidalPattern::Params> _cv_sinusoidal_patterns_params = nullptr;

    std::vector<cv::Mat> _pattern_images;

    unsigned int _proj_height = 0;
    unsigned int _proj_width = 0;

    unsigned int _num_cameras = 0;
};

#endif // _STRUCTURED_LIGHT_H
