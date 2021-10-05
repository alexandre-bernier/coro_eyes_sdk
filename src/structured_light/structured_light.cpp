/** @file structured_light.cpp
 *  @brief Contains methods for the StructuredLight class.
 *  @copyright BSD-3-Clause License
 */

#include <mutex>
#include <opencv2/highgui.hpp>
#include "structured_light/structured_light.h"

/**
 * @warning The number of cameras shouldn't exceed 2.
 */
StructuredLight::StructuredLight(unsigned int proj_height, unsigned int proj_width, unsigned int num_cameras):
    _proj_height(proj_height),
    _proj_width(proj_width),
    _num_cameras(num_cameras)
{
    // Create sinusoidal patterns parameter structure
    cv::structured_light::SinusoidalPattern::Params params;
    params.width = _proj_width;
    params.height = _proj_height;
    params.nbrOfPeriods = 20;
    params.setMarkers = true;
    params.horizontal = false;
    params.methodId = cv::structured_light::FTP;
    _cv_sinusoidal_patterns_params = cv::makePtr<cv::structured_light::SinusoidalPattern::Params>(params);
}

StructuredLight::~StructuredLight()
{
    _cv_graycode_patterns.release();
    _cv_sinusoidal_patterns.release();
}

/**
 * @brief Generate gray code structured light patterns using OpenCV.
 * @details Taken from https://docs.opencv.org/4.5.2/d1/dec/classcv_1_1structured__light_1_1GrayCodePattern.html.
 */
void StructuredLight::generate_gray_code_patterns()
{
    // Clean old patterns
    _cv_graycode_patterns.release();
    _cv_sinusoidal_patterns.release();

    // Create new instance
    _cv_graycode_patterns = cv::structured_light::GrayCodePattern::create(_proj_width, _proj_height);
    _pattern_type_generated = PatternType::GrayCode;

    // Generate patterns
    _cv_graycode_patterns->generate(_pattern_images);

    // Generate all-white and all-black images for shadows mask computation
    cv::Mat white, black;
    _cv_graycode_patterns->getImagesForShadowMasks(black, white);
    _pattern_images.push_back(white);
    _pattern_images.push_back(black);

    // Set thresholds
    _cv_graycode_patterns->setWhiteThreshold(0);
}

/**
 * @brief Generate sinusoidal structured light patterns using OpenCV.
 * @details Taken from https://docs.opencv.org/4.5.2/d6/d96/classcv_1_1structured__light_1_1SinusoidalPattern.html.
 */
void StructuredLight::generate_sinusoidal_patterns()
{
    // Clean old patterns
    _cv_graycode_patterns.release();
    _cv_sinusoidal_patterns.release();

    // Create new instance
    _cv_sinusoidal_patterns = cv::structured_light::SinusoidalPattern::create(_cv_sinusoidal_patterns_params);
    _pattern_type_generated = PatternType::Sinusoidal;

    // Generate patterns
    _cv_sinusoidal_patterns->generate(_pattern_images);

    /// @todo Compute phase maps
}

/**
 * @brief Show all generated structured light pattern images.
 */
void StructuredLight::visualize_patterns()
{
    // Exit if no patterns were generated
    if(_pattern_type_generated == PatternType::None)
        return;

    // Windows
    std::string window_name;
    if(_pattern_type_generated == PatternType::GrayCode)
        window_name = "Gray Code Patterns";
    else if(_pattern_type_generated == PatternType::Sinusoidal)
        window_name = "Sinusoidal Patterns";

    // Display patterns one at a time
    for(unsigned int i=0; i<get_nb_patterns(); i++) {
        cv::imshow(window_name, _pattern_images[i]);
        cv::waitKey();
    }

    // Close window
    cv::destroyAllWindows();
}

/**
 * @brief Identify the structured light patterns in the captured images and copy them into another vector.
 * @warning It's important that the images be dark before the start of the pattern sequence.
 * @param [in] captured_images: Vector of images captured by a camera
 * @param [out] pattern_images: Vector of images containing only patterns
 * @return True if the operation was successful
 */
bool StructuredLight::extract_pattern_images(std::vector<cv::Mat> captured_images, std::vector<cv::Mat>& pattern_images)
{
    bool first_pattern_found = false;
    double image_sum = 0;
    double previous_sum = image_sum;

    pattern_images.clear();

    for(unsigned int i_image=0; i_image<captured_images.size(); i_image++) {
        if(!first_pattern_found) {
            image_sum = cv::sum(captured_images[i_image])[0];

            if(i_image == 0)
                previous_sum = image_sum;

            else {
                if(image_sum > (previous_sum * 1.1))
                    first_pattern_found = true;
            }
        }

        if(first_pattern_found) {
            pattern_images.push_back(captured_images[i_image]);
        }

        if(pattern_images.size() == get_nb_patterns())
            break;
    }

    if(pattern_images.size() == get_nb_patterns())
        return true;

    return false;
}

/**
 * @brief Compute the disparity map from the provided captured images.
 * @details captured_images must have the same size as the number of cameras with the right camera first.
 *          The white image pattern must be the second to last image in each vector.
 *          The black image pattern must be the last image in each vector.
 * @param [in] captured_images: Vector of vectors containing the captured (remapped) images of all cameras
 * @param [out] disparity_map: Generated disparity map
 * @return True if the disparity map was generated successfuly
 */
bool StructuredLight::compute_disparity_map(std::vector<std::vector<cv::Mat> > captured_images, cv::Mat& disparity_map)
{
    bool decoded = false;

    // Exit if the input vector is of a different size than the number of cameras
    if(captured_images.size() != _num_cameras)
        return decoded;

    // Separate the input vector images
    std::vector<cv::Mat> white_images;
    std::vector<cv::Mat> black_images;

    // Used for threshold calculation
    double min, max, black_thresh = 0;

    // For each camera...
    for(unsigned int i_cam=0; i_cam<_num_cameras; i_cam++) {
        // Extract black image
        black_images.push_back(captured_images[i_cam].back());
        captured_images[i_cam].pop_back();

        // Extract white image
        white_images.push_back(captured_images[i_cam].back());
        captured_images[i_cam].pop_back();

        // Extract information for threshold calculation
        cv::minMaxIdx(black_images[i_cam], &min, &max);
        black_thresh += max;
    }

    // Find the black threshold with the black pattern images (average of max values of black images)
    black_thresh /= _num_cameras;
    _cv_graycode_patterns->setBlackThreshold(black_thresh);

    // Generate the disparity map
    if(_pattern_type_generated == PatternType::GrayCode)
        decoded = _cv_graycode_patterns->decode(captured_images, disparity_map, black_images, white_images,
                                                cv::structured_light::DECODE_3D_UNDERWORLD);

    return decoded;
}

/**
 * @brief Apply a colormap to a disparity map to help visualization.
 * @details Taken from https://docs.opencv.org/4.5.1/dc/da9/tutorial_decode_graycode_pattern.html.
 * @param disparity_map: A disparity map (see StructuredLight::generate_disparity_map)
 * @return A colored disparity map
 */
cv::Mat StructuredLight::apply_color_to_disparity_map(const cv::Mat& disparity_map)
{
    double min, max;
    cv::Mat cm_disp, scaledDisparityMap;

    cv::minMaxIdx(disparity_map, &min, &max);

    cv::convertScaleAbs(disparity_map, scaledDisparityMap, 255/(max-min));
    cv::applyColorMap(scaledDisparityMap, cm_disp, cv::COLORMAP_JET);

    return cm_disp;
}

/**
 * @brief Compute a point cloud from a disparity map.
 * @details
 * @param disparity_map: A disparity map (see StructuredLight::generate_disparity_map)
 * @param Q: Disparity-to-depth mapping matrix (see Calibration::StereoData and Calibration::run_stereo_calibration)
 * @return A point cloud
 */
std::vector<cv::Point3f> StructuredLight::compute_point_cloud(cv::Mat disparity_map, const cv::Mat& Q)
{
    // Compute point cloud
    cv::Mat point_cloud_image;
    disparity_map.convertTo(disparity_map, CV_32FC1);
    cv::reprojectImageTo3D(disparity_map, point_cloud_image, Q, true, -1);

    // Compute mask to remove the background
    double min, max;
    cv::Mat scaledDisparityMap;
    cv::minMaxIdx(disparity_map, &min, &max);
    cv::convertScaleAbs(disparity_map, scaledDisparityMap, 255/(max-min));

    cv::Mat thresholded_disp;
    cv::threshold(scaledDisparityMap, thresholded_disp, 0, 255, cv::THRESH_OTSU + cv::THRESH_BINARY);

    // Apply mask to the point cloud
    cv::Mat point_cloud_tresh;
    point_cloud_image.copyTo(point_cloud_tresh, thresholded_disp);

    // Remove points at origin
    std::vector<cv::Point3f> point_cloud;
    point_cloud.reserve(thresholded_disp.size().width * thresholded_disp.size().height);    // Reserving the maximum size possible because we get a SegFault when the lambda functor tries to reallocate the vector

    std::mutex pixel_mutex;
    point_cloud_tresh.forEach<cv::Point3f>([&](cv::Point3f &point, const int *position) -> void {
        if(point.x != 0.0 && point.y != 0.0 && point.z != 0.0) {
            const std::lock_guard<std::mutex> lock(pixel_mutex);    // Must lock when pushing into the vector, because cv::Mat::ForEach runs in parallel
            point_cloud.push_back(cv::Point3f(point.x, point.y, point.z));
        }
    });

    return point_cloud;
}
