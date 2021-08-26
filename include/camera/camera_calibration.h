/** @file camera_calibration.h
 *  @brief Main header for the camera calibration api.
 *  @details Taken from https://github.com/opencv/opencv/blob/3.4/samples/cpp/tutorial_code/calib3d/camera_calibration/camera_calibration.cpp
 *  @copyright BSD-3-Clause License
 */
#ifndef _CAMERA_CALIBRATION_H
#define _CAMERA_CALIBRATION_H

#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

namespace Calibration
{
class Settings
{
public:
    enum Pattern {
        NOT_EXISTING,
        CHESSBOARD,
        CIRCLES_GRID,
        ASYMMETRIC_CIRCLES_GRID };

    Settings() : goodInput(false) {}

    void write(cv::FileStorage& fs) const;  // Write serialization for this class
    void read(const cv::FileNode& node);    // Read serialization for this class

    void validate();

public:
    cv::Size boardSize;             // The size of the board -> Number of items by width and height
    Pattern calibrationPattern;     // One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;               // The size of a square in your defined unit (point, millimeter,etc).
    int nrFrames;                   // The number of frames to use from the input for calibration
    float aspectRatio;              // The aspect ratio
    bool writePoints;               // Write detected feature points
    bool writeExtrinsics;           // Write extrinsic parameters
    bool calibZeroTangentDist;      // Assume zero tangential distortion
    bool calibFixPrincipalPoint;    // Fix the principal point at the center
    bool flipVertical;              // Flip the captured images around the horizontal axis
    bool showUndistorted;           // Show undistorted images after calibration
    bool useFisheye;                // use fisheye camera model for calibration
    bool fixK1;                     // fix K1 distortion coefficient
    bool fixK2;                     // fix K2 distortion coefficient
    bool fixK3;                     // fix K3 distortion coefficient
    bool fixK4;                     // fix K4 distortion coefficient
    bool fixK5;                     // fix K5 distortion coefficient

    bool goodInput;
    int flag;

private:
    std::string patternToUse;
};

static inline void read(const cv::FileNode& node, Settings& x, const Settings& default_value = Settings())
{
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

    struct Data {
        cv::Mat intrinsic;
        cv::Mat distorsion;
        std::vector<cv::Mat> rvecs, tvecs;
        std::vector<float> reproj_errors;
        double total_avg_error;
    };

    bool find_corners(Settings& s, cv::Mat& image, std::vector<cv::Point2f>& image_points);
    bool run_calibration(Settings& s, cv::Size image_size, Data& calib_data, std::vector<std::vector<cv::Point2f> > &image_points);
    bool save_calibration(std::string file_name, Settings& s, cv::Size image_size, Data& calib_data, std::vector<std::vector<cv::Point2f> > &image_points);
};

#endif // _CAMERA_CALIBRATION_H
