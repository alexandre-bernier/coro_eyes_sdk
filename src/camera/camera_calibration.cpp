/** @file camera_calibration.cpp
 *  @brief Contains methods for the Camera calibration class.
 *  @details Taken from https://github.com/opencv/opencv/blob/3.4/samples/cpp/tutorial_code/calib3d/camera_calibration/camera_calibration.cpp
 *  @copyright BSD-3-Clause License
 */

#include <iostream>
#include <sstream>
#include <ctime>
#include <cstdio>
#include <opencv2/imgcodecs.hpp>
#include "camera/camera_calibration.h"

/**
 * @brief Calculate the real board corners' position
 * @param boardSize: Amount of corners
 * @param squareSize: cv::Size of the squares
 * @param [out] corners: Reference of the std::vector that will hold the board corners' position
 * @param patternType: Type of board used (see Pattern)
 */
void calcBoardCornerPositions(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners,
                              Calibration::Settings::Pattern patternType)
{
    corners.clear();

    switch(patternType) {
    case Calibration::Settings::CHESSBOARD:
    case Calibration::Settings::CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                corners.push_back(cv::Point3f(j*squareSize, i*squareSize, 0));
        break;

    case Calibration::Settings::ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(cv::Point3f((2*j + i % 2)*squareSize, i*squareSize, 0));
        break;

    default:
        break;
    }
}

/**
 * @brief Compute the reprojection errors
 * @param objectPoints: Coordinates of the object points (std::vector of std::vector of cv::Point3f)
 * @param imagePoints: Coordinates of the image points (std::vector of std::vector of cv::Point2f)
 * @param rvecs: Rotation std::vector provided by cv::calibrateCamera
 * @param tvecs: Translation std::vector provided by cv::calibrateCamera
 * @param cameraMatrix: Intrinsic matrix provided by cv::calibrateCamera
 * @param distCoeffs: Distorsion coefficients provided by cv::calibrateCamera
 * @param [out] perViewErrors: Error for each view provided
 * @param fisheye: True if using a Fisheye camera
 * @return The average error for all views
 */
double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& objectPoints,
                                 const std::vector<std::vector<cv::Point2f> >& imagePoints,
                                 const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                                 const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                                 std::vector<float>& perViewErrors, bool fisheye)
{
    std::vector<cv::Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for(size_t i=0; i<objectPoints.size(); ++i ) {
        if(fisheye) {
            cv::fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
                                       distCoeffs);
        }
        else {
            projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        }

        err = norm(imagePoints[i], imagePoints2, cv::NORM_L2);

        size_t n = objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

/**
 * @brief Convert an OpenCV rotation matrix into quaternions.
 * @details Taken from https://gist.github.com/shubh-agrawal/76754b9bfb0f4143819dbd146d15d4c8
 * @param R: Rotation matrix of type cv::Mat
 * @param Q: Quaternions of type double[4]
 */
void getQuaternion(cv::Mat R, double Q[])
{
    double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);

    if (trace > 0.0)
    {
        double s = sqrt(trace + 1.0);
        Q[3] = (s * 0.5);
        s = 0.5 / s;
        Q[0] = ((R.at<double>(2,1) - R.at<double>(1,2)) * s);
        Q[1] = ((R.at<double>(0,2) - R.at<double>(2,0)) * s);
        Q[2] = ((R.at<double>(1,0) - R.at<double>(0,1)) * s);
    }

    else
    {
        int i = R.at<double>(0,0) < R.at<double>(1,1) ? (R.at<double>(1,1) < R.at<double>(2,2) ? 2 : 1) : (R.at<double>(0,0) < R.at<double>(2,2) ? 2 : 0);
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;

        double s = sqrt(R.at<double>(i, i) - R.at<double>(j,j) - R.at<double>(k,k) + 1.0);
        Q[i] = s * 0.5;
        s = 0.5 / s;

        Q[3] = (R.at<double>(k,j) - R.at<double>(j,k)) * s;
        Q[j] = (R.at<double>(j,i) + R.at<double>(i,j)) * s;
        Q[k] = (R.at<double>(k,i) + R.at<double>(i,k)) * s;
    }
}

namespace Calibration {

// Write serialization for this class
void Settings::write(cv::FileStorage& fs) const
{
    fs << "{"
              << "BoardSize_Width"  << boardSize.width
              << "BoardSize_Height" << boardSize.height
              << "Square_Size"         << squareSize
              << "Calibrate_Pattern" << patternToUse
              << "Calibrate_NrOfFrameToUse" << nrFrames
              << "Calibrate_FixAspectRatio" << aspectRatio
              << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
              << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

              << "Write_DetectedFeaturePoints" << writePoints
              << "Write_extrinsicParameters"   << writeExtrinsics

              << "Show_UndistortedImage" << showUndistorted

              << "Input_FlipAroundHorizontalAxis" << flipVertical
       << "}";
}

// Read serialization for this class
void Settings::read(const cv::FileNode& node)
{
    node["BoardSize_Width" ] >> boardSize.width;
    node["BoardSize_Height"] >> boardSize.height;
    node["Calibrate_Pattern"] >> patternToUse;
    node["Square_Size"]  >> squareSize;
    node["Calibrate_NrOfFrameToUse"] >> nrFrames;
    node["Calibrate_FixAspectRatio"] >> aspectRatio;
    node["Write_DetectedFeaturePoints"] >> writePoints;
    node["Write_extrinsicParameters"] >> writeExtrinsics;
    node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
    node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
    node["Calibrate_UseFisheyeModel"] >> useFisheye;
    node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
    node["Show_UndistortedImage"] >> showUndistorted;
    node["Fix_K1"] >> fixK1;
    node["Fix_K2"] >> fixK2;
    node["Fix_K3"] >> fixK3;
    node["Fix_K4"] >> fixK4;
    node["Fix_K5"] >> fixK5;

    validate();
}

void Settings::validate()
{
    goodInput = true;
    if (boardSize.width <= 0 || boardSize.height <= 0)
    {
        std::cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << std::endl;
        goodInput = false;
    }
    if (squareSize <= 10e-6)
    {
        std::cerr << "Invalid square size " << squareSize << std::endl;
        goodInput = false;
    }
    if (nrFrames <= 0)
    {
        std::cerr << "Invalid number of frames " << nrFrames << std::endl;
        goodInput = false;
    }

    flag = 0;
    if(calibFixPrincipalPoint) flag |= cv::CALIB_FIX_PRINCIPAL_POINT;
    if(calibZeroTangentDist)   flag |= cv::CALIB_ZERO_TANGENT_DIST;
    if(aspectRatio)            flag |= cv::CALIB_FIX_ASPECT_RATIO;
    if(fixK1)                  flag |= cv::CALIB_FIX_K1;
    if(fixK2)                  flag |= cv::CALIB_FIX_K2;
    if(fixK3)                  flag |= cv::CALIB_FIX_K3;
    if(fixK4)                  flag |= cv::CALIB_FIX_K4;
    if(fixK5)                  flag |= cv::CALIB_FIX_K5;

    if (useFisheye) {
        // the fisheye model has its own enum, so overwrite the flags
        flag = cv::fisheye::CALIB_FIX_SKEW | cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
        if(fixK1)                   flag |= cv::fisheye::CALIB_FIX_K1;
        if(fixK2)                   flag |= cv::fisheye::CALIB_FIX_K2;
        if(fixK3)                   flag |= cv::fisheye::CALIB_FIX_K3;
        if(fixK4)                   flag |= cv::fisheye::CALIB_FIX_K4;
        if(calibFixPrincipalPoint)  flag |= cv::fisheye::CALIB_FIX_PRINCIPAL_POINT;
    }

    calibrationPattern = NOT_EXISTING;
    if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
    if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
    if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
    if (calibrationPattern == NOT_EXISTING)
    {
        std::cerr << "Camera calibration mode does not exist: " << patternToUse << std::endl;
        goodInput = false;
    }
}

/**
 * @brief Find calibration board corners from an image.
 * @param s: Calibration settings (see Calibration::Settings)
 * @param image: Image in cv::Mat format
 * @param [out] image_points: Coordinates of the calibration board corners found
 * @return True if corners were found
 */
bool find_corners(Settings& s, cv::Mat& image, std::vector<cv::Point2f>& image_points)
{
    bool found;

    int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;

    if(!s.useFisheye) {
        // Fast check erroneously fails with high distortions like fisheye
        chessBoardFlags |= cv::CALIB_CB_FAST_CHECK;
    }

    switch( s.calibrationPattern ) {    // Find feature points on the input format
    case Settings::CHESSBOARD:
        found = findChessboardCorners(image, s.boardSize, image_points, chessBoardFlags);
        break;
    case Settings::CIRCLES_GRID:
        found = findCirclesGrid(image, s.boardSize, image_points );
        break;
    case Settings::ASYMMETRIC_CIRCLES_GRID:
        found = findCirclesGrid(image, s.boardSize, image_points, cv::CALIB_CB_ASYMMETRIC_GRID );
        break;
    default:
        found = false;
        break;
    }

    // If done with success
    if(found) {
        // Improve the found corners' coordinate accuracy for chessboard
        if(s.calibrationPattern == Settings::CHESSBOARD) {
            cv::cornerSubPix(image, image_points, cv::Size(11,11),
                             cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1));
        }
    }

    return found;
}

/**
 * @brief Run camera calibration.
 * @param s: Calibration settings (see Calibration::Settings)
 * @param image_size: Size of the images used to find the image_points in cv::Size format
 * @param [out] calib_data: Structure containing the camera calibration results (see Calibration::Data)
 * @param image_points: Coordinates of the calibration board corners found for all images (see Calibration::find_corners)
 * @return True if the calibration succeeded
 */
bool run_camera_calibration(Settings& s, cv::Size image_size, Data& calib_data, std::vector<std::vector<cv::Point2f> >& image_points)
{
    // Initialize calibration data arrays
    calib_data.intrinsic = cv::Mat::eye(3, 3, CV_64F);

    if(!s.useFisheye && s.flag & cv::CALIB_FIX_ASPECT_RATIO)
        calib_data.intrinsic.at<double>(0,0) = s.aspectRatio;

    if(s.useFisheye)
        calib_data.distorsion = cv::Mat::zeros(4, 1, CV_64F);
    else
        calib_data.distorsion = cv::Mat::zeros(8, 1, CV_64F);

    // Create object points
    std::vector<std::vector<cv::Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);
    objectPoints.resize(image_points.size(),objectPoints[0]);

    // Find intrinsic and extrinsic camera parameters
    double rms;

    if(s.useFisheye) {
        cv::Mat _rvecs, _tvecs;
        rms = cv::fisheye::calibrate(objectPoints, image_points, image_size, calib_data.intrinsic, calib_data.distorsion,
                                     _rvecs, _tvecs, s.flag);

        calib_data.rvecs.reserve(_rvecs.rows);
        calib_data.tvecs.reserve(_tvecs.rows);
        for(int i=0; i<int(objectPoints.size()); i++) {
            calib_data.rvecs.push_back(_rvecs.row(i));
            calib_data.tvecs.push_back(_tvecs.row(i));
        }
    }

    else {
        rms = cv::calibrateCamera(objectPoints, image_points, image_size, calib_data.intrinsic, calib_data.distorsion,
                                  calib_data.rvecs, calib_data.tvecs, s.flag);
    }

    // Re-projection error
    bool ok = checkRange(calib_data.intrinsic) && checkRange(calib_data.distorsion);

    calib_data.total_avg_error = computeReprojectionErrors(objectPoints, image_points, calib_data.rvecs, calib_data.tvecs, calib_data.intrinsic,
                                                           calib_data.distorsion, calib_data.reproj_errors, s.useFisheye);

    return ok;
}

/**
 * @brief Save the calibration results to a file
 * @param file_name: Name of the file where the calibration results will be saved
 * @param s: Calibration settings (see Calibration::Settings)
 * @param image_size: Size of the images used to find the image_points in cv::Size format
 * @param calib_data: Structure containing the camera calibration results (see Calibration::run_calibration)
 * @param image_points: Coordinates of the calibration board corners found for all images (see Calibration::find_corners)
 * @return True if the file was created successfully
 */
bool save_camera_calibration(std::string file_name, Settings& s, cv::Size image_size, Data& calib_data, std::vector<std::vector<cv::Point2f> >& image_points)
{
    cv::FileStorage fs(file_name, cv::FileStorage::WRITE);

    if(!fs.isOpened()) {
        std::cerr << "Could not open/create the calibration data file: " << file_name << std::endl;
        return false;
    }

    time_t tm;
    time(&tm);
    struct tm *t2 = localtime(&tm);
    char buf[1024];
    strftime(buf, sizeof(buf), "%c", t2);

    fs << "calibration_time" << buf;

    if(!calib_data.rvecs.empty() || !calib_data.reproj_errors.empty())
        fs << "nr_of_frames" << (int)std::max(calib_data.rvecs.size(), calib_data.reproj_errors.size());
    fs << "image_width" << image_size.width;
    fs << "image_height" << image_size.height;
    fs << "board_width" << s.boardSize.width;
    fs << "board_height" << s.boardSize.height;
    fs << "square_size" << s.squareSize;

    if(!s.useFisheye && s.flag & cv::CALIB_FIX_ASPECT_RATIO)
        fs << "fix_aspect_ratio" << s.aspectRatio;

    if(s.flag) {
        std::stringstream flagsStringStream;
        if (s.useFisheye) {
            flagsStringStream << "flags:"
                << (s.flag & cv::fisheye::CALIB_FIX_SKEW ? " +fix_skew" : "")
                << (s.flag & cv::fisheye::CALIB_FIX_K1 ? " +fix_k1" : "")
                << (s.flag & cv::fisheye::CALIB_FIX_K2 ? " +fix_k2" : "")
                << (s.flag & cv::fisheye::CALIB_FIX_K3 ? " +fix_k3" : "")
                << (s.flag & cv::fisheye::CALIB_FIX_K4 ? " +fix_k4" : "")
                << (s.flag & cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC ? " +recompute_extrinsic" : "");
        }
        else {
            flagsStringStream << "flags:"
                << (s.flag & cv::CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "")
                << (s.flag & cv::CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "")
                << (s.flag & cv::CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "")
                << (s.flag & cv::CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "")
                << (s.flag & cv::CALIB_FIX_K1 ? " +fix_k1" : "")
                << (s.flag & cv::CALIB_FIX_K2 ? " +fix_k2" : "")
                << (s.flag & cv::CALIB_FIX_K3 ? " +fix_k3" : "")
                << (s.flag & cv::CALIB_FIX_K4 ? " +fix_k4" : "")
                << (s.flag & cv::CALIB_FIX_K5 ? " +fix_k5" : "");
        }
        fs.writeComment(flagsStringStream.str());
    }

    fs << "flags" << s.flag;

    fs << "fisheye_model" << s.useFisheye;

    fs << "camera_matrix" << calib_data.intrinsic;
    fs << "distortion_coefficients" << calib_data.distorsion;

    fs << "avg_reprojection_error" << calib_data.total_avg_error;
    if(s.writeExtrinsics && !calib_data.reproj_errors.empty())
        fs << "per_view_reprojection_errors" << cv::Mat(calib_data.reproj_errors);

    if(s.writeExtrinsics && !calib_data.rvecs.empty() && !calib_data.tvecs.empty()) {
        fs.writeComment("extrinsic parameters");
        fs << "rvecs" << calib_data.rvecs;
        fs << "tvecs" << calib_data.tvecs;
    }

    if(s.writePoints && !image_points.empty()) {
        cv::Mat imagePtMat((int)image_points.size(), (int)image_points[0].size(), CV_32FC2);
        for(size_t i=0; i<image_points.size(); i++) {
            cv::Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
            cv::Mat imgpti(image_points[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }

    return true;
}

/**
 * @brief Load camera calibratino data from file.
 * @param [in] file_name: Name of the file where the calibration results are stored
 * @param [out] calib_data: Structure containing the camera calibration results (see Calibration::run_calibration)
 * @return True if the calibration data was loaded successfully
 */
bool load_camera_calibration(std::string file_name, Data& calib_data)
{
    cv::FileStorage fs(file_name, cv::FileStorage::READ);

    if(!fs.isOpened()) {
        std::cout << "Could not open the stereo calibration data file: " << file_name << std::endl;
        return false;
    }

    fs["camera_matrix"] >> calib_data.intrinsic;
    fs["distortion_coefficients"] >> calib_data.distorsion;

    // Not loading these parameters for now since they aren't needed (and they aren't always saved to file)
//    fs["rvecs"] >> calib_data.rvecs;
//    fs["tvecs"] >> calib_data.tvecs;
//    fs["avg_reprojection_error"] >> calib_data.total_avg_error;

//    cv::Mat reproj_errors_mat;
//    fs["per_view_reprojection_errors"] >> reproj_errors_mat;
//    unsigned int num_errors = reproj_errors_mat.rows * reproj_errors_mat.cols;
//    for(unsigned int i=0; i<num_errors; i++) {
//        calib_data.reproj_errors.push_back(reproj_errors_mat.at<float>(i,0));
//    }


    fs.release();

    return true;
}

/**
 * @brief Run stereo camera calibration and generate rectify maps.
 * @param s: Calibration settings (see Calibration::Settings)
 * @param image_size: Size of the images used to find the image_points in cv::Size format
 * @param [out] stereo_calib_data: Structure containing the stereo camera calibration results (see Calibration::StereoData)
 * @param [in] camL_calib_data: Structure containing the camera calibration results (see Calibration::run_camera_calibration)
 * @param [in] camL_image_points: Coordinates of the calibration board corners found for all images (see Calibration::find_corners)
 * @param [in] camR_calib_data: Structure containing the camera calibration results (see Calibration::run_camera_calibration)
 * @param [in] camR_image_points: Coordinates of the calibration board corners found for all images (see Calibration::find_corners)
 * @return True if the calibration succeeded
 */
bool run_stereo_calibration(Settings& s, cv::Size image_size, StereoData& stereo_calib_data,
                            Data& camL_calib_data, std::vector<std::vector<cv::Point2f> > &camL_image_points,
                            Data& camR_calib_data, std::vector<std::vector<cv::Point2f> > &camR_image_points)
{
    // Check that the cameras' calibration data isn't empty
    if(camL_calib_data.intrinsic.empty() || camR_calib_data.intrinsic.empty() ||
            camL_calib_data.distorsion.empty() || camR_calib_data.distorsion.empty()) {
        std::cerr << "Camera calibration is empty. Please call \"run_camera_calibration()\" for each camera before calling \"run_stereo_calibration()\"." << std::endl;
        return false;
    }

    // Create object points
    std::vector<std::vector<cv::Point3f> > objectPoints;
    objectPoints.resize(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);
    objectPoints.resize(camL_image_points.size(), objectPoints[0]);

    // Create calibration flags
    int stereo_calibration_flags;

    if(s.useFisheye) {
        stereo_calibration_flags =
                cv::fisheye::CALIB_FIX_SKEW |
                cv::fisheye::CALIB_FIX_INTRINSIC |
                cv::fisheye::CALIB_USE_INTRINSIC_GUESS |
                cv::fisheye::CALIB_FIX_K1 |
                cv::fisheye::CALIB_FIX_K2 |
                cv::fisheye::CALIB_FIX_K3 |
                cv::fisheye::CALIB_FIX_K4;
        if(s.calibFixPrincipalPoint)
            stereo_calibration_flags |= cv::fisheye::CALIB_FIX_PRINCIPAL_POINT;
    }

    else {
        stereo_calibration_flags =
                cv::CALIB_USE_INTRINSIC_GUESS |
                cv::CALIB_SAME_FOCAL_LENGTH |
                cv::CALIB_RATIONAL_MODEL |
                cv::CALIB_FIX_K3 |
                cv::CALIB_FIX_K4 |
                cv::CALIB_FIX_K5;
        if(s.calibFixPrincipalPoint)
            stereo_calibration_flags |= cv::CALIB_FIX_PRINCIPAL_POINT;
        if(s.calibZeroTangentDist)
            stereo_calibration_flags |= cv::CALIB_ZERO_TANGENT_DIST;
        if(s.aspectRatio)
            stereo_calibration_flags |= cv::CALIB_FIX_ASPECT_RATIO;
    }

    // Find stereo extrinsic parameters (to bring cam1 into cam2's coordinate system)
    double rms;

    if(s.useFisheye) {
        rms = cv::fisheye::stereoCalibrate(objectPoints, camR_image_points, camL_image_points,
                                           camR_calib_data.intrinsic, camR_calib_data.distorsion, camL_calib_data.intrinsic, camL_calib_data.distorsion,
                                           image_size, stereo_calib_data.R, stereo_calib_data.T, stereo_calibration_flags,
                                           cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, 1e-5));
    }
    else {
        rms = cv::stereoCalibrate(objectPoints, camR_image_points, camL_image_points,
                                  camR_calib_data.intrinsic, camR_calib_data.distorsion, camL_calib_data.intrinsic, camL_calib_data.distorsion,
                                  image_size, stereo_calib_data.R, stereo_calib_data.T, stereo_calib_data.E, stereo_calib_data.F,
                                  stereo_calibration_flags, cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, 1e-5));
    }

    // Re-projection error
    bool ok = checkRange(stereo_calib_data.R) && checkRange(stereo_calib_data.T);

    stereo_calib_data.total_avg_error = rms;

    // Find rectification transforms for each camera
    cv::stereoRectify(camR_calib_data.intrinsic, camR_calib_data.distorsion, camL_calib_data.intrinsic, camL_calib_data.distorsion,
                      image_size, stereo_calib_data.R, stereo_calib_data.T, stereo_calib_data.R1, stereo_calib_data.R2, stereo_calib_data.P1, stereo_calib_data.P2, stereo_calib_data.Q,
                      cv::CALIB_ZERO_DISPARITY, 1, image_size, &stereo_calib_data.validROI1, &stereo_calib_data.validROI2);

    return ok;
}

/**
 * @brief Save the calibration results to a file
 * @param file_name: Name of the file where the calibration results will be saved
 * @param s: Calibration settings (see Calibration::Settings)
 * @param image_size: Size of the images used to find the image_points in cv::Size format
 * @param stereo_calib_data: Structure containing the stereo camera calibration results (see Calibration::StereoData)
 * @param camL_image_points: Coordinates of the calibration board corners found for all images (see Calibration::find_corners)
 * @param camR_image_points: Coordinates of the calibration board corners found for all images (see Calibration::find_corners)
 * @return True if the file was created successfully
 */
bool save_stereo_calibration(std::string file_name, Settings& s, cv::Size image_size, StereoData& stereo_calib_data,
                             std::vector<std::vector<cv::Point2f> > &camL_image_points,
                             std::vector<std::vector<cv::Point2f> > &camR_image_points)
{
    cv::FileStorage fs(file_name, cv::FileStorage::WRITE);

    if(!fs.isOpened()) {
        std::cerr << "Could not open/create the calibration data file: " << file_name << std::endl;
        return false;
    }

    time_t tm;
    time(&tm);
    struct tm *t2 = localtime(&tm);
    char buf[1024];
    strftime(buf, sizeof(buf), "%c", t2);

    fs << "calibration_time" << buf;

    if(!camL_image_points.empty())
        fs << "nr_of_frames" << (int)camL_image_points.size();
    fs << "image_width" << image_size.width;
    fs << "image_height" << image_size.height;
    fs << "board_width" << s.boardSize.width;
    fs << "board_height" << s.boardSize.height;
    fs << "square_size" << s.squareSize;

    if(!s.useFisheye && s.flag & cv::CALIB_FIX_ASPECT_RATIO)
        fs << "fix_aspect_ratio" << s.aspectRatio;

    std::stringstream flagsStringStream;
    if (s.useFisheye) {
        flagsStringStream << "flags:" <<
                             " +fix_skew" <<
                             " +fix_instrinsic" <<
                             " +use_intrinsic_guess" <<
                             " +fix_k1" <<
                             " +fix_k2" <<
                             " +fix_k3" <<
                             " +fix_k4" <<
                             (s.calibFixPrincipalPoint ? " +fix_principal_point" : "");
    }
    else {
        flagsStringStream << "flags:" <<
                             " +fix_instrinsic" <<
                             " +use_intrinsic_guess" <<
                             " +fix_k1" <<
                             " +fix_k2" <<
                             " +fix_k3" <<
                             " +fix_k4" <<
                             " +fix_k5" <<
                             " +fix_s1_s2_s3_s4" <<
                             (s.calibFixPrincipalPoint ? " +fix_principal_point" : "") <<
                             (s.calibZeroTangentDist ? " +zero_tangent_dist" : "") <<
                             (s.aspectRatio ? " +fix_aspectRatio" : "");
    }
    fs.writeComment(flagsStringStream.str());

    fs << "flags" << s.flag;

    fs << "fisheye_model" << s.useFisheye;

    fs << "avg_reprojection_error" << stereo_calib_data.total_avg_error;

    if(!stereo_calib_data.R.empty() && !stereo_calib_data.T.empty() &&
            !stereo_calib_data.E.empty() && !stereo_calib_data.F.empty()) {
        fs << "R" << stereo_calib_data.R;
        fs << "T" << stereo_calib_data.T;
        fs << "E" << stereo_calib_data.E;
        fs << "F" << stereo_calib_data.F;
    }

    if(!stereo_calib_data.R1.empty() && !stereo_calib_data.R2.empty() &&
            !stereo_calib_data.P1.empty() && !stereo_calib_data.P2.empty() &&
            !stereo_calib_data.Q.empty()) {
        fs << "R1" << stereo_calib_data.R1;
        fs << "R2" << stereo_calib_data.R2;
        fs << "P1" << stereo_calib_data.P1;
        fs << "P2" << stereo_calib_data.P2;
        fs << "Q" << stereo_calib_data.Q;
    }

    if(!stereo_calib_data.validROI1.empty() && !stereo_calib_data.validROI2.empty()) {
        fs << "validROI1" << stereo_calib_data.validROI1;
        fs << "validROI2" << stereo_calib_data.validROI2;
    }

    if(s.writePoints && !camL_image_points.empty()) {
        cv::Mat imagePtMat((int)camL_image_points.size(), (int)camL_image_points[0].size(), CV_32FC2);

        for(size_t i=0; i<camL_image_points.size(); i++) {
            cv::Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
            cv::Mat imgpti(camL_image_points[i]);
            imgpti.copyTo(r);
        }
        fs << "camL_image_points" << imagePtMat;

        for(size_t i=0; i<camR_image_points.size(); i++) {
            cv::Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
            cv::Mat imgpti(camR_image_points[i]);
            imgpti.copyTo(r);
        }
        fs << "camR_image_points" << imagePtMat;
    }

    return true;
}

/**
 * @brief Load stereo calibratino data from file.
 * @param [in] file_name: Name of the file where the calibration results are stored
 * @param [out] stereo_calib_data: Structure that will contain the stereo camera calibration results (see Calibration::StereoData)
 * @return True if the calibration data was loaded successfully
 */
bool load_stereo_calibration(std::string file_name, StereoData& stereo_calib_data)
{
    cv::FileStorage fs(file_name, cv::FileStorage::READ);

    if(!fs.isOpened()) {
        std::cout << "Could not open the stereo calibration data file: " << file_name << std::endl;
        return false;
    }

    fs["R"] >> stereo_calib_data.R;
    fs["T"] >> stereo_calib_data.T;
    fs["E"] >> stereo_calib_data.E;
    fs["F"] >> stereo_calib_data.F;
    fs["avg_reprojection_error"] >> stereo_calib_data.total_avg_error;
    fs["R1"] >> stereo_calib_data.R1;
    fs["R2"] >> stereo_calib_data.R2;
    fs["P1"] >> stereo_calib_data.P1;
    fs["P2"] >> stereo_calib_data.P2;
    fs["Q"] >> stereo_calib_data.Q;
    fs["validROI1"] >> stereo_calib_data.validROI1;
    fs["validROI2"] >> stereo_calib_data.validROI2;

    fs.release();

    return true;
}

/**
 * @brief Generate stereo rectify maps from provided calibration data
 * @param [in] camL_calib_data: Structure containing the camera calibration results (see Calibration::run_camera_calibration)
 * @param [in] camR_calib_data: Structure containing the camera calibration results (see Calibration::run_camera_calibration)
 * @param [in] stereo_calib_data: Structure containing the stereo camera calibration results (see Calibration::StereoData)
 * @param [in] image_size: Size of the images used to find the image_points in cv::Size format
 * @param [out] reproj_maps_L: Structure containing re-projection maps (see Calibration::ReprojMaps)
 * @param [out] reproj_maps_R: Structure containing re-projection maps (see Calibration::ReprojMaps)
 * @return True if the rectify maps were generated successfuly
 */
bool calculate_stereo_reproj_maps(Data& camL_calib_data, Data& camR_calib_data, StereoData& stereo_calib_data, cv::Size image_size,
                                  ReprojMaps &reproj_maps_L, ReprojMaps &reproj_maps_R)
{
    // Exit if data is missing
    if(camL_calib_data.intrinsic.empty() || camL_calib_data.distorsion.empty() ||
            camR_calib_data.intrinsic.empty() || camR_calib_data.distorsion.empty() ||
            stereo_calib_data.R1.empty() || stereo_calib_data.P1.empty() ||
            stereo_calib_data.R2.empty() || stereo_calib_data.P2.empty() ||
            image_size.width <= 0 || image_size.height <= 0)
        return false;

    // Generate stereo rectify maps from provided calibration data
    cv::initUndistortRectifyMap(camL_calib_data.intrinsic, camL_calib_data.distorsion,
                                stereo_calib_data.R2, stereo_calib_data.P2, image_size, CV_32FC1, reproj_maps_L.mapx, reproj_maps_L.mapy);
    cv::initUndistortRectifyMap(camR_calib_data.intrinsic, camR_calib_data.distorsion,
                                stereo_calib_data.R1, stereo_calib_data.P1, image_size, CV_32FC1, reproj_maps_R.mapx, reproj_maps_R.mapy);

    return true;
}

/**
 * @brief Remap images using the provided re-projection maps.
 * @param [in] original_images: Vector of images to be remapped
 * @param [in] reproj_maps: Structure containing re-projection maps (see Calibration::ReprojMaps)
 * @param [out] remapped_images: Vector of remapped images
 * @return True if the remapping completed successfuly
 */
bool remap_images(std::vector<cv::Mat>& original_images, ReprojMaps &reproj_maps, std::vector<cv::Mat>& remapped_images)
{
    // Exit if missing arguements
    if(reproj_maps.mapx.empty() || reproj_maps.mapy.empty() || original_images.empty())
        return false;

    // Empty the output vector
    remapped_images.clear();

    // Remap all images from the input vector
    for(unsigned int i_image=0; i_image<original_images.size(); i_image++) {
        cv::Mat remapped_image;
        cv::remap(original_images[i_image], remapped_image, reproj_maps.mapx, reproj_maps.mapy, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar());
        remapped_images.push_back(remapped_image);
    }

    return true;
}

/**
 * @brief Estimate the pose of a camera in relation to a user-defined reference frame using
 * real life coordinates of chessboard corners.
 * @param calib_data: Structure containing the camera calibration results (see Calibration::Data)
 * @param stereo_calib_data: Structure containing the stereo camera calibration results (see Calibration::StereoData)
 * @param object_points: Coordinates of the calibration board corners in the user-defined reference frame (3D)
 * @param image_points: Pixel coordinates of the same calibration board corners (see Calibration::find_corners) (2D)
 * @param [out] pose_data: Structure containing the pose estimation results (see Calibration::Pose)
 * @return True if the pose estimation succeeded
 */
bool run_pose_estimation(Data& calib_data, StereoData& stereo_calib_data, std::vector<cv::Point3f>& object_points,
                         std::vector<cv::Point2f>& image_points, Pose& pose_data)
{
    bool success = false;

    // Verify arguments
    if(calib_data.intrinsic.empty() || calib_data.distorsion.empty() ||
            object_points.empty() || image_points.empty() ||
            object_points.size() != image_points.size())
        return success;

    // Pose estimation
    cv::Mat rvecs, tvecs;
    success = cv::solvePnP(object_points, image_points, calib_data.intrinsic, calib_data.distorsion,
                                rvecs, tvecs, false, cv::SOLVEPNP_ITERATIVE);

    if(success) {
        // Convert rotation vector to matrix
        cv::Mat rmat;
        cv::Rodrigues(rvecs, rmat);

        // Create homogenous matrix
        cv::Mat h, last_row;
        cv::hconcat(rmat, tvecs, h);
        cv::vconcat(h, cv::Mat::zeros(1, 4, CV_64FC1), h);
        h.at<double>(3,3) = 1;

        // Rotate to be in the reprojected coordinate system
        cv::Mat r = stereo_calib_data.R2;
        cv::hconcat(r, cv::Mat::zeros(3,1,CV_64FC1), r);
        cv::vconcat(r, cv::Mat::zeros(1, 4, CV_64FC1), r);
        r.at<double>(3,3) = 1;

        h = r * h;

        // Fill Pose data translation
        pose_data.translation[0] = h.at<double>(0,3);
        pose_data.translation[1] = h.at<double>(1,3);
        pose_data.translation[2] = h.at<double>(2,3);

        // Fill Pose data quaternions
        getQuaternion(h, pose_data.quaternions);
    }

    return success;
}

/**
 * @brief Save the pose data to file.
 * @param [in] file_name: Name of the file where the pose estimation results will be saved
 * @param [out] pose_data: Structure containing the pose estimation results (see Calibration::Pose)
 * @return True if the pose estimation data was saved successfully
 */
bool save_pose_estimation(std::string file_name, Pose& pose_data)
{
    cv::FileStorage fs(file_name, cv::FileStorage::WRITE);

    if(!fs.isOpened()) {
        std::cerr << "Could not open/create the pose data file: " << file_name << std::endl;
        return false;
    }

    time_t tm;
    time(&tm);
    struct tm *t2 = localtime(&tm);
    char buf[1024];
    strftime(buf, sizeof(buf), "%c", t2);

    fs << "calibration_time" << buf;

    fs.writeComment("Pose of a user-defined frame in relation to the right camera.");
    fs << "frame" << pose_data.frame;

    fs.writeComment("Translation (m)");
    fs << "x" << pose_data.translation[0];
    fs << "y" << pose_data.translation[1];
    fs << "z" << pose_data.translation[2];

    fs.writeComment("Quaternions");
    fs << "x" << pose_data.quaternions[0];
    fs << "y" << pose_data.quaternions[1];
    fs << "z" << pose_data.quaternions[2];
    fs << "w" << pose_data.quaternions[3];

    fs.writeComment(std::to_string(pose_data.translation[0]) + " " + std::to_string(pose_data.translation[1]) + " " + std::to_string(pose_data.translation[2]) + "\n" +
            std::to_string(pose_data.quaternions[0]) + " " + std::to_string(pose_data.quaternions[1]) + " " + std::to_string(pose_data.quaternions[2]) + " " + std::to_string(pose_data.quaternions[3]));

    return true;
}

}
