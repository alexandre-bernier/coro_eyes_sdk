/** @file camera_calibration.cpp
 *  @brief Contains methods for the Camera calibration class.
 *  @details Taken from https://github.com/opencv/opencv/blob/3.4/samples/cpp/tutorial_code/calib3d/camera_calibration/camera_calibration.cpp
 *  @copyright BSD-3-Clause License
 */

#include <iostream>
#include <sstream>
#include <ctime>
#include <cstdio>
#include "camera_calibration.h"

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
        CV_Assert(calib_data.rvecs[0].type() == calib_data.tvecs[0].type());
        cv::Mat bigmat((int)calib_data.rvecs.size(), 6, CV_MAKETYPE(calib_data.rvecs[0].type(), 1));
        bool needReshapeR = calib_data.rvecs[0].depth() != 1 ? true : false;
        bool needReshapeT = calib_data.tvecs[0].depth() != 1 ? true : false;

        for(size_t i=0; i<calib_data.rvecs.size(); i++) {
            cv::Mat r = bigmat(cv::Range(int(i), int(i+1)), cv::Range(0,3));
            cv::Mat t = bigmat(cv::Range(int(i), int(i+1)), cv::Range(3,6));

            if(needReshapeR)
                calib_data.rvecs[i].reshape(1, 1).copyTo(r);
            else {
                //*.t() is MatExpr (not Mat) so we can use assignment operator
                CV_Assert(calib_data.rvecs[i].rows == 3 && calib_data.rvecs[i].cols == 1);
                r = calib_data.rvecs[i].t();
            }

            if(needReshapeT)
                calib_data.tvecs[i].reshape(1, 1).copyTo(t);
            else {
                CV_Assert(calib_data.tvecs[i].rows == 3 && calib_data.tvecs[i].cols == 1);
                t = calib_data.tvecs[i].t();
            }
        }
        fs.writeComment("a set of 6-tuples (rotation vector + translation vector) for each view");
        fs << "extrinsic_parameters" << bigmat;
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

bool run_stereo_calibration(Settings& s, cv::Size image_size, StereoData& stereo_calib_data,
                            Data& cam1_calib_data, std::vector<std::vector<cv::Point2f> > &cam1_image_points,
                            Data& cam2_calib_data, std::vector<std::vector<cv::Point2f> > &cam2_image_points)
{
    // Check that the cameras' calibration data isn't empty
    if(cam1_calib_data.intrinsic.empty() || cam2_calib_data.intrinsic.empty() ||
            cam1_calib_data.distorsion.empty() || cam2_calib_data.distorsion.empty()) {
        std::cerr << "Camera calibration is empty. Please call \"run_camera_calibration()\" for each camera before calling \"run_stereo_calibration()\"." << std::endl;
        return false;
    }

    // Create object points
    std::vector<std::vector<cv::Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);
    objectPoints.resize(cam1_image_points.size(),objectPoints[0]);

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
                cv::CALIB_FIX_INTRINSIC |
                cv::CALIB_FIX_K1 |
                cv::CALIB_FIX_K2 |
                cv::CALIB_FIX_K3 |
                cv::CALIB_FIX_K4 |
                cv::CALIB_FIX_K5 |
                cv::CALIB_FIX_S1_S2_S3_S4;
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
        rms = cv::fisheye::stereoCalibrate(objectPoints, cam1_image_points, cam2_image_points,
                                           cam1_calib_data.intrinsic, cam1_calib_data.distorsion, cam2_calib_data.intrinsic, cam2_calib_data.distorsion,
                                           image_size, stereo_calib_data.R, stereo_calib_data.T, stereo_calibration_flags);
    }
    else {
        rms = cv::stereoCalibrate(objectPoints, cam1_image_points, cam2_image_points,
                                  cam1_calib_data.intrinsic, cam1_calib_data.distorsion, cam2_calib_data.intrinsic, cam2_calib_data.distorsion,
                                  image_size, stereo_calib_data.R, stereo_calib_data.T, stereo_calib_data.E, stereo_calib_data.F,
                                  stereo_calibration_flags);
    }

    // Re-projection error
    bool ok = checkRange(stereo_calib_data.R) && checkRange(stereo_calib_data.T);

    stereo_calib_data.total_avg_error = rms;

    return ok;
}

bool save_stereo_calibration(std::string file_name, Settings& s, cv::Size image_size, StereoData& stereo_calib_data,
                             std::vector<std::vector<cv::Point2f> > &cam1_image_points,
                             std::vector<std::vector<cv::Point2f> > &cam2_image_points)
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

    if(!cam1_image_points.empty())
        fs << "nr_of_frames" << (int)cam1_image_points.size();
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

    if(!stereo_calib_data.R.empty() && !stereo_calib_data.T.empty()) {
        fs << "R" << stereo_calib_data.R;
        fs << "T" << stereo_calib_data.T;
    }

    if(s.writePoints && !cam1_image_points.empty()) {
        cv::Mat imagePtMat((int)cam1_image_points.size(), (int)cam1_image_points[0].size(), CV_32FC2);

        for(size_t i=0; i<cam1_image_points.size(); i++) {
            cv::Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
            cv::Mat imgpti(cam1_image_points[i]);
            imgpti.copyTo(r);
        }
        fs << "cam1_image_points" << imagePtMat;

        for(size_t i=0; i<cam2_image_points.size(); i++) {
            cv::Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
            cv::Mat imgpti(cam2_image_points[i]);
            imgpti.copyTo(r);
        }
        fs << "cam2_image_points" << imagePtMat;
    }

    return true;
}

}
