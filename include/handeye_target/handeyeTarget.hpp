#pragma once

#include <vector>
#include <map>
#include <iostream>
#include <boost/geometry/geometries/adapted/boost_array.hpp>
#include "handeyeTargetConfig.hpp"
// #include <handeye_target_base.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

// RealSense
#include <librealsense2/rs.hpp>


namespace handeyeCalibration
{
class HandeyeTargetAruco
{
public:
  HandeyeTargetAruco()
  {
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    distortion_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
  }
  ~HandeyeTargetAruco()
  {
    camera_matrix_ .release();
    distortion_coeffs_.release();
  }

  /**
   * @brief Initialize handeye target with default parameters.
   * @return True if initialization was successful, otherwise false.
   */
  bool init();

  /**
   * @brief Create an target image, so that the target can be viewed and printed.
   * @param image Use for storing the created image.
   * @return True if no errors happen, false otherwise.
   */
  bool createTargetImage(cv::Mat& targetImage) const;

  /**
   * @brief Create an charuco target image, so that the target can be viewed and printed.
   * @param image Use for storing the created image.
   * @return True if no errors happen, false otherwise.
   */
  bool createCharucoTargetImage(cv::Mat& targetImage) const;

  /**
   * @brief Given an image containing a target captured from a camera view point, get the target pose with respect to
   * the camera optical frame. Target parameters and camera intrinsic parameters should be correctly set
   * before calling this function.
   * @param image Input image, assume a grayscale image.
   * @param H_mat Output transformation matrix with cv::Mat type.
   * @return True if no errors happen, false otherwise.
   */
  bool detectTargetPose(cv::Mat& image, cv::Mat& H_mat);

  /**
   * @brief detectTargetPose in charuco version
   */
  bool detectCharucoPose(cv::Mat& img_raw, cv::Mat& H_mat, bool corner_detect_only = false);

  bool detectSinglePose(cv::Mat& img_raw, cv::Mat& H_mat, int id);


  // Replace OpenCV drawAxis func with custom one, drawing (x, y, z) -axes in red, green, blue color
  void drawAxis(cv::InputOutputArray _image, cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
                cv::InputArray _rvec, cv::InputArray _tvec, float length) const;

  /**
   * @brief Get aruco marker dictionaries.
   * @return vector of string, containing all keys in dictionaries.
   */
  std::vector<std::string> getArucoDictionaries();

  /**
   * @brief Config camera by the given intrinc parameters, only for handeye target to do pnp.
   * Dosen't work for real camera.
   * @param D Distortion coefficients (k1, k2, t1, t2, k3).
   * @param K Array like camera matrix K (fx, 0, cx, 0, fy, cy, 0, 0, 1).
   * @return True if no errors happen, false otherwise.
   */
  bool configCamera(std::vector<double>& D, boost::array<double, 9>& K);

  /**
   * @brief Config the handeye target by given cofiguration parameters.
   * @param config Parameters of handeye target with struct type. 
   * @return True if no errors happen, false otherwise.
   */
  bool configTarget(handeyeTargetConfig config);

  /**
   * @brief Get the current handeye target parameters.
   * @return handeye target parameters struct.
   */
  handeyeTargetConfig getHandeyeTargetConfig();

  std::map<int, std::vector<cv::Point2f> > corners_map_; 


private:

  //***************************
  // 1. Target parameters
  // Predefined dictionary map
  std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> ARUCO_DICTIONARY_ = {
  { "DICT_4X4_250", cv::aruco::DICT_4X4_250 },
  { "DICT_5X5_250", cv::aruco::DICT_5X5_250 },
  { "DICT_6X6_250", cv::aruco::DICT_6X6_250 },
  { "DICT_7X7_250", cv::aruco::DICT_7X7_250 },
  { "DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL },
  { "DICT_APRILTAG_16h5", cv::aruco::DICT_APRILTAG_16h5 },
  { "DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11 },
};

  handeyeTargetConfig handeyeTargetConfig_; // struct of handeye target parameters

  cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_id_;     // Marker dictionary id

  //****************************
  // 2. Camera intrinsic paramters

  // 3x3 floating-point camera matrix
  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]
  cv::Mat camera_matrix_;

  // Distortion coefficients (k1, k2, t1, t2, k3)
  cv::Mat distortion_coeffs_;

  // flag to indicate if target parameter values are correctly defined
  bool target_params_ready_;

  // List of parameters for this target type
  // std::vector<Parameter> parameters_;

}; // class HandeyeTargetAruco
  
}  // namespace handeyeCalibration