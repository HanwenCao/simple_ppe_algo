#include "handeye_target/handeyeTarget.hpp"


namespace handeyeCalibration
{

bool estimatePoseCharucoNoniterative(cv::InputArray _charucoCorners, cv::InputArray _charucoIds,
                              const cv::Ptr<cv::aruco::CharucoBoard> &_board, cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
                              cv::InputOutputArray _rvec, cv::InputOutputArray _tvec, bool useExtrinsicGuess);


bool HandeyeTargetAruco::init()
{
  target_params_ready_ = false;
  handeyeTargetConfig_.dictionary_id_str = "DICT_4X4_250";
  dictionary_id_ = cv::aruco::DICT_4X4_250;

  handeyeTargetConfig_.marker_measured_size = 0.053;
  handeyeTargetConfig_.marker_measured_separation = 0.0053;

  target_params_ready_ = true;
  return target_params_ready_;
}


bool HandeyeTargetAruco::configCamera(std::vector<double>& D, boost::array<double, 9>& K)
{
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      camera_matrix_.at<double>(i,j) = K.at(i * 3 + j);
    }
  }
  // matrix D
  // distortion_coeffs_ = cv::Mat::zeros(1, 5, CV_64FC1);
  for (int i = 0; i < 5; ++i)
  {
    distortion_coeffs_.at<double>(1, i) = D.at(i);
  }
  std::cout << "\ncamera_intrinsic_matrix set to: \n" << camera_matrix_ << std::endl;
  std::cout << "\ncamera_distortion_coeffs set to: \n" << distortion_coeffs_ << std::endl;
  return true;
}


handeyeTargetConfig HandeyeTargetAruco::getHandeyeTargetConfig()
{
  std::cout << "get handeye target config" << std::endl;
  return handeyeTargetConfig_;
}


bool HandeyeTargetAruco::configTarget(handeyeTargetConfig config)
{
  // config handeye target
  handeyeTargetConfig_ = config;
  // assign the dictionary id if in the list
  if ( ARUCO_DICTIONARY_.find(handeyeTargetConfig_.dictionary_id_str) == ARUCO_DICTIONARY_.end())
  {
    std::cout << "Error, handeye target type not valid!!!" << std::endl;
    return false;
  }
  dictionary_id_ = ARUCO_DICTIONARY_.find(handeyeTargetConfig_.dictionary_id_str)->second;
  return true;
}


bool HandeyeTargetAruco::createTargetImage(cv::Mat& targetImage) const
{
  cv::Size image_size;
  image_size.width = handeyeTargetConfig_.num_marker_x * (handeyeTargetConfig_.marker_pixel_size + handeyeTargetConfig_.marker_pixel_separation) 
                     - handeyeTargetConfig_.marker_pixel_separation + 2 * handeyeTargetConfig_.marker_pixel_separation;
  image_size.height = handeyeTargetConfig_.num_marker_y* (handeyeTargetConfig_.marker_pixel_size + handeyeTargetConfig_.marker_pixel_separation) 
                     - handeyeTargetConfig_.marker_pixel_separation + 2 * handeyeTargetConfig_.marker_pixel_separation;

  try
  {
    // Create target
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_id_);  
    cv::Ptr<cv::aruco::GridBoard> board =
        cv::aruco::GridBoard::create( handeyeTargetConfig_.num_marker_x, 
                                      handeyeTargetConfig_.num_marker_y, 
                                      float(handeyeTargetConfig_.marker_measured_size), 
                                      float(handeyeTargetConfig_.marker_measured_separation), 
                                      dictionary);

    std::cout << "[createTargetImage] handeyeTarget created." << std::endl;
    std::cout << "[createTargetImage] num_marker_x: " << handeyeTargetConfig_.num_marker_x << std::endl;
    std::cout << "[createTargetImage] num_marker_y: " << handeyeTargetConfig_.num_marker_y << std::endl;
    std::cout << "[createTargetImage] marker_size: " << handeyeTargetConfig_.marker_pixel_size << std::endl;
    std::cout << "[createTargetImage] separation: " << handeyeTargetConfig_.marker_pixel_separation << std::endl;
    std::cout << "[createTargetImage] marker_id: " << dictionary_id_ << std::endl;

    // Create target image
    board->draw(image_size, targetImage, handeyeTargetConfig_.marker_pixel_separation, handeyeTargetConfig_.border_bits);

  }
  catch (const cv::Exception& e)
  {
    std::cout << "[createTargetImage]: failed." << std::endl;
    return false;
  }
  return true;
}


bool HandeyeTargetAruco::createCharucoTargetImage(cv::Mat& targetImage) const
{
  cv::Size image_size;
  image_size.width = handeyeTargetConfig_.num_marker_x * handeyeTargetConfig_.marker_pixel_size;
  image_size.height = handeyeTargetConfig_.num_marker_y* handeyeTargetConfig_.marker_pixel_size;

  try
  {
    // Create target
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_id_);  
    cv::Ptr<cv::aruco::CharucoBoard> board  =
        cv::aruco::CharucoBoard::create(handeyeTargetConfig_.num_marker_x, 
                                        handeyeTargetConfig_.num_marker_y, 
                                        float(handeyeTargetConfig_.marker_measured_size),  // chessboard square side length (m) here
                                        float(handeyeTargetConfig_.marker_measured_separation), //marker side length (m) here
                                        dictionary);

    std::cout << "[createTargetImage] charuco target created." << std::endl;
    std::cout << "[createTargetImage] squaresX: " << handeyeTargetConfig_.num_marker_x << std::endl;
    std::cout << "[createTargetImage] squaresY: " << handeyeTargetConfig_.num_marker_y << std::endl;
    std::cout << "[createTargetImage] squareLength: " << handeyeTargetConfig_.marker_pixel_size << std::endl;
    std::cout << "[createTargetImage] markerLength: " << handeyeTargetConfig_.marker_pixel_separation << std::endl;
    std::cout << "[createTargetImage] dictionary: " << dictionary_id_ << std::endl;

    // Create target image
    board->draw(image_size, targetImage, handeyeTargetConfig_.marker_pixel_separation, handeyeTargetConfig_.border_bits);

  }
  catch (const cv::Exception& e)
  {
    std::cout << "[createTargetImage]: failed." << std::endl;
    return false;
  }
  return true;
}


std::vector<std::string> HandeyeTargetAruco::getArucoDictionaries()
{
  std::vector<std::string> Aruco_dictionaries;
  std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> ::iterator iter;
  for (iter = ARUCO_DICTIONARY_.begin(); iter != ARUCO_DICTIONARY_.end(); iter++)
  {
    Aruco_dictionaries.push_back(iter->first);
  }
  return Aruco_dictionaries;
}


bool HandeyeTargetAruco::detectTargetPose(cv::Mat& img_raw, cv::Mat& H_mat)
{
  // convert to gray if rgb input type
  cv::Mat image;
  if (img_raw.channels() == 3)
      cv::cvtColor(img_raw, image, cv::COLOR_RGB2GRAY);
  else 
      image = img_raw;
  
  try
  {
    // Detect aruco board
    H_mat = cv::Mat::zeros(4, 4, CV_64FC1);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_id_);  
    cv::Ptr<cv::aruco::GridBoard> board =
        cv::aruco::GridBoard::create(handeyeTargetConfig_.num_marker_x, handeyeTargetConfig_.num_marker_y, float(handeyeTargetConfig_.marker_measured_size), float(handeyeTargetConfig_.marker_measured_separation), dictionary);
    cv::Ptr<cv::aruco::DetectorParameters> params_ptr(new cv::aruco::DetectorParameters());

    params_ptr->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids, params_ptr);
    if (marker_ids.empty())
    {
      std::cout << "[detectTargetPose]: Pose estimation error, No marker detected" << std::endl;
      return false;
    }

    // Refine markers borders
    std::vector<std::vector<cv::Point2f>> rejected_corners;
    cv::aruco::refineDetectedMarkers(image, board, marker_corners, marker_ids, rejected_corners, camera_matrix_,
                                     distortion_coeffs_);

    // Rotation and translation of the board w.r.t the camera frame
    cv::Vec3d translation_vect;
    cv::Vec3d rotation_vect;

    // *******************test******************
    // print for debugging
    for (int i = 0; i < marker_corners.size(); ++i)
    {
      // std::cout << "[marker]: " << i << std::endl;
      // std::cout << "[marker id]: " << marker_ids[i] << std::endl;
      corners_map_.insert(std::pair<int, std::vector<cv::Point2f> >(marker_ids[i], marker_corners[i]));     
      // for (int j = 0; j < marker_corners[i].size(); ++j)
      // {
      //   std::cout << "  [corners]:" << j << std::endl;
      //   std::cout << "  " << marker_corners[i][j].x << ", " << marker_corners[i][j].y << std::endl;
      // }
    }
    // *******************test******************


    // Estimate aruco board pose
    int valid = cv::aruco::estimatePoseBoard(marker_corners, marker_ids, board, camera_matrix_, distortion_coeffs_,
                                             rotation_vect, translation_vect);

    cv::Mat Rot_matrix;
    cv::Rodrigues(rotation_vect, Rot_matrix);

    // Draw the markers and frame axis if at least one marker is detected
    if (valid == 0 ||
        std::log10(std::fabs(rotation_vect[0])) > 10    || std::log10(std::fabs(rotation_vect[1])) > 10    ||
        std::log10(std::fabs(rotation_vect[2])) > 10    || std::log10(std::fabs(translation_vect[0])) > 10 ||
        std::log10(std::fabs(translation_vect[1])) > 10 || std::log10(std::fabs(translation_vect[2])) > 10) 
    {
      std::cout << "[detectTargetPose]: Pose estimation error, invalid value" << std::endl;
      return false;
    }

    cv::Mat image_rgb;
    cv::cvtColor(image, image_rgb, cv::COLOR_GRAY2RGB);
    cv::aruco::drawDetectedMarkers(image_rgb, marker_corners);
    drawAxis(image_rgb, camera_matrix_, distortion_coeffs_, rotation_vect, translation_vect, 0.1);

    // return value
    img_raw = image_rgb;
    Rot_matrix.copyTo(H_mat(cv::Range(0,3), cv::Range(0,3)));
    H_mat.at <double>(0,3) = translation_vect[0];
    H_mat.at <double>(1,3) = translation_vect[1];
    H_mat.at <double>(2,3) = translation_vect[2];
    H_mat.at <double>(3,3) = 1.0;
  }
  catch (const cv::Exception& e)
  {
    const char* err_msg = e.what();
    std::cout << "exception caught: " << err_msg << std::endl;
    return false;
  }

  return true;
}

bool HandeyeTargetAruco::detectSinglePose(cv::Mat& img_raw, cv::Mat& H_mat, int id)
{
  // convert to gray if rgb input type
  cv::Mat image;
  if (img_raw.channels() == 3)
      cv::cvtColor(img_raw, image, cv::COLOR_RGB2GRAY);
  else 
      image = img_raw;

  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_id_);  
  cv::Ptr<cv::aruco::DetectorParameters> params_ptr(new cv::aruco::DetectorParameters());
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids, params_ptr);
  if (marker_ids.empty())
  {
    std::cout << "[detectSinglePose]: Pose estimation error, No marker detected" << std::endl;
    return false;
  }

  // pose estimation
  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(marker_corners, handeyeTargetConfig_.marker_measured_separation, camera_matrix_, distortion_coeffs_, rvecs, tvecs);
  std::vector< int >::iterator iter = std::find(marker_ids.begin(),marker_ids.end(),id); 
  int id_index = iter - marker_ids.begin();
  cv::Vec3d translation_vect = tvecs[id_index];
  cv::Vec3d rotation_vect = rvecs[id_index];
  std::cout << id_index << std::endl;

  // detection result visualization
  cv::Mat image_rgb;
  cv::cvtColor(image, image_rgb, cv::COLOR_GRAY2RGB);
  drawAxis(image_rgb, camera_matrix_, distortion_coeffs_, rotation_vect, translation_vect, 0.1);

  cv::aruco::drawDetectedMarkers(image_rgb, marker_corners, marker_ids);

  // // return value
  img_raw = image_rgb;
  cv::Mat Rot_matrix;
  cv::Rodrigues(rotation_vect, Rot_matrix);
  Rot_matrix.copyTo(H_mat(cv::Range(0,3), cv::Range(0,3)));
  H_mat.at <double>(0,3) = translation_vect[0];
  H_mat.at <double>(1,3) = translation_vect[1];
  H_mat.at <double>(2,3) = translation_vect[2];
  H_mat.at <double>(3,3) = 1.0;
}


bool HandeyeTargetAruco::detectCharucoPose(cv::Mat& img_raw, cv::Mat& H_mat, bool corner_detect_only)
{
  // convert to gray if rgb input type
  cv::Mat image;
  if (img_raw.channels() == 3)
      cv::cvtColor(img_raw, image, cv::COLOR_RGB2GRAY);
  else 
      image = img_raw;

  try
  {
    // Detect charuco board
    H_mat = cv::Mat::zeros(4, 4, CV_64FC1);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_id_);  
    cv::Ptr<cv::aruco::DetectorParameters> params_ptr(new cv::aruco::DetectorParameters());
    cv::Ptr<cv::aruco::CharucoBoard> board  =
        cv::aruco::CharucoBoard::create(handeyeTargetConfig_.num_marker_x, 
                                        handeyeTargetConfig_.num_marker_y, 
                                        float(handeyeTargetConfig_.marker_measured_size),  // chessboard square side length (m) here
                                        float(handeyeTargetConfig_.marker_measured_separation), //marker side length (m) here
                                        dictionary);

    params_ptr->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids, params_ptr);
    if (marker_ids.empty())
    {
      std::cout << "[detectTargetPose]: Pose estimation error, No marker detected" << std::endl;
      return false;
    }

    // Refine markers borders
    std::vector<std::vector<cv::Point2f>> rejected_corners;
    cv::aruco::refineDetectedMarkers(image, board, marker_corners, marker_ids, rejected_corners, camera_matrix_,
                                     distortion_coeffs_);

    // Rotation and translation of the board w.r.t the camera frame
    cv::Vec3d translation_vect;
    cv::Vec3d rotation_vect;

    std::vector<cv::Point2f> charucoCorners;
    std::vector<int> charucoIds;
    cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, image, board, charucoCorners, charucoIds, camera_matrix_, distortion_coeffs_);

    // *******************test charuco******************
    // print for debugging
    for (int i = 0; i < charucoCorners.size(); ++i)
    {
      // std::cout << "[marker]: " << i << std::endl;
      // std::cout << "[marker id]: " << charucoIds[i] << std::endl;
      std::vector<cv::Point2f> cross_corner;
      cross_corner.push_back(charucoCorners[i]);
      corners_map_.insert(std::pair<int, std::vector<cv::Point2f> >(charucoIds[i], cross_corner));     
      // std::cout << "  " << charucoCorners[i].x << ", " << charucoCorners[i].y << std::endl;

    }
    // *******************test******************

    // stop if only need to detect corners
    if(corner_detect_only)
    {
      return true;
    }

    // // Estimate charuco board pose
    // int valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, camera_matrix_, distortion_coeffs_,
    //                                          rotation_vect, translation_vect);
         
    int valid = estimatePoseCharucoNoniterative(charucoCorners, charucoIds, board, camera_matrix_, distortion_coeffs_,
                                             rotation_vect, translation_vect, false);

    cv::Mat Rot_matrix;
    cv::Rodrigues(rotation_vect, Rot_matrix);

    // Draw the markers and frame axis if at least one marker is detected
    if (valid == 0 ||
        std::log10(std::fabs(rotation_vect[0])) > 10    || std::log10(std::fabs(rotation_vect[1])) > 10    ||
        std::log10(std::fabs(rotation_vect[2])) > 10    || std::log10(std::fabs(translation_vect[0])) > 10 ||
        std::log10(std::fabs(translation_vect[1])) > 10 || std::log10(std::fabs(translation_vect[2])) > 10) 
    {
      std::cout << "[detectTargetPose]: Pose estimation error, invalid value" << std::endl;
      return false;
    }

    cv::Mat image_rgb;
    cv::cvtColor(image, image_rgb, cv::COLOR_GRAY2RGB);
    // cv::aruco::drawDetectedMarkers(image_rgb, marker_corners);
    drawAxis(image_rgb, camera_matrix_, distortion_coeffs_, rotation_vect, translation_vect, 0.1);

    cv::aruco::drawDetectedMarkers(image_rgb, marker_corners);
    cv::aruco::drawDetectedCornersCharuco(image_rgb, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
    cv::imwrite("/home/dunn/Calibration_ws/calibration_module/charuco_detected.jpg", image_rgb);


    // return value
    img_raw = image_rgb;
    Rot_matrix.copyTo(H_mat(cv::Range(0,3), cv::Range(0,3)));
    H_mat.at <double>(0,3) = translation_vect[0];
    H_mat.at <double>(1,3) = translation_vect[1];
    H_mat.at <double>(2,3) = translation_vect[2];
    H_mat.at <double>(3,3) = 1.0;
  }
  catch (const cv::Exception& e)
  {
    const char* err_msg = e.what();
    std::cout << "exception caught: " << err_msg << std::endl;
    return false;
  }

  return true;
}

void HandeyeTargetAruco::drawAxis(cv::InputOutputArray _image, cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
                                  cv::InputArray _rvec, cv::InputArray _tvec, float length) const
{
  CV_Assert(_image.getMat().total() != 0 && (_image.getMat().channels() == 1 || _image.getMat().channels() == 3));
  CV_Assert(length > 0);

  // project axis points
  std::vector<cv::Point3f> axis_points;
  axis_points.push_back(cv::Point3f(0, 0, 0));
  axis_points.push_back(cv::Point3f(length, 0, 0));
  axis_points.push_back(cv::Point3f(0, length, 0));
  axis_points.push_back(cv::Point3f(0, 0, length));
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(axis_points, _rvec, _tvec, _cameraMatrix, _distCoeffs, image_points);

  // draw axis lines
  cv::line(_image, image_points[0], image_points[1], cv::Scalar(0, 0, 255), 2);
  cv::line(_image, image_points[0], image_points[2], cv::Scalar(0, 255, 0), 2);
  cv::line(_image, image_points[0], image_points[3], cv::Scalar(255, 0, 0), 2);


  cv::putText(_image, "z: " + std::to_string(_tvec.getMat().at<double>(2,0)), 
              image_points[0] , cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2, 8, 0);
  image_points[0].y -= 27;
  cv::putText(_image, "y: " + std::to_string(_tvec.getMat().at<double>(1,0)), 
              image_points[0] , cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2, 8, 0);
  image_points[0].y -= 27;
  cv::putText(_image, "x: " + std::to_string(_tvec.getMat().at<double>(0,0)), 
              image_points[0] , cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2, 8, 0);              

}

bool estimatePoseCharucoNoniterative(cv::InputArray _charucoCorners, cv::InputArray _charucoIds,
                              const cv::Ptr<cv::aruco::CharucoBoard> &_board, cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
                              cv::InputOutputArray _rvec, cv::InputOutputArray _tvec, bool useExtrinsicGuess) {

    CV_Assert((_charucoCorners.getMat().total() == _charucoIds.getMat().total()));

    // need, at least, 4 corners
    if(_charucoIds.getMat().total() < 4) return false;

    std::vector<cv::Point3f> objPoints;
    objPoints.reserve(_charucoIds.getMat().total());
    for(unsigned int i = 0; i < _charucoIds.getMat().total(); i++) {
        int currId = _charucoIds.getMat().at< int >(i);
        CV_Assert(currId >= 0 && currId < (int)_board->chessboardCorners.size());
        objPoints.push_back(_board->chessboardCorners[currId]);
    }

    // points need to be in different lines, check if detected points are enough
    // if(!_arePointsEnoughForPoseEstimation(objPoints)) return false;



    // enum  	cv::SolvePnPMethod {
    //   cv::SOLVEPNP_ITERATIVE = 0,
    //   cv::SOLVEPNP_EPNP = 1,
    //   cv::SOLVEPNP_P3P = 2,
    //   cv::SOLVEPNP_DLS = 3,
    //   cv::SOLVEPNP_UPNP = 4,
    //   cv::SOLVEPNP_AP3P = 5,
    //   cv::SOLVEPNP_IPPE = 6,
    //   cv::SOLVEPNP_IPPE_SQUARE = 7,
    //   cv::SOLVEPNP_SQPNP = 8
    // }
    solvePnP(objPoints, _charucoCorners, _cameraMatrix, _distCoeffs, _rvec, _tvec, useExtrinsicGuess, 6);

    return true;
}


} // namespace handeyeCalibration
