#include "handeye_target/handeyeTarget.hpp"

bool rs_connect(rs2::pipeline pipe_);
std::vector<double> cam_info_D_;
boost::array<double, 9> cam_info_K_;

int main(int argc, char *argv[]) {

  // 1. CHARUCO Configurations
  // create instance
  handeyeCalibration::HandeyeTargetAruco target_aruco_;
  target_aruco_.init();

  // config
  handeyeTargetConfig charuco_cfg;
  charuco_cfg.marker_measured_size = 0.02f;
  charuco_cfg.marker_measured_separation = 0.016f;
  charuco_cfg.dictionary_id_str = "DICT_4X4_250";
  target_aruco_.configTarget(charuco_cfg);

  // get camera intrinsic parameters which is needed for pose estimation
  // automatically set params if connected to realsense
  rs2::pipeline pipe_;
  rs_connect(pipe_);
  target_aruco_.configCamera(cam_info_D_, cam_info_K_);

  // hardcode params if cannot connect to realsense

  // 2. Generate charuco cheesboard
  cv::Mat targetImage;
  if (target_aruco_.createCharucoTargetImage(targetImage))
  {
      cv::imshow("targetAruco", targetImage);
      cv::waitKey(0);
      cv::destroyWindow("targetAruco");
      cv::imwrite("chAruco_5X5_DICT4X4.jpg", targetImage);
  }


  // 3. test with data
  // read single img for simulation test 
  cv::Mat color_mat = cv::imread("img_0.jpg");
  cv::Mat gray_mat;
  cv::Mat Hcam_cvmat;
  // detect markers and estimate poses
  cv::cvtColor(color_mat, gray_mat, cv::COLOR_RGB2GRAY);
  if (target_aruco_.detectCharucoPose(gray_mat, Hcam_cvmat, false))
  {
    cv::namedWindow("detected", 0);
    cv::imshow("detected", gray_mat);
    cv::waitKey(0);
    cv::destroyWindow("detected");
  }

  // get video stream for real data test
	rs2::frameset frames;
  cv::Mat img;
  cv::namedWindow("rs_color_stream", 0);
  bool stop_flag = false;
  bool single_marker_mode = false; // true for detect single marker, otherwise detect marker chessboard's pose
  int marker_id = 0;  // only works in single marker mode

  while(!stop_flag)
  {
    // get color stream
    frames = pipe_.wait_for_frames();
    rs2::frame color_frame = frames.get_color_frame();
    img = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

    // detect marker / chessboard
    if(single_marker_mode)
    {
      target_aruco_.detectSinglePose(img, Hcam_cvmat, marker_id);
    }
    else
    {
      target_aruco_.detectCharucoPose(img, Hcam_cvmat, false);
    }
    cv::imshow("rs_color_stream", img);
    char c = cv::waitKey(50);
    switch (c)
    {
    case 'q': // press q to quit
      std::cout << "\n ->> stop signal detected, stop video stream" << std::endl;
      stop_flag = true; 
      break;
    case 's': // press s to switch mode (chessboard or single marker pose estimation)
      single_marker_mode = !single_marker_mode;
      std::cout << "\n ->> switch detection mode, single marker mode: " << single_marker_mode << std::endl;
      break;
    
    case 'u': // press up arrow key to increase marker id
      marker_id ++;
      std::cout << "\n ->> current single marker id: " << marker_id << std::endl;
      break;

    case 'd': // press down arrow key to increase marker id
      marker_id --;
      std::cout << "\n ->> current single marker id: " << marker_id << std::endl;
      break;

    }

  }
  cv::destroyWindow("rs_color_stream");
  return 0;
}


bool rs_connect(rs2::pipeline pipe_)
{
    rs2::config pipe_config;
    pipe_config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    pipe_config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    pipe_config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    rs2::pipeline_profile selection = pipe_.start(pipe_config);

    for (int i = 0; i < 30; i++)
    {
        auto frames = pipe_.wait_for_frames();   //Drop several frames for auto-exposure
    }

    // get intrinsic parameters
    rs2::frameset frames;
	  frames = pipe_.wait_for_frames();
    rs2::frame color = frames.get_color_frame();
    rs2::stream_profile cprofile = color.get_profile();
    rs2::video_stream_profile cvsprofile(cprofile);
    rs2_intrinsics color_intrin = cvsprofile.get_intrinsics();

    // distortion coeefs
    cam_info_D_.clear();
    for (auto value : color_intrin.coeffs)
    {
        cam_info_D_.push_back(value);
    }

    // camera matrix
    cam_info_K_ = { color_intrin.fx,    0.,     color_intrin.ppx, 
                    0.,    color_intrin.fy,     color_intrin.ppy, 
                    0.,                 0.,                   1. };

    return true;
}