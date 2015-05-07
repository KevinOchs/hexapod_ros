#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <libfovis/visual_odometry.hpp>
#include <iostream>

#include "visualization.hpp"


void _drawPyramidLevelMatches(const fovis::VisualOdometry* odometry, int level, 
    cv::Mat& canvas)
{
  using namespace fovis;

  cv::Mat target_canvas(canvas.rowRange(0, canvas.rows/2));
  cv::Mat reference_canvas(canvas.rowRange(canvas.rows/2, canvas.rows));
 
  const PyramidLevel* reference_level = odometry->getReferenceFrame()->getLevel(0);
  for (int i = 0; i < reference_level->getNumKeypoints(); ++i)
  {
    const KeyPoint& kp = reference_level->getKeypoint(i);
    cv::Point2f center(kp.u, kp.v);
    cv::circle(reference_canvas, center, (level+1)*10, cv::Scalar(255, 0, 0));
  }
  const PyramidLevel* target_level = odometry->getTargetFrame()->getLevel(0);
  for (int i = 0; i < target_level->getNumKeypoints(); ++i)
  {
    const KeyPoint& kp = target_level->getKeypoint(i);
    cv::Point2f center(kp.u, kp.v);
    cv::circle(target_canvas, center, (level+1)*10, cv::Scalar(255, 0, 0));
  }
}

void _drawMatch(const fovis::FeatureMatch& match, cv::Mat& canvas)
{
  using namespace fovis;
  cv::Mat target_canvas(canvas.rowRange(0, canvas.rows/2));
  cv::Mat reference_canvas(canvas.rowRange(canvas.rows/2, canvas.rows));
  const KeyPoint& target_keypoint = match.target_keypoint->kp;
  const KeyPoint& ref_keypoint = match.ref_keypoint->kp;
  int target_level = match.target_keypoint->pyramid_level;
  int ref_level = match.ref_keypoint->pyramid_level;
  cv::Point2f ref_center(
      ref_keypoint.u*(ref_level+1), ref_keypoint.v*(ref_level+1));
  cv::Point2f target_center(
      target_keypoint.u*(target_level+1), target_keypoint.v*(target_level+1));
  cv::Scalar color(0, 255, 0);
  if (!match.inlier)
    color = cv::Scalar(0, 0, 255);
  cv::circle(reference_canvas, ref_center, 
      (match.ref_keypoint->pyramid_level+1)*10, color);
  cv::circle(target_canvas, target_center, 
      (match.target_keypoint->pyramid_level+1)*10, color);
  cv::Point2f global_ref_center(ref_center.x, ref_center.y + canvas.rows/2);
  cv::line(canvas, target_center, global_ref_center, color);
  // motion flow
  // cv::line(canvas, target_center, ref_center, color);
}

void _drawKeypoint(const fovis::KeypointData& kp_data, cv::Mat& canvas)
{
  cv::Point2f center(kp_data.rect_base_uv.x(), kp_data.rect_base_uv.y());
  cv::Scalar color;
  if (kp_data.has_depth)
  {
    color = cv::Scalar(255, 0, 0);
  }
  else
  {
    color = cv::Scalar(0, 0, 0);
  }
  cv::circle(canvas, center, (kp_data.pyramid_level+1)*10, color);
}

template<typename T>
std::string toStr(T t)
{
  std::stringstream ss;
  ss << t;
  return ss.str();
}

std::vector<std::string> _createInfoStrings(const fovis::VisualOdometry* odometry)
{
  std::vector<std::string> infostrings;
  infostrings.push_back(std::string("Status: ") + fovis::MotionEstimateStatusCodeStrings[odometry->getMotionEstimateStatus()]);
  infostrings.push_back(toStr(odometry->getTargetFrame()->getNumDetectedKeypoints()) + " keypoints");
  infostrings.push_back(toStr(odometry->getTargetFrame()->getNumKeypoints()) + " filtered keypoints");
  infostrings.push_back(toStr(odometry->getMotionEstimator()->getNumMatches()) + " matches");
  infostrings.push_back(toStr(odometry->getMotionEstimator()->getNumInliers()) + " inliers");
  return infostrings;
}

cv::Mat fovis_ros::visualization::paint(const fovis::VisualOdometry* odometry)
{
  using namespace fovis;
  const OdometryFrame* reference_frame = odometry->getReferenceFrame();
  const OdometryFrame* target_frame = odometry->getTargetFrame();

  int width = target_frame->getLevel(0)->getWidth();
  int height = target_frame->getLevel(0)->getHeight();

  // We have to const cast here because there is no 
  // cv::Mat constructor for const data.
  // The data will be copied later anyways.
  const cv::Mat reference_image(height, width, CV_8U, 
      const_cast<unsigned char*>(
        reference_frame->getLevel(0)->getGrayscaleImage()));
  const cv::Mat target_image(height, width, CV_8U,
      const_cast<unsigned char*>(
        target_frame->getLevel(0)->getGrayscaleImage()),
      target_frame->getLevel(0)->getGrayscaleImageStride());

  cv::Mat canvas(2*height, width, CV_8U);
  cv::Mat upper_canvas(canvas.rowRange(0, height));
  cv::Mat lower_canvas(canvas.rowRange(height, 2*height));
  target_image.copyTo(upper_canvas);
  reference_image.copyTo(lower_canvas);
  cv::cvtColor(canvas, canvas, CV_GRAY2BGR);

  for (int level = 0; level < reference_frame->getNumLevels(); ++level)
  {
    const PyramidLevel* pyramid_level = reference_frame->getLevel(level);
    for (int i = 0; i < pyramid_level->getNumKeypoints(); ++i)
    {
      _drawKeypoint(*(pyramid_level->getKeypointData(i)), canvas);
    }
  }
      
  const MotionEstimator* motion_estimator = odometry->getMotionEstimator(); 
  for (int i = 0; i < motion_estimator->getNumMatches(); ++i)
  {
    _drawMatch(motion_estimator->getMatches()[i], canvas);
  }
  std::vector<std::string> infostrings = _createInfoStrings(odometry);
  for (size_t i = 0; i < infostrings.size(); ++i)
  {
    cv::putText(canvas, infostrings[i], cv::Point(10, 40*(i + 1)),
          CV_FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 3);
  }
  return canvas;
}


