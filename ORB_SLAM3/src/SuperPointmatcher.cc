//
// Created by zhy on 7/4/22.
//

#include <opencv2/core/core.hpp>

int SuperPointmatcher::DescriptorDistance(
        const cv::Mat &a, const cv::Mat &b) {
  return cv::norm(a, b, cv::NORM_L2) * 100;
}
