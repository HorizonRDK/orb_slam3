#include "ORBVocabulary.h"
#include "ORBextractor.h"

#include <vector>
#include <iostream>
#include <csignal>

#ifdef SUPPORT_SUPERPOINT
#include <rclcpp/rclcpp.hpp>
#endif

bool b_continue_session;

void exit_loop_handler(int s) {
  std::cout << "Finishing session" << std::endl;
  b_continue_session = false;
}

void ChangeStructure(const cv::Mat &plain, std::vector<cv::Mat> &out) {
  out.resize(plain.rows);
  for(int i = 0; i < plain.rows; ++i) {
    out[i] = plain.row(i);
  }
}

void LoadFeatures(std::vector<std::vector<cv::Mat>> &features,
        const cv::Mat &descriptors) {
  features.emplace_back();
  ChangeStructure(descriptors, features.back());
}

void CreateVocabularyFile(
        ORB_SLAM3::ORBVocabulary &voc, const std::string &fileName,
        const std::vector<std::vector<cv::Mat>> &features) {
  std::cout << "> Creating vocabulary. May take some time ..." << std::endl;
  voc.create(features);
  std::cout << "... done!" << std::endl;

  std::cout << "> Vocabulary information: " << std::endl
       << voc << std::endl << std::endl;

  // save the vocabulary to disk
  std::cout << std::endl << "> Saving vocabulary..." << std::endl;
  voc.saveToTextFile(fileName);
  std::cout << "... saved to file: " << fileName << std::endl;
}

void LoadImages(const std::string &strImagePath, const std::string &strPathTimes,
                std::vector<std::string> &vstrImages,
                std::vector<double> &vTimeStamps) {
  std::ifstream fTimes;
  fTimes.open(strPathTimes.c_str());
  vTimeStamps.reserve(5000);
  vstrImages.reserve(5000);
  while(!fTimes.eof()) {
    std::string s;
    getline(fTimes,s);
    if(!s.empty()) {
      std::stringstream ss;
      ss << s;
      vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
      double t;
      ss >> t;
      vTimeStamps.push_back(t*1e-9);
    }
  }
}

int main(int argc, char **argv) {
  if(argc < 4) {
    std::cout << "Usage: ./create_vocabulary"
                 " path_to_images "
                 " path_to_timestamp "
                 " path_to_vocabulary" << std::endl;
    return -1;
  }
#ifdef SUPPORT_SUPERPOINT
  rclcpp::init(argc, argv);
#endif

  struct sigaction sigIntHandler {};

  sigIntHandler.sa_handler = exit_loop_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);
  b_continue_session = true;

  std::vector<std::vector<cv::Mat>> features;
  ORB_SLAM3::ORBextractor *extractor = nullptr;
  auto extractor_type = ORB_SLAM3::ORBextractor::EXTRACTOR_TYPE::SUPERPOINT;
  extractor = ORB_SLAM3::ORBextractor::make_extractor(
          800, 1.0, 1, 25, 12,
          extractor_type);
  if (!extractor) {
    std::cout << "unsupport extractor type: " << extractor_type << std::endl;
    return -1;
  }
  std::vector<int> lapping = {0, 0};
  std::string images_directory = std::string(argv[1]);
  std::string timestamp_file = std::string(argv[2]);
  std::vector<std::string> images;
  std::vector<double> timestamps;
  LoadImages(images_directory, timestamp_file, images, timestamps);
  unsigned int cnt = 0;
  auto image_size = images.size();
  for (const auto &image : images) {
    auto image_mat = cv::imread(image, cv::IMREAD_UNCHANGED);
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    if (image_mat.channels() == 3) {
      cv::cvtColor(image_mat, image_mat, cv::COLOR_RGB2GRAY);
    }
    (*extractor)(image_mat, cv::Mat(),
                        keypoints, descriptors, lapping);
    //  std::cout << descriptors << std::endl;
    LoadFeatures(features, descriptors);

    std::cout << "extractor image... "
              << ++cnt << "/" << image_size << std::endl;
    if (!b_continue_session) {
      return -1;
    }
  }

  const int k = 10;
  const int L = 6;

#ifdef SUPPORT_DBOW3
  const DBoW3::WeightingType weight = DBoW3::TF_IDF;
  const DBoW3::ScoringType scoring = DBoW3::L1_NORM;
#else
  const DBoW2::WeightingType weight = DBoW2::TF_IDF;
  const DBoW2::ScoringType scoring = DBoW2::L1_NORM;
#endif
  ORB_SLAM3::ORBVocabulary voc(k, L, weight, scoring);
  CreateVocabularyFile(voc, std::string(argv[3]), features);
  return 0;
}
