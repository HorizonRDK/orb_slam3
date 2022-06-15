#include "ORBVocabulary.h"
#include "ORBextractor.h"

#include <vector>
#include <iostream>

void ChangeStructure(const cv::Mat &plain, vector<cv::Mat> &out) {
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

void LoadImages(const string &strImagePath, const string &strPathTimes,
                std::vector<string> &vstrImages,
                std::vector<double> &vTimeStamps) {
  ifstream fTimes;
  fTimes.open(strPathTimes.c_str());
  vTimeStamps.reserve(5000);
  vstrImages.reserve(5000);
  while(!fTimes.eof()) {
    string s;
    getline(fTimes,s);
    if(!s.empty()) {
      stringstream ss;
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
  std::vector<std::vector<cv::Mat>> features;
  ORB_SLAM3::ORBextractor *extractor = nullptr;
  auto extractor_type = ORB_SLAM3::ORBextractor::EXTRACTOR_TYPE::ORB;
  extractor = ORB_SLAM3::ORBextractor::make_extractor(
          800, 1.2, 8, 25, 12,
          extractor_type);
  if (!extractor) {
    std::cout << "unsupport extractor type: " << extractor_type << std::endl;
    return -1;
  }
  std::vector<int> lapping = {0, 0};
  string images_directory = std::string(argv[1]) + "/cam0";
  string timestamp_file = std::string(argv[2]);
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
    LoadFeatures(features, descriptors);
    if (++cnt % 100 == 0 || cnt >= image_size) {
      std::cout << "extractor image... "
                << cnt << "/" << image_size << std::endl;
    }
  }

  const int k = 10;
  const int L = 6;
  const DBoW2::WeightingType weight = DBoW2::TF_IDF;
  const DBoW2::ScoringType scoring = DBoW2::L1_NORM;
  ORB_SLAM3::ORBVocabulary voc(k, L, weight, scoring);
  CreateVocabularyFile(voc, std::string(argv[3]), features);
  return 0;
}