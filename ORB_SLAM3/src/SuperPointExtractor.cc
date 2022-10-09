//
// Created by zhy on 6/20/22.
//

#include "SuperPointExtractor.h"
#include "dnn_node/util/output_parser/utils.h"
#include <fstream>
#include <atomic>
#include <opencv2/imgproc.hpp>


namespace ORB_SLAM3
{
SuperPointextractor::SuperPointextractor(
        int nfeatures, float scaleFactor, int nlevels, int iniThFAST, int minThFAST)
        : ORBextractor(nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST) {
          static std::atomic_int num{0};
          node_ = std::make_shared<SuperPointNode>("superpoint" + std::to_string(num++));
          threadpool_ = std::make_shared<hobot::CThreadPool>();
          threadpool_->CreatThread(3);
}

int SuperPointextractor::operator()(
        cv::InputArray image, cv::InputArray mask,
        std::vector<cv::KeyPoint> &keypoints, cv::OutputArray descriptors,
        std::vector<int> &vLappingArea) {
  cv::Mat image_mat = image.getMat();
  ComputePyramid(image_mat);
  std::vector<std::vector<cv::KeyPoint>> allKeypoints;
  std::vector<cv::Mat> descs(nlevels);
  allKeypoints.resize(nlevels);
  const float W = 35;
  std::atomic_int nkeypoints{0};
  std::vector<std::shared_ptr<std::promise<bool>>> promises;
  promises.reserve(nlevels);
  auto start = std::chrono::high_resolution_clock::now();
  for (int level = 0; level < nlevels; ++level) {
    auto extractor_kps_cpu = [&](int level) {
      const int minBorderX = EDGE_THRESHOLD - 3;
      const int minBorderY = minBorderX;
      const int maxBorderX = mvImagePyramid[level].cols - EDGE_THRESHOLD + 3;
      const int maxBorderY = mvImagePyramid[level].rows - EDGE_THRESHOLD + 3;
      const int scaledPatchSize = PATCH_SIZE * mvScaleFactor[level];
      std::vector<cv::KeyPoint> vToDistributeKeys;
      vToDistributeKeys.reserve(nfeatures * 10);
      const float width = (maxBorderX - minBorderX);
      const float height = (maxBorderY - minBorderY);

      const int nCols = width / W;
      const int nRows = height / W;
      const int wCell = std::ceil(width / nCols);
      const int hCell = std::ceil(height / nRows);

      for (int i = 0; i < nRows; i++) {
        const float iniY = minBorderY + i * hCell;
        float maxY = iniY + hCell + 6;

        if (iniY >= maxBorderY - 3)
          continue;
        if (maxY > maxBorderY)
          maxY = maxBorderY;

        for (int j = 0; j < nCols; j++) {
          const float iniX = minBorderX + j * wCell;
          float maxX = iniX + wCell + 6;
          if (iniX >= maxBorderX - 6)
            continue;
          if (maxX > maxBorderX)
            maxX = maxBorderX;

          std::vector<cv::KeyPoint> vKeysCell;

          cv::FAST(mvImagePyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX),
                   vKeysCell, iniThFAST, true);

          if (vKeysCell.empty()) {
            cv::FAST(mvImagePyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX),
                     vKeysCell, minThFAST, true);
          }

          if (!vKeysCell.empty()) {
            for (std::vector<cv::KeyPoint>::iterator vit = vKeysCell.begin();
                 vit != vKeysCell.end(); vit++) {
              (*vit).pt.x += j * wCell;
              (*vit).pt.y += i * hCell;
              vToDistributeKeys.push_back(*vit);
            }
          }
        }
      }

      {
        //std::lock_guard<std::mutex> mtx (allKeypoints_mtx_);
        allKeypoints[level] = DistributeOctTree(
          vToDistributeKeys, minBorderX, maxBorderX,
          minBorderY, maxBorderY,mnFeaturesPerLevel[level], level);
      }
      // Add border to coordinates and scale information
      const int nkps = allKeypoints[level].size();
      for(int i = 0; i < nkps ; i++) {
        if (level != superpoint_layer_) {
          allKeypoints[level][i].pt.x += minBorderX;
          allKeypoints[level][i].pt.y += minBorderY;
        }
        allKeypoints[level][i].octave = level;
        allKeypoints[level][i].size = scaledPatchSize;
      }
      nkeypoints += nkps;
      computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
      cv::Mat workingMat;
      cv::GaussianBlur(mvImagePyramid[level], workingMat, cv::Size(7, 7),
                       2, 2, cv::BORDER_REFLECT_101);
      descs[level] = cv::Mat(nkps, 32, CV_8U);
      computeDescriptors(workingMat, allKeypoints[level], descs[level], pattern);
    };

    if (level != superpoint_layer_) {
      auto promise = threadpool_->PostTask(std::bind(extractor_kps_cpu, level));
      promises.emplace_back(promise);
    } else if (level == superpoint_layer_) {
      auto extractor_kps_bpu = [&](int level) {
        const int minBorderX = EDGE_THRESHOLD - 3;
        const int minBorderY = minBorderX;
        const int maxBorderX = mvImagePyramid[level].cols - EDGE_THRESHOLD + 3;
        const int maxBorderY = mvImagePyramid[level].rows - EDGE_THRESHOLD + 3;
        const int scaledPatchSize = PATCH_SIZE * mvScaleFactor[level];
        const cv::Mat &image_model_input = mvImagePyramid[level];
        auto dnn_output = std::make_shared<SuperPointOutput>();
        PredictByImage(image_model_input, dnn_output);
        dnn_output->predict_promise_.get_future().get();
        const std::vector<std::shared_ptr<DNNResult>> &outputs = dnn_output->outputs;
        auto *extractor_result =
                dynamic_cast<SuperPointResult *>(outputs[0].get());
        if (dnn_output->width_ratio_ != 1.0f
            || dnn_output->hight_ratio_ != 1.0f) {
          for (auto &kp : extractor_result->keypoints_) {
            kp.pt.x *= dnn_output->width_ratio_;
            kp.pt.y *= dnn_output->hight_ratio_;
          }
        }
        {
          //std::lock_guard<std::mutex> mtx(allKeypoints_mtx_);
          allKeypoints[level] = extractor_result->keypoints_;
          // Add border to coordinates and scale information
          const int nkps = allKeypoints[level].size();
          for(int i = 0; i < nkps ; i++) {
            allKeypoints[level][i].octave = level;
            allKeypoints[level][i].size = scaledPatchSize;
          }
          nkeypoints += nkps;
          computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
          cv::Mat workingMat;
          cv::GaussianBlur(mvImagePyramid[level], workingMat, cv::Size(7, 7),
                           2, 2, cv::BORDER_REFLECT_101);
          descs[level] = cv::Mat(nkps, 32, CV_8U);
          computeDescriptors(workingMat, allKeypoints[level], descs[level], pattern);
        }
      };
      auto promise = threadpool_->PostTask(std::bind(extractor_kps_bpu, level));
      promises.emplace_back(promise);
//      for (const auto &kp : allKeypoints[level]) {
//        cv::circle(image_model_input, kp.pt, 3, cv::Scalar(0,255,0), -1);
//      }
//      static int toShowId = 0;
//      cv::imwrite("render_" + std::to_string(toShowId++) + ".png", image_model_input);
//      allKeypoints[level] = DistributeOctTree(
//              extractor_result->keypoints_, minBorderX, maxBorderX,
//              minBorderY, maxBorderY,mnFeaturesPerLevel[level], level);
    }
  }

  for (int level = 0; level < nlevels; ++level) {
    promises[level]->get_future().get();
  }
  auto end = std::chrono::high_resolution_clock::now();
//  std::cout << "cost desc: " << std::chrono::duration_cast<std::chrono::microseconds>(
//          end - start).count() << std::endl;
  cv::Mat descriptors_m;
  if (nkeypoints == 0) {
    descriptors.release();
    return 0;
  } else {
    descriptors.create(nkeypoints, 32, CV_8U);
    descriptors_m = descriptors.getMat();
  }
  keypoints = std::vector<cv::KeyPoint>(nkeypoints);
  int monoIndex = 0, stereoIndex = nkeypoints - 1;
  for (int level = 0; level < nlevels; ++level) {
    float scale = mvScaleFactor[level]; //getScale(level, firstLevel, scaleFactor);
    int i = 0;
    for (auto keypoint = allKeypoints[level].begin(),
                 keypointEnd = allKeypoints[level].end();
         keypoint != keypointEnd; ++keypoint) {
      if (level != 0) {
        keypoint->pt *= scale;
        //printf("x, y: %f, %f\n", keypoint->pt.x, keypoint->pt.y);
      }
      if (keypoint->pt.x >= vLappingArea[0] && keypoint->pt.x <= vLappingArea[1]){
        keypoints.at(stereoIndex) = (*keypoint);
        descs[level].row(i).copyTo(descriptors_m.row(stereoIndex));
        stereoIndex--;
      } else {
        keypoints.at(monoIndex) = (*keypoint);
        descs[level].row(i).copyTo(descriptors_m.row(monoIndex));
        monoIndex++;
      }
      i++;
    }
  }

  return 0;
}

int SuperPointextractor::PredictByImage(
        const cv::Mat &image,
        std::shared_ptr<SuperPointOutput> dnn_output) {
  return node_->PredictByImage(image, dnn_output);
}

SuperPointNode::SuperPointNode(const std::string &node_name,
        const rclcpp::NodeOptions &options) : DnnNode(node_name, options) {
  int ret = Start();
  if (ret != 0) {

  }
}

int SuperPointNode::PredictByImage(const cv::Mat &image,
        std::shared_ptr<SuperPointOutput> dnn_output) {
  auto inputs = std::vector<std::shared_ptr<DNNTensor>> {};
  auto dnntensor = new DNNTensor;
  auto input = std::shared_ptr<DNNTensor>(dnntensor,  [](DNNTensor *tensor){
    hbSysFreeMem(&tensor->sysMem[0]);
    delete tensor;
  });
  cv::Mat image_model_input;
  if (model_input_height_ != image.rows || model_input_width_ != image.cols || true) {
    cv::resize(image, image_model_input,
               cv::Size(model_input_width_, model_input_height_), cv::INTER_AREA);
    dnn_output->width_ratio_ = 1.0f * image.cols / model_input_width_;
    dnn_output->hight_ratio_ = 1.0f * image.rows / model_input_height_;
  } else {
    image_model_input = image;
  }
  auto timep1 = std::chrono::high_resolution_clock::now();
  auto *gray = &input->sysMem[0];
  gray->memSize = model_input_height_ * model_input_width_;
  hbSysAllocCachedMem(gray, gray->memSize);
  std::memcpy(gray->virAddr, image_model_input.data, gray->memSize);
  hbSysFlushMem(gray, HB_SYS_MEM_CACHE_CLEAN);
  input->properties = input_tensor_properties_;
  inputs.push_back(input);
  auto timep2 = std::chrono::high_resolution_clock::now();
  uint32_t ret = Predict(inputs, dnn_output);
  if (ret != 0) {
    RCLCPP_ERROR(get_logger(), "predict img failed!");
  }
  auto timep3 = std::chrono::high_resolution_clock::now();
  auto copy_data_time = std::chrono::duration_cast<
          std::chrono::microseconds>(timep2 - timep1).count();
  auto predict_time = std::chrono::duration_cast<
          std::chrono::milliseconds>(timep3 - timep2).count();
//  RCLCPP_WARN(get_logger(),
//              "copy_data_time: %d us, predict_time: %d ms", copy_data_time, predict_time);
  return ret;
}

int SuperPointNode::Start() {
  int ret = Init();
  if (ret != 0) {
    RCLCPP_ERROR(get_logger(), "Init failed!");
    return ret;
  }
  ret = GetModelInputSize(0, model_input_width_, model_input_height_);
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(),
                 "Get model input size fail!");
    return ret;
  }
  std::cout << "model input width: " << model_input_width_
            << ", height: " << model_input_height_ << std::endl;

  auto model_manage = GetModel();

  ret = model_manage->GetInputTensorProperties(
          input_tensor_properties_, 0);
  if (ret < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("SuperPointextractor"),
                 "Get model InputTensorProperties fail!");
    return ret;
  }
  output_tensor_properties_.resize(2);
  ret = model_manage->GetOutputTensorProperties(
          output_tensor_properties_[0], 0);
  ret = model_manage->GetOutputTensorProperties(
          output_tensor_properties_[1], 1);
  int width = output_tensor_properties_[0].alignedShape.dimensionSize[3];
  int height = output_tensor_properties_[0].alignedShape.dimensionSize[2];
  std::cout << "model aligned out w, h: " << width << ", " << height << std::endl;
  return 0;
}

int SuperPointNode::SetNodePara() {
  if (!dnn_node_para_ptr_) {
    RCLCPP_ERROR(rclcpp::get_logger("SuperPointextractor"), "Please use Start() first!");
  }
  dnn_node_para_ptr_->model_file = model_file_name_;
  dnn_node_para_ptr_->model_name = model_name_;
  dnn_node_para_ptr_->model_task_type = model_task_type_;
  dnn_node_para_ptr_->task_num = 1;
  return 0;
}

int SuperPointNode::PostProcess(
        const std::shared_ptr<DnnNodeOutput> &node_output) {
  auto superpoint_output =
          std::dynamic_pointer_cast<SuperPointOutput>(node_output);
  auto &promise = superpoint_output->predict_promise_;
  promise.set_value(true);
  return 0;
}

int SuperPointNode::SetOutputParser() {
  auto model_manage = GetModel();
  if (!model_manage || !dnn_node_para_ptr_) {
    RCLCPP_ERROR(rclcpp::get_logger("SuperPointextractor"),
                 "Invalid model, Please use Start() first");
    return -1;
  }
  for (int i = 0; i < output_index_; ++i) {
    std::shared_ptr<OutputParser> assist_parser =
            std::make_shared<SuperPointAssistParser>();
    model_manage->SetOutputParser(i, assist_parser);
  }

  auto output_desc = std::make_shared<OutputDescription>(
          model_manage, output_index_, "SuperPoint_branch");
  for (int i = 0; i < output_index_; ++i) {
    output_desc->GetDependencies().push_back(i);
  }

  output_desc->SetType("SuperPoint");
  model_manage->SetOutputDescription(output_desc);
  std::shared_ptr<OutputParser> out_parser =
          std::make_shared<SuperPointOutputParser>(std::string());
  model_manage->SetOutputParser(output_index_, out_parser);
  return 0;
}

int SuperPointNode::Predict(
        std::vector<std::shared_ptr<DNNTensor>> &inputs,
        std::shared_ptr<DnnNodeOutput> dnn_output) {
  std::vector<std::shared_ptr<OutputDescription>> output_descs;
  int ret = Run(inputs, output_descs, dnn_output, is_sync_mode_);
  return ret;
}

int32_t SuperPointOutputParser::Parse(
        std::shared_ptr<SuperPointResult>& output,
        std::vector<std::shared_ptr<InputDescription>>& input_descriptions,
        std::shared_ptr<OutputDescription>& output_description,
        std::shared_ptr<DNNTensor>& output_tensor) {
  std::shared_ptr<SuperPointResult> result;
  if (!output) {
    result = std::make_shared<SuperPointResult>();
    result->Reset();
    output = result;
  } else {
    result = std::dynamic_pointer_cast<SuperPointResult>(output);
    result->Reset();
  }
  std::vector<std::shared_ptr<DNNTensor>> depend_output_tensors {output_tensor};
  int ret = PostProcess(depend_output_tensors, result->keypoints_, result->descriptors_);
  if (ret != 0) {
    RCLCPP_INFO(rclcpp::get_logger("SuperPointExtractor"),
                "postprocess return error, code = %d", ret);
  }
  return ret;
}

//int32_t SuperPointOutputParser::Parse(
//        std::shared_ptr<DNNResult> &output,
//        std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
//        std::shared_ptr<OutputDescription> &output_descriptions,
//        std::shared_ptr<DNNTensor> &output_tensor,
//        std::vector<std::shared_ptr<OutputDescription>> &depend_output_descs,
//        std::vector<std::shared_ptr<DNNTensor>> &depend_output_tensors,
//        std::vector<std::shared_ptr<DNNResult>> &depend_outputs) {
//  std::shared_ptr<SuperPointResult> result;
//  if (!output) {
//    result = std::make_shared<SuperPointResult>();
//    result->Reset();
//    output = result;
//  } else {
//    result = std::dynamic_pointer_cast<SuperPointResult>(output);
//    result->Reset();
//  }
//
//  int ret = PostProcess(depend_output_tensors, result->keypoints_, result->descriptors_);
//  if (ret != 0) {
//    RCLCPP_INFO(rclcpp::get_logger("SuperPointExtractor"),
//                "postprocess return error, code = %d", ret);
//  }
//  return ret;
//}

void NmsFast(std::vector<cv::KeyPoint>& kps, std::vector<cv::KeyPoint> &kps_nms,
        int dist_thresh, int img_width, int img_height) {
  kps_nms.reserve(kps.size());
  cv::Mat grid = cv::Mat(cv::Size(img_width, img_height), CV_8UC1);
  cv::Mat inds = cv::Mat(cv::Size(img_width, img_height), CV_16UC1);

  grid.setTo(0);
  inds.setTo(0);

  for (int i = 0; i < kps.size(); i++) {
    int uu = (int) kps[i].pt.x;
    int vv = (int) kps[i].pt.y;
    if (uu < dist_thresh || vv < dist_thresh
    ||  uu >= img_width - dist_thresh || vv >= img_height - dist_thresh) {
      continue;
    }
    grid.at<char>(vv, uu) = 1;
    inds.at<unsigned short>(vv, uu) = i;
  }

  for (int i = 0; i < kps.size(); i++) {
    int uu = (int) kps[i].pt.x;
    int vv = (int) kps[i].pt.y;
    if (grid.at<char>(vv, uu) != 1)
      continue;
    float current_score = kps[i].response;
    for (int k = -dist_thresh; k < dist_thresh; k++) {
      for (int j = -dist_thresh; j < dist_thresh; j++) {
        if (j == 0 && k == 0) {
          continue;
        }
        float dist_score =
                kps[inds.at<unsigned short>(vv + k, uu + j)].response;
        if (dist_score < current_score) {
          grid.at<char>(vv + k, uu + j) = 0;
        }
      }
    }
    kps_nms.push_back(kps[i]);
    grid.at<char>(vv, uu) = 2;
  }
}

void SuperPointOutputParser::GetHeatMap(float *data, float *out) {
  uint32_t N = 1;
  uint32_t H = height_size_[0];
  uint32_t W = width_size_[0];
  uint32_t C_C = channel_size_[0];
  uint32_t src_n_stride = H * W * C_C;
  uint32_t src_h_stride = W * C_C;
  uint32_t src_c_stride = H * W;
  float *nhwc_data = new float[src_n_stride];
  for (uint32_t nn = 0; nn < N; nn++) {
    uint32_t nsrc_n_stride = nn * src_n_stride;
    for (uint32_t hh = 0; hh < H; hh++) {
      uint32_t h_src_h_stride = hh * src_h_stride;
      for (uint32_t ww = 0; ww < W; ww++) {
        uint32_t w_src_w_stride = ww * C_C;
        for (uint32_t cc = 0; cc < C_C; cc++) {
          uint32_t nhwc_index = cc + nsrc_n_stride + h_src_h_stride + w_src_w_stride;
          uint32_t nchw_index = cc * src_c_stride + nsrc_n_stride + ww + hh * W;
          nhwc_data[nhwc_index] = data[nchw_index];
        }
      }
    }
  }
  for (uint32_t nn = 0; nn < N; nn++) {
    float *cur_n_src = nhwc_data + nn * src_n_stride;
    float *cur_n_dst = out + nn * src_n_stride;
    for (uint32_t hh = 0; hh < H; hh++) {
      uint32_t h_src_h_stride = hh * src_h_stride;
      for (uint32_t ww = 0; ww < W; ww++) {
        float *cur_src = cur_n_src + h_src_h_stride + ww * C_C;
        float *cur_dst = cur_n_dst + h_src_h_stride + ww * C_C;
        SoftmaxCore(cur_src, C_C, cur_dst);
      }
    }
  }
  delete []nhwc_data;
}

int SuperPointOutputParser::PostProcess(
        std::vector<std::shared_ptr<DNNTensor>> &tensors,
        std::vector<cv::KeyPoint> &keypoints,
        cv::Mat &descriptors) {
  ParameterInit(tensors);
  size_t out_layer = tensors.size();
  //  PostProcess Keypoint
  std::vector<cv::KeyPoint> kps;
  std::vector<cv::KeyPoint> &kps_nms = keypoints;
  kps.reserve(500);
  // {
    hbSysFlushMem(&(tensors[0]->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
    uint32_t H = height_size_[0];
    uint32_t W = width_size_[0];
    uint32_t C_C = channel_size_[0];
    uint32_t C = cell_size_;
    static uint32_t image_width = W * C;
    static uint32_t image_height = H * C;
    const uint32_t kHWC = H * W * C;
    const uint32_t kWC = W * C;
    const uint32_t kHW = H * W;
    float *elements = new float[kHW * C_C];
    float *data = static_cast<float *>(tensors[0]->sysMem[0].virAddr);
    auto timep1 = std::chrono::high_resolution_clock::now();
    GetHeatMap(data, elements);
    auto timep2 = std::chrono::high_resolution_clock::now();
    int border = 32;
    for (uint32_t h = 0; h < H; ++h) {
      for (uint32_t hc = 0; hc < cell_size_; ++hc) {
        uint32_t h_index = h * cell_size_ + hc;
        for (uint32_t w = 0; w < W; ++w) {
          for (uint32_t wc = 0; wc < cell_size_; ++wc) {
            uint32_t w_index = w * cell_size_ + wc;
            float response =
                    elements[wc + w * C_C + hc * cell_size_ + h * W * C_C];
            if (response >= conf_thresh_ && w_index >= border && w_index < image_width - border
             && h_index >= border && h_index < image_height - border) {
              kps.emplace_back(cv::KeyPoint(w_index, h_index, 8, -1, response));
            }
          }
        }
      }
    }
    auto timep3 = std::chrono::high_resolution_clock::now();
    delete []elements;
  //  }
  std::sort(kps.begin(), kps.end(),
          [](const cv::KeyPoint &a, const cv::KeyPoint &b) {
    return a.response > b.response;
  });
  NmsFast(kps, kps_nms, nms_dist_,
          W * cell_size_, H * cell_size_);
//  for (auto &kp : kps_nms) {
//    kp.pt.x = kp.pt.x * 2;
//    kp.pt.y = kp.pt.y * 2;
//  }
//  std::cout << "kps_size: " << kps.size() << std::endl;
//  std::cout << "kps_nms_size: " << kps_nms.size() << std::endl;
  auto timep4 = std::chrono::high_resolution_clock::now();
  bool use_superpoint_descriptors = false;
  //  PostProcess Keypoint
  if (use_superpoint_descriptors) {
    hbSysFlushMem(&(tensors[1]->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
    static int32_t H = height_size_[1];
    static int32_t W = width_size_[1];
    static int32_t C = channel_size_[1];
    //  static const int32_t kHWC = H * W * C;
    //  static const int32_t kWC = W * C;
    static const int32_t kHW = H * W;
    auto* desc_data = static_cast<float*>(tensors[1]->sysMem[0].virAddr);
    std::vector<float> desc;
    desc.reserve(kps_nms.size() * C);
    cv::Mat desc_mat(cv::Size(C, kps_nms.size()), CV_32FC1, desc.data());
    //  cv::Mat image = cv::imread("./config/images/250.png", -1);
    for (int32_t k = 0; k < kps_nms.size(); ++k) {
      const auto &kp = kps_nms[k];
      //  cv::circle(image, kp.pt * 2, 3, cv::Scalar(0,255,0), -1);
      int32_t x = kp.pt.x / cell_size_;
      int32_t y = kp.pt.y / cell_size_;
      std::vector<float> one_desc(C);
      float sum{0.};
      int location =  y * W + x;
      for (int32_t i = 0; i < C; ++i) {
        one_desc[i] = desc_data[i * kHW + location];
        sum += one_desc[i] * one_desc[i];
      }
      sum = std::sqrt(sum);
      for (int32_t i = 0; i < C; ++i) {
        desc.push_back(one_desc[i] / sum);
      }
    }
    descriptors = desc_mat.clone();
    //  cv::imwrite("./result.png", image);
  } else {

  }
  auto timep5 = std::chrono::high_resolution_clock::now();
  auto softmax_time = std::chrono::duration_cast<
          std::chrono::milliseconds>(timep2 - timep1).count();
  auto getkeypt_time = std::chrono::duration_cast<
          std::chrono::milliseconds>(timep3 - timep2).count();
  auto nms_time = std::chrono::duration_cast<
          std::chrono::milliseconds>(timep4 - timep3).count();
  auto desc_time = std::chrono::duration_cast<
          std::chrono::milliseconds>(timep5 - timep4).count();
//  printf("softmax_time: %d ms, getkeypt_time: %d ms, nms_time: %d, desc_time: %d\n",
//          softmax_time, getkeypt_time, nms_time, desc_time);
  return 0;
}

void SuperPointOutputParser::ParameterInit(
        std::vector<std::shared_ptr<DNNTensor>> &tensors) {
  if (!is_parameter_init_) {
    int h_idx, w_idx, c_idx;
    hobot::dnn_node::output_parser::get_tensor_hwc_index(tensors[0], &h_idx, &w_idx, &c_idx);
    model_output_width_ =
            tensors[0]->properties.validShape.dimensionSize[w_idx];
    model_output_height_ =
            tensors[0]->properties.validShape.dimensionSize[h_idx];
    std::cout << "model out width: " << model_output_width_
              << ", height: " << model_output_height_ << std::endl;
    size_t out_layer = tensors.size();
    output_shifts_.resize(out_layer);
    for (size_t layer = 0; layer < out_layer; layer++) {
      int32_t tensordatatype =
              tensors[layer]->properties.tensorType;
      std::cout << "model output type: " << tensordatatype << std::endl;
      int h_idx, w_idx, c_idx;
      hobot::dnn_node::output_parser::get_tensor_hwc_index(tensors[layer], &h_idx, &w_idx, &c_idx);
      std::cout << "model out hidx:" << h_idx << ", w_idx:" << w_idx
                << ", c_idx:" << c_idx << std::endl;
      channel_size_.push_back(
              tensors[layer]->properties.validShape.dimensionSize[c_idx]);
      aligned_channel_size_.push_back(
              tensors[layer]->properties.alignedShape.dimensionSize[c_idx]);

      width_size_.push_back(
              tensors[layer]->properties.validShape.dimensionSize[w_idx]);
      aligned_width_size_.push_back(
              tensors[layer]->properties.alignedShape.dimensionSize[w_idx]);

      height_size_.push_back(
              tensors[layer]->properties.validShape.dimensionSize[h_idx]);
      aligned_height_size_.push_back(
              tensors[layer]->properties.alignedShape.dimensionSize[h_idx]);

      std::cout << "channel_size_ push:"
                << tensors[layer]->properties.validShape.dimensionSize[c_idx]
                << std::endl;
      std::cout << "aligned_channel_size_ push:"
                << tensors[layer]->properties.alignedShape.dimensionSize[c_idx]
                << std::endl;

      std::cout << "width_size_ push:"
                << tensors[layer]->properties.validShape.dimensionSize[w_idx]
                << std::endl;
      std::cout << "aligned_width_size_ push:"
                << tensors[layer]->properties.alignedShape.dimensionSize[w_idx]
                << std::endl;

      std::cout << "height_size_ push:"
                << tensors[layer]->properties.validShape.dimensionSize[h_idx]
                << std::endl;
      std::cout << "aligned_height_size_ push:"
                << tensors[layer]->properties.alignedShape.dimensionSize[h_idx]
                << std::endl;
      std::cout << "shiftData len:" << tensors[layer]->properties.shift.shiftLen
                << std::endl;
      // std::cout << "shiftData:" <<
      // tensors[layer]->properties.shift.shiftData<< std::endl;
      std::cout << "shiftData channel_size_:" << channel_size_[layer]
                << std::endl;
      if (tensors[layer]->properties.shift.shiftLen != 0) {
        output_shifts_[layer].assign(
                tensors[layer]->properties.shift.shiftData,
                tensors[layer]->properties.shift.shiftData + channel_size_[layer]);
      }
    }
    is_parameter_init_ = true;
  }
}

}
