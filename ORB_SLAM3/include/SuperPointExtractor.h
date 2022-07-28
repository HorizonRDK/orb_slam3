//
// Created by zhy on 6/20/22.
//

#include "ORBextractor.h"

#include "dnn_node/dnn_node.h"
#include "dnn_node/dnn_node_data.h"

#include "dnn/hb_dnn_ext.h"
#include "easy_dnn/data_structure.h"
#include "easy_dnn/description.h"
#include "easy_dnn/model.h"
#include "easy_dnn/output_parser.h"

#include <opencv2/imgcodecs.hpp>
#include <arm_neon.h>

#include <future>
#include <rclcpp/rclcpp.hpp>

#include "threadpool.h"

#ifndef SUPERPOINTEXTRACTOR_H
#define SUPERPOINTEXTRACTOR_H

using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::DnnNodePara;
using hobot::dnn_node::ModelTaskType;
using hobot::dnn_node::TaskId;

using hobot::easy_dnn::DNNTensor;
using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DNNResult;
using hobot::dnn_node::NV12PyramidInput;

using hobot::easy_dnn::OutputDescription;
using hobot::easy_dnn::InputDescription;
using hobot::easy_dnn::OutputParser;
using hobot::easy_dnn::SingleBranchOutputParser;
using hobot::easy_dnn::MultiBranchOutputParser;

namespace ORB_SLAM3
{

#define c_exp_hi 88.3762626647949f
#define c_exp_lo -88.3762626647949f

#define c_cephes_LOG2EF 1.44269504088896341f
#define c_cephes_exp_C1 0.693359375f
#define c_cephes_exp_C2 -2.12194440e-4f

#define c_cephes_exp_p0 1.9875691500E-4f
#define c_cephes_exp_p1 1.3981999507E-3f
#define c_cephes_exp_p2 8.3334519073E-3f
#define c_cephes_exp_p3 4.1665795894E-2f
#define c_cephes_exp_p4 1.6666665459E-1f
#define c_cephes_exp_p5 5.0000001201E-1f

/* exp() computed for 4 float32_t at once */
static float32x4_t exp_ps(float32x4_t x) {
  float32x4_t tmp;
  float32x4_t fx;

  float32x4_t const one{vdupq_n_f32(1.f)};
  x = vminq_f32(x, vdupq_n_f32(c_exp_hi));
  x = vmaxq_f32(x, vdupq_n_f32(c_exp_lo));

  /* "express exp(x) as exp(g + n*log(2))" */
  fx = vmlaq_f32(vdupq_n_f32(0.5f), x, vdupq_n_f32(c_cephes_LOG2EF));

  /* perform a floorf */
  tmp = vcvtq_f32_s32(vcvtq_s32_f32(fx));

  /* if greater, substract 1 */
  uint32x4_t mask{vcgtq_f32(tmp, fx)};
  mask = vandq_u32(mask, vreinterpretq_u32_f32(one));

  fx = vsubq_f32(tmp, vreinterpretq_f32_u32(mask));

  tmp = vmulq_f32(fx, vdupq_n_f32(c_cephes_exp_C1));
  float32x4_t z{vmulq_f32(fx, vdupq_n_f32(c_cephes_exp_C2))};
  x = vsubq_f32(x, tmp);
  x = vsubq_f32(x, z);

  static float32_t const cephes_exp_p[6]{c_cephes_exp_p0, c_cephes_exp_p1,
                                         c_cephes_exp_p2, c_cephes_exp_p3,
                                         c_cephes_exp_p4, c_cephes_exp_p5};
  float32x4_t y{vld1q_dup_f32(cephes_exp_p + 0)};
  float32x4_t const c1{vld1q_dup_f32(cephes_exp_p + 1)};
  float32x4_t const c2{vld1q_dup_f32(cephes_exp_p + 2)};
  float32x4_t const c3{vld1q_dup_f32(cephes_exp_p + 3)};
  float32x4_t const c4{vld1q_dup_f32(cephes_exp_p + 4)};
  float32x4_t const c5{vld1q_dup_f32(cephes_exp_p + 5)};

  y = vmulq_f32(y, x);
  z = vmulq_f32(x, x);

  y = vaddq_f32(y, c1);
  y = vmulq_f32(y, x);
  y = vaddq_f32(y, c2);
  y = vmulq_f32(y, x);
  y = vaddq_f32(y, c3);
  y = vmulq_f32(y, x);
  y = vaddq_f32(y, c4);
  y = vmulq_f32(y, x);
  y = vaddq_f32(y, c5);

  y = vmulq_f32(y, z);
  y = vaddq_f32(y, x);
  y = vaddq_f32(y, one);

  /* build 2^n */
  int32x4_t mm;
  mm = vcvtq_s32_f32(fx);
  mm = vaddq_s32(mm, vdupq_n_s32(0x7f));
  mm = vshlq_n_s32(mm, 23);
  float32x4_t const pow2n{vreinterpretq_f32_s32(mm)};

  y = vmulq_f32(y, pow2n);
  return y;
}

static int32_t SoftmaxCore(float32_t const* const in,
                                  uint32_t const length, float32_t* const out) {
  float32_t mmax{in[0]};
  for (uint32_t i{1U}; i < length; ++i) {
    if (mmax < in[i]) {
      mmax = in[i];
    }
  }
  float32x4_t const max_v{vdupq_n_f32(mmax)};
  float32x4_t sum_v{vdupq_n_f32(0.f)};
  uint32_t const nn{(length >> 2) << 2};
  for (uint32_t i{0U}; i < nn; i += 4U) {
    float32x4_t in_v{vld1q_f32(in + i)};
    in_v = vsubq_f32(in_v, max_v);
    float32x4_t const out_v{exp_ps(in_v)};
    vst1q_f32(out + i, out_v);
    sum_v = vaddq_f32(sum_v, out_v);
  }
  float32x2_t sum_p{vpadd_f32(vget_high_f32(sum_v), vget_low_f32(sum_v))};
  sum_p = vpadd_f32(sum_p, sum_p);
  float32_t sum{vget_lane_f32(sum_p, 0)};
  for (uint32_t i{nn}; i < length; ++i) {
    out[i] = std::exp(in[i] - mmax);
    sum += out[i];
  }

  float32x4_t const div_v{vdupq_n_f32(1.0f / sum)};
  for (uint32_t i{0U}; i < nn; i += 4U) {
    float32x4_t out_v{vld1q_f32(out + i)};
    out_v = vmulq_f32(out_v, div_v);
    vst1q_f32(out + i, out_v);
  }
  for (uint32_t i{nn}; i < length; ++i) {
    out[i] /= sum;
  }
  return 0;
}

struct SuperPointOutput : public DnnNodeOutput {
public:
  std::promise<bool> predict_promise_;
};

static void NmsFast(std::vector<cv::KeyPoint>& pts, std::vector<cv::KeyPoint> &kps_nms,
                    int dist_thresh, int img_width, int img_height);

struct SuperPointNode : public DnnNode {
  explicit SuperPointNode(const std::string &node_name,
                 const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~SuperPointNode() {}

  int PredictByImage(const cv::Mat &image,
          std::shared_ptr<SuperPointOutput> dnn_output);

protected:
  int SetNodePara() override;
  int SetOutputParser() override;
  int PostProcess(const std::shared_ptr<DnnNodeOutput> &outputs) override;

private:
  void GetConfig();
  int Start();
  int Predict(std::vector<std::shared_ptr<DNNTensor>> &inputs,
              std::shared_ptr<DnnNodeOutput> dnn_output);

  std::string model_file_name_ = "config/superpoint_640x480.bin";
  std::string model_name_ = "superpoint_640x480";
  ModelTaskType model_task_type_ = ModelTaskType::ModelInferType;
  hbDNNTensorProperties input_tensor_properties_{};
  std::vector<hbDNNTensorProperties> output_tensor_properties_;
  int model_input_width_;
  int model_input_height_;
  int output_index_ = 0;
  bool is_sync_mode_ = true;

  friend class SuperPointextractor;
};

struct SuperPointAssistParser : public SingleBranchOutputParser {};

struct SuperPointOutputParser : public SingleBranchOutputParser {
public:
  SuperPointOutputParser(const std::string &config_file) {
    //  yaml_file_ = config_file + "/centernet.yaml";
  }
//  int32_t Parse(
//          std::shared_ptr<DNNResult> &output,
//          std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
//          std::shared_ptr<OutputDescription> &output_descriptions,
//          std::shared_ptr<DNNTensor> &output_tensor,
//          std::vector<std::shared_ptr<OutputDescription>> &depend_output_descs,
//          std::vector<std::shared_ptr<DNNTensor>> &depend_output_tensors,
//          std::vector<std::shared_ptr<DNNResult>> &depend_outputs) override;
  int32_t Parse(
          std::shared_ptr<DNNResult>& output,
          std::vector<std::shared_ptr<InputDescription>>& input_descriptions,
          std::shared_ptr<OutputDescription>& output_description,
          std::shared_ptr<DNNTensor>& output_tensor);

private:
  bool is_parameter_init_ = false;
  int model_output_height_, model_output_width_;
  int model_input_height_, model_input_width_;
  std::vector<std::vector<uint8_t>> output_shifts_;
  std::vector<int> channel_size_, aligned_channel_size_;
  std::vector<int> width_size_, aligned_width_size_;
  std::vector<int> height_size_, aligned_height_size_;

private:
  void ParameterInit(std::vector<std::shared_ptr<DNNTensor>> &tensors);
  int PostProcess(std::vector<std::shared_ptr<DNNTensor>> &tensors,
          std::vector<cv::KeyPoint> &keypoints,
          cv::Mat &descriptors);

private:
  int border_remove_ = 4;
  int nms_dist_ = border_remove_;
  int cell_size_ = 8;
  float conf_thresh_ = 0.015;
  float nn_thresh_ = 0.7;

  void GetHeatMap(float *data, float *out);
};

struct SuperPointResult : public DNNResult {
public:
  void Reset() override {
    keypoints_.clear();
  }

  cv::Mat descriptors_;
  std::vector<cv::KeyPoint> keypoints_;
};

class SuperPointextractor : public ORBextractor {
public:
  SuperPointextractor(int nfeatures, float scaleFactor, int nlevels,
               int iniThFAST, int minThFAST);

  ~SuperPointextractor(){}

  // Compute the ORB features and descriptors on an image.
  // ORB are dispersed on the image using an octree.
  // Mask is ignored in the current implementation.
  int operator()( cv::InputArray image, cv::InputArray mask,
                  std::vector<cv::KeyPoint>& keypoints,
                  cv::OutputArray descriptors,
                  std::vector<int> &vLappingArea) override;

  int PredictByImage(const cv::Mat &image,
          std::shared_ptr<SuperPointOutput> dnn_output);

private:
  std::shared_ptr<SuperPointNode> node_ = nullptr;
  int superpoint_layer_ = 0;
  std::shared_ptr<hobot::CThreadPool> threadpool_ = nullptr;
  std::mutex allKeypoints_mtx_;
};

}

#endif  //  SUPERPOINTEXTRACTOR_H
