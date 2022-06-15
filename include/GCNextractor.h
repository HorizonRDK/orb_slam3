/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef GCNEXTRACTOR_H
#define GCNEXTRACTOR_H

#include <torch/script.h> // One-stop header.
#include <torch/torch.h>

#include "ORBextractor.h"

// Compile will fail for opimizier since pytorch defined this
#ifdef EIGEN_MPL2_ONLY
#undef EIGEN_MPL2_ONLY
#endif

#include <vector>
#include <list>
#include <opencv2/core.hpp>

namespace ORB_SLAM3
{

class GCNextractor : public ORBextractor {
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    GCNextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~GCNextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    int operator()( cv::InputArray image, cv::InputArray mask,
                    std::vector<cv::KeyPoint>& keypoints,
                    cv::OutputArray descriptors,
                    std::vector<int> &vLappingArea) override ;

#if defined(TORCH_NEW_API)
    torch::jit::script::Module module;
#else
    std::shared_ptr<torch::jit::script::Module> module;
#endif
};

}  //  namespace ORB_SLAM3

#endif

