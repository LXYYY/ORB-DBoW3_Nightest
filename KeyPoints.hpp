//
//  KeyPoints.hpp
//  ORB-DBoW3
//
//  Created by Xiangyu Liu on 3/9/18.
//  Copyright © 2018 Xiangyu Liu. All rights reserved.
//

#ifndef KeyPoints_hpp
#define KeyPoints_hpp

#include "CommonInclude.hpp"
#include <opencv2/opencv.hpp>

class KeyPoint
{
public:
    typedef std::shared_ptr<KeyPoint> Ptr;
    
    cv::Ptr<cv::Feature2D> orb_;
    cv::Mat descriptors_;
    std::vector<cv::KeyPoint> keypoints_;
    
    KeyPoint();
    
    bool detectAndCompute(cv::Mat image);
};

#endif /* KeyPoints_hpp */
