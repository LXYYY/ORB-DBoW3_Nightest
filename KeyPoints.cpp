//
//  KeyPoints.cpp
//  ORB-DBoW3
//
//  Created by Xiangyu Liu on 3/9/18.
//  Copyright © 2018 Xiangyu Liu. All rights reserved.
//

#include "KeyPoints.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <boost/timer.hpp>



KeyPoint::KeyPoint() {
    orb_ = cv::ORB::create();
}

bool KeyPoint::detectAndCompute(cv::Mat image) {
    orb_->detectAndCompute(image, cv::Mat(), keypoints_, descriptors_);
    
    return true;
}

