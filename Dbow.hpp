//
//  Dbow.hpp
//  ORB-DBoW3
//
//  Created by Xiangyu Liu on 3/9/18.
//  Copyright © 2018 Xiangyu Liu. All rights reserved.
//

#ifndef Dbow_hpp
#define Dbow_hpp

#include "CommonInclude.hpp"

#include "DBoW3.h"
#include "timers.h"

class Dbow{
public:
    typedef std::shared_ptr<Dbow> Ptr;
    
    DBoW3::Vocabulary vocab_;
    
    Dbow();
};
#endif /* Dbow_hpp */
