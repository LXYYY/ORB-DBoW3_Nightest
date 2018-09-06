//
//  main.cpp
//  ORB-DBoW3
//
//  Created by Xiangyu Liu on 3/9/18.
//  Copyright © 2018 Xiangyu Liu. All rights reserved.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include "CommonInclude.hpp"
#include "Dbow.hpp"
#include "KeyPoints.hpp"
#include <boost/timer.hpp>

#define TRAINING_VOCAB 1
#define TEST
//#define TRAIN
//#define LOOP

using namespace std;

int main(int argc, const char * argv[]) {
    KeyPoint::Ptr keyPoint(new KeyPoint);
    Dbow::Ptr dbow(new Dbow);
    
//    //read images
//    cout<<"reading images"<<endl;
//    vector<cv::Mat> images;
//
//    string data_dir="/Users/xiangyuliu/Library/TUM_dataset/rgbd_dataset_freiburg1_xyz/";
//    ifstream fin ( data_dir+"rgb.txt" );
//    if ( !fin )
//    {
//        cout<<"please generate the rgb file called rgb.txt!"<<endl;
//        return 1;
//    }
//
//    vector<string> rgb_files;
//    vector<double> rgb_times;
//    while ( !fin.eof() )
//    {
//        string rgb_time, rgb_file, depth_time, depth_file;
//        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
//        rgb_times.push_back ( atof ( rgb_time.c_str() ));
//        rgb_files.push_back ( data_dir+rgb_file);
//        if ( fin.good() == false )
//            break;
//    }
//    for( string fileName : rgb_files){
//        images.push_back(cv::imread(fileName));
//    }
//
    
    cout << "reading database." << endl;
    string dir = "/Users/xiangyuliu/Onedrive/OneDrive - Nanyang Technological University/Dissertation-Multi-Robot SLAM/slamBook/ch12/";
//    DBoW3::Vocabulary vocab(dir+"vocabulary.yml.gz");
//    if (vocab.empty()) {
//        cerr << "vocabulary does not exist." << endl;
//        return 1;
//    }
    cout << "reading images." << endl;
    vector<cv::Mat> images;
    for (int i = 0; i < 10; ++i) {
        string path = dir +"data/"+ to_string(i + 1) + ".png";
        images.push_back(cv::imread(path));
    }
    
#ifdef TRAIN
    //    detect ORB features
    cout << "detecting ORB features." << endl;
    cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
    vector<cv::Mat> descriptors;
    for (cv::Mat &image:images) {
        vector<cv::KeyPoint> keypoints;
        cv::Mat descriptor;
        detector->detectAndCompute(image, cv::Mat(), keypoints, descriptor);
        descriptors.push_back(descriptor);
    }
    
    //    create vocabulary
    cout << "creating vocabulary." << endl;
    DBoW3::Vocabulary vocab;
    vocab.create(descriptors);
    cout << "vocabulary info: " << vocab << endl;
    vocab.save("vocabulary.yml.gz");
    cout << "done." << endl;
#endif
    
#ifdef LOOP
    cout << "reading database." << endl;
    DBoW3::Vocabulary vocab(dir+"vocabulary.yml.gz");
    if (vocab.empty()) {
        cerr << "vocabulary does not exist." << endl;
        return 1;
    }
    
    //    detect ORB features
    cout << "detecting ORB features." << endl;
    cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
    vector<cv::Mat> descriptors;
    for (cv::Mat &image:images) {
        vector<cv::KeyPoint> keypoints;
        cv::Mat descriptor;
        detector->detectAndCompute(image, cv::Mat(), keypoints, descriptor);
        descriptors.push_back(descriptor);
    }
    
    cout << "comparing image with images." << endl;
    for (int i = 0; i < images.size(); ++i) {
        DBoW3::BowVector v1;
        vocab.transform(descriptors[i], v1);
        for (int j = i; j < images.size(); ++j) {
            DBoW3::BowVector v2;
            vocab.transform(descriptors[j], v2);
            double score = vocab.score(v1, v2);
            cout << "image " << i << " vs image " << j << " : " << score << endl;
        }
        cout << endl;
    }
    
    //    compare with database
    cout << "comparing images with database." << endl;
    DBoW3::Database db(vocab, false, 0);
    for (const auto &descriptor : descriptors) {
        db.add(descriptor);
    }
    cout << "database info:" << db << endl;
    for (int i = 0; i < descriptors.size(); ++i) {
        DBoW3::QueryResults ret;
        //        max result = 4
        db.query(descriptors[i], ret, 4);
        cout << "searching for image " << i << " returns " << ret << endl << endl;
    }
    cout << "done" << endl;
#endif
    
#ifdef TEST
    vector<cv::Mat> descriptors;
    
    boost::timer timer;
    
    for(int i=0;i<10;i++){
        cv::Mat image=images.at(i);
        keyPoint->detectAndCompute(image);
        descriptors.push_back(keyPoint->descriptors_);
    }
    
    cout<<"ORB costs time: "<<timer.elapsed() <<endl;
    cout<<"descriptors number:"<<descriptors.size()<<endl;
    
#if TRAINING_VOCAB
    cout<<"creating vocabulary"<<endl;
    dbow->vocab_.create(descriptors);
    cout<<"vocabulary info: "<<dbow->vocab_<<endl;
    dbow->vocab_.save("vacabulary.yml.gz");
    cout<<"done"<<endl;
#endif
    
    //compute PR
    cout<<"reading database"<<endl;
    DBoW3::Vocabulary vocab("vacabulary.yml.gz");
    if(vocab.empty()){
        cerr<<"Vocabulary does not exist."<<endl;
        return 1;
    }
    
    //compare images directly
    DBoW3::BowVector v1;
    vocab.transform(descriptors[4], v1);
    //cout<<descriptors[4]<<endl;
    cout<<"v1:"<<endl;
    for(DBoW3::BowVector::iterator iter = v1.begin(); iter != v1.end(); iter++) {
        cout << iter->first << " : " << iter->second << endl;
    }
    
    for(int i=0;i<descriptors.size()-1;i++){
        DBoW3::BowVector v2;
        vocab.transform(descriptors[i], v2);
        double score=vocab.score(v1,v2);
        cout<<"image.1 vs image."<<i<<" : "<<score<<endl;
    }
#endif
    return 0;
}
