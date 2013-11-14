//
//  ofxCvFeaturesTracker.h
//  markerless_AR
//
//  Created by kikko_fr on 07/11/13.
//
//

#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "PatternTracker.h"

namespace ofxCv {

    class FeaturesTracker {
        
    public:
        
        virtual ~FeaturesTracker(){ ofLog() << "destroying featuresTracker"; }
        
        void setup(ofxCv::Calibration calibration);
        void add(ofBaseHasPixels & img);
        void update(ofBaseHasPixels & frame);
        void draw();
        
        virtual bool isFound() const { return found; }
        
        virtual int getUpdateTime() const { return updateTime; }
        virtual int getNumFeatures() const { return tracker.getQueryKeyPoints().size(); }
        virtual int getNumMatches() const { return tracker.getMatches().size(); }
        
        virtual std::vector<cv::KeyPoint> getPatternKeyPoints() const { return tracker.getPatternKeyPoints(); }
        virtual std::vector<cv::KeyPoint> getQueryKeyPoints() const { return tracker.getQueryKeyPoints(); }
        virtual std::vector<cv::DMatch> getMatches() const { return tracker.getMatches(); }
        virtual std::vector<cv::Point2f> getQuad() const { return tracker.getQuad(); }
        
        virtual ofMatrix4x4 & getModelMatrix() { return modelMatrix; }
        virtual ofMatrix4x4 & getModelMatrix(cv::Mat & cameraMatrix, cv::Mat & distCoefs);
        virtual bool getRT(const cv::Mat & cameraMatrix, const cv::Mat & distCoefs, cv::Mat & rvec, cv::Mat & tvec);
        
    protected:
        
        void drawImgKeyPoints();
        void drawQueryPoints();
        void drawMatches();
        void drawQuad();
        
        bool found;
        int updateTime;
        cv::PatternTracker tracker;
        Calibration calibration;
        ofMatrix4x4 modelMatrix;
    };

}