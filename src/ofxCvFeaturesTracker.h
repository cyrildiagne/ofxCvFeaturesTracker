//
//  ofxCvFeaturesTracker.h
//  markerless_AR
//
//  Created by kikko on 07/11/13.
//
//

#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "PatternTracker.h"

class ofxCvFeaturesTracker {
    
public:
    void setup(ofxCv::Calibration calibration);
    void add(ofBaseHasPixels & img);
    void update(ofBaseHasPixels & frame);
    void draw();
    
    virtual bool isFound() { return found; }
    
    virtual int getUpdateTime() { return updateTime; }
    virtual int getNumFeatures() { return tracker.getQueryKeyPoints().size(); }
    virtual int getNumMatches() { return tracker.getMatches().size(); }
    
    virtual std::vector<cv::KeyPoint> getPatternKeyPoints() { return tracker.getPatternKeyPoints(); }
    virtual std::vector<cv::KeyPoint> getQueryKeyPoints() { return tracker.getQueryKeyPoints(); }
    virtual std::vector<cv::DMatch> getMatches() { return tracker.getMatches(); }
    virtual std::vector<cv::Point2f> getQuad() { return tracker.getQuad(); }
    
    virtual ofMatrix4x4 & getModelMatrix() { return modelMatrix; }
    
protected:
	
    void drawImgKeyPoints();
    void drawQueryPoints();
    void drawMatches();
    void drawQuad();
    
    bool found;
    int updateTime;
    cv::PatternTracker tracker;
    ofxCv::Calibration calibration;
    ofMatrix4x4 modelMatrix;
};
