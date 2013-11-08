//
//  ofxCvFeaturesTrackerThreaded.h
//  markerless_AR
//
//  Created by kikko on 07/11/13.
//
//

#pragma once

#include "ofMain.h"
#include "ofxCvFeaturesTracker.h"

class ofxCvFeaturesTrackerThreaded : public ofxCvFeaturesTracker, public ofThread {
    
public:
    
    ofxCvFeaturesTrackerThreaded()
    :needUpdateFront(false)
    ,needUpdateBack(false)
    {}
    
    ~ofxCvFeaturesTrackerThreaded() {
		stopThread();
		ofSleepMillis(500);
	}
    
    void setup(ofxCv::Calibration calib){
        calibration = calib;
        frameFront.setUseTexture(false);
        frameBack.setUseTexture(false);
		startThread(true, false);
    }
    
    void add(ofBaseHasPixels & img){
        stopThread();
		ofSleepMillis(500);
        imgs.push_back(&img);
        startThread(true, false);
    }
    
    void update(ofBaseHasPixels & _frame){
        lock();
        frameFront.setFromPixels( _frame.getPixelsRef() );
        needUpdateFront = true;
        unlock();
    }
    
    bool isFound(){ return bFound; }
    
    int getNumFeatures() { return numFeatures; }
    int getNumMatches() { return numMatches; }
    int getUpdateTime() { return updateTime; }
    
    std::vector<cv::KeyPoint> getPatternKeyPoints() { return patternKeyPoints; }
    std::vector<cv::KeyPoint> getQueryKeyPoints() { return queryKeyPoints; }
    std::vector<cv::DMatch>   getMatches() { return matches; }
    std::vector<cv::Point2f>  getQuad() { return quad; }
    
    ofMatrix4x4 & getModelMatrix() { return modelMatrix; }
    
protected:
    
    void threadedFunction() {
        threadedTracker = new ofxCvFeaturesTracker();
        threadedTracker->setup(calibration);
        for (auto img : imgs) {
            threadedTracker->add(*img);
        }
        while (isThreadRunning()) {
            
            lock();
            
            if (needUpdateFront) {
                needUpdateBack = needUpdateFront;
                frameBack = frameFront;
                needUpdateFront = false;
            }
            
            unlock();
            
            
            if(needUpdateBack) {
				threadedTracker->update(frameBack);
                needUpdateBack = false;
			} else {
				ofSleepMillis(4);
			}
            
            
            lock();
            
            bFound              = threadedTracker->isFound();
            numFeatures         = threadedTracker->getNumFeatures();
            numMatches          = threadedTracker->getNumMatches();
            updateTime          = threadedTracker->getUpdateTime();
            patternKeyPoints    = threadedTracker->getPatternKeyPoints();
            queryKeyPoints      = threadedTracker->getQueryKeyPoints();
            matches             = threadedTracker->getMatches();
            quad                = threadedTracker->getQuad();
            modelMatrix.set( threadedTracker->getModelMatrix().getPtr() );
            
            unlock();
        }
        delete threadedTracker;
    }
    
    int numFeatures;
    int numMatches;
    ofMatrix4x4 modelMatrix;
    std::vector<cv::KeyPoint> patternKeyPoints;
    std::vector<cv::KeyPoint> queryKeyPoints;
    std::vector<cv::DMatch> matches;
    std::vector<cv::Point2f> quad;
    int updateTime;
    bool bFound;
    
private:
    
    ofxCvFeaturesTracker * threadedTracker;
    vector<ofBaseHasPixels*> imgs;
    bool needUpdateFront, needUpdateBack;
    ofImage frameFront, frameBack;
    ofxCv::Calibration calibration;
};