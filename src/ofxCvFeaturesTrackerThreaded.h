//
//  ofxCvFeaturesTrackerThreaded.h
//  markerless_AR
//
//  Created by kikko_fr on 07/11/13.
//
//

#pragma once

#include "ofMain.h"
#include "ofxCvFeaturesTracker.h"

namespace ofxCv {

    class FeaturesTrackerThreaded : public ofThread {
        
    public:
        
        FeaturesTrackerThreaded()
        :needUpdateFront(false)
        ,needUpdateBack(false)
        ,bFound(false)
        {}
        
        ~FeaturesTrackerThreaded() {
            ofLog() << "destroying threaded tracker";
            stopThread();
        }
        
        void setup(ofxCv::Calibration calib){
            calibration = calib;
            frameFront.setUseTexture(false);
            frameBack = new ofImage();
            frameBack->setUseTexture(false);
        }
        
        void add(ofBaseHasPixels & img){
            bool bRestart = isThreadRunning();
            if(bRestart){
                stopThread();
                ofSleepMillis(100);
            }
            imgs.push_back(&img);
            if(bRestart) startThread(true, false);
        }
        
        void update(ofBaseHasPixels & _frame){
            lock();
            frameFront.setFromPixels( _frame.getPixelsRef() );
            needUpdateFront = true;
            unlock();
        }
        
        bool isFound(){
            return bFound;
        }
        
        int getNumFeatures() { return numFeatures; }
        int getNumMatches() { return numMatches; }
        int getUpdateTime() { return updateTime; }
        
        std::vector<cv::KeyPoint> getPatternKeyPoints() { return patternKeyPoints; }
        std::vector<cv::KeyPoint> getQueryKeyPoints() { return queryKeyPoints; }
        std::vector<cv::DMatch>   getMatches() { return matches; }
        std::vector<cv::Point2f>  getQuad() { return quad; }
        
        ofMatrix4x4 & getModelMatrix() { return modelMatrix; }
        
        bool getRT(cv::Mat & rvec_out, cv::Mat & tvec_out) {
            rvec.copyTo(rvec_out);
            tvec.copyTo(tvec_out);
        }
        
    protected:
        
        void threadedFunction() {
            threadedTracker = new FeaturesTracker();
            threadedTracker->setup(calibration);
            for (auto img : imgs) {
                threadedTracker->add(*img);
            }
            while (isThreadRunning()) {
                
                lock();
                
                if (needUpdateFront) {
                    needUpdateBack = needUpdateFront;
                    frameBack->setFromPixels( frameFront.getPixelsRef() );
                    needUpdateFront = false;
                }
                
                unlock();
                
                if(needUpdateBack) {
                    // todo : this systematically crash on exit.. need to figure out why :(
                    threadedTracker->update(*frameBack);
                    needUpdateBack = false;
                } else {
                    ofSleepMillis(10);
                }
                
                
                lock();
                
                const auto t = threadedTracker;
                bFound           = t->isFound();
                numFeatures      = t->getNumFeatures();
                numMatches       = t->getNumMatches();
                updateTime       = t->getUpdateTime();
                patternKeyPoints = t->getPatternKeyPoints();
                queryKeyPoints   = t->getQueryKeyPoints();
                matches          = t->getMatches();
                quad             = t->getQuad();
                
                modelMatrix.set( threadedTracker->getModelMatrix().getPtr() );
                threadedTracker->getRT(calibration.getDistortedIntrinsics().getCameraMatrix(), calibration.getDistCoeffs(), rvec, tvec);
                
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
        
        FeaturesTracker * threadedTracker;
        vector<ofBaseHasPixels*> imgs;
        bool needUpdateFront, needUpdateBack;
        ofImage frameFront/*, frameBack */;
        ofImage * frameBack;
        Calibration calibration;
        cv::Mat rvec, tvec;
    };

}