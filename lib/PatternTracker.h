//
//  PatternTracker.h
//  markerless_AR
//
//  Created by kikko on 07/11/13.
//
//


// most of the code here comes from :
// Ch3 of the book "Mastering OpenCV with Practical Computer Vision Projects"
// http://www.packtpub.com/cool-projects-with-opencv/book

#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace cv {

    /**
     * Store the image data and computed descriptors of target pattern
     */
    struct Pattern
    {
        cv::Size                  size;
        cv::Mat                   frame;
        cv::Mat                   grayImg;
        
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat                   descriptors;
        
        std::vector<cv::Point2f>  points2d;
        std::vector<cv::Point3f>  points3d;
    };
    
    /**
     * Intermediate pattern tracking info structure
     */
    struct TrackingInfo
    {
        cv::Mat                   homography;
        std::vector<cv::Point2f>  points2d;
    };
    
    /**
     * Train pattern and perform feature extraction & matching on input frames
     */
    class PatternTracker
    {
    public:
        PatternTracker();
        
        void setup();
        void add(const cv::Mat& image);
        bool find(const cv::Mat& image);
        void getPose(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, cv::Mat& rvec, cv::Mat& tvec);
        
        int minNumberMatchesAllowed;
        bool enableRatioTest;
        bool enableHomographyRefinement;
        float homographyReprojectionThreshold;
        float rescale;
        
        const std::vector<cv::KeyPoint>&    getPatternKeyPoints() { return m_pattern.keypoints; }
        const std::vector<cv::KeyPoint>&    getQueryKeyPoints() { return m_queryKeypoints; }
        const std::vector<cv::DMatch>&      getMatches() { return m_matches; }
        const std::vector<cv::Point2f>&     getQuad() { return m_info.points2d; }
        
    protected:
        
        
        /**
         * Initialize Pattern structure from the input image.
         * This function finds the feature points and extract descriptors for them.
         */
        Pattern buildPatternFromImage(const cv::Mat& image) const;
        
        /**
         *
         */
        bool extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const;
        
        /**
         *
         */
        void getMatches(const cv::Mat& queryDescriptors, std::vector<cv::DMatch>& matches);
        
        /**
         * Get the gray image from the input image.
         * Function performs necessary color conversion if necessary
         * Supported input images types - 1 channel (no conversion is done), 3 channels (assuming BGR) and 4 channels (assuming BGRA).
         */
        static void getGray(const cv::Mat& image, cv::Mat& gray);
        
        /**
         *
         */
        bool refineMatchesWithHomography(const std::vector<cv::KeyPoint>& queryKeypoints,
                                        const std::vector<cv::KeyPoint>& trainKeypoints,
                                        float reprojectionThreshold,
                                        std::vector<cv::DMatch>& matches,
                                        cv::Mat& homography);
        
    private:
        std::vector<cv::KeyPoint> m_queryKeypoints;
        cv::Mat                   m_queryDescriptors;
        std::vector<cv::DMatch>   m_matches;
        std::vector< std::vector<cv::DMatch> > m_knnMatches;
        
        cv::Mat                   m_img;
        cv::Mat                   m_grayImg;
        cv::Mat                   m_warpedImg;
        cv::Mat                   m_roughHomography;
        cv::Mat                   m_refinedHomography;
        
        Pattern                   m_pattern;
        TrackingInfo              m_info;
        
        cv::Ptr<cv::FeatureDetector>     m_detector;
        cv::Ptr<cv::DescriptorExtractor> m_extractor;
        cv::Ptr<cv::DescriptorMatcher>   m_matcher;
    };
    
}