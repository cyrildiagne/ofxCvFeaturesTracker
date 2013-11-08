//
//  PatternTracker.cpp
//  markerless_AR
//
//  Created by kikko on 07/11/13.
//
//


// most of the code here comes from :
// Ch3 of the book "Mastering OpenCV with Practical Computer Vision Projects"
// http://www.packtpub.com/cool-projects-with-opencv/book

#include "PatternTracker.h"

namespace cv {

    PatternTracker::PatternTracker()
    : enableRatioTest(true)
    , enableHomographyRefinement(true)
    , homographyReprojectionThreshold(3)
    , minNumberMatchesAllowed(6)
    , rescale(1)
    {
    }

    void PatternTracker::setup(){
        m_detector = new cv::OrbFeatureDetector(400); // cv::ORB(1000);
        m_extractor = new cv::OrbDescriptorExtractor(); // cv::FREAK(false, false)
        m_matcher = new cv::BruteForceMatcher< cv::HammingLUT >(); // cv::BFMatcher(cv::NORM_HAMMING, true)
    }
    
    void PatternTracker::add(const cv::Mat& image)
    {
        // Store the pattern object
        m_pattern = buildPatternFromImage(image);
        
        // API of cv::DescriptorMatcher is somewhat tricky
        // First we clear old train data:
        m_matcher->clear();
        
        // Then we add vector of descriptors (each descriptors matrix describe one image).
        // This allows us to perform search across multiple images:
        std::vector<cv::Mat> descriptors(1);
        descriptors[0] = m_pattern.descriptors.clone();
        m_matcher->add(descriptors);
        
        // After adding train data perform actual train:
        m_matcher->train();
    }

    bool PatternTracker::find(const cv::Mat& image)
    {
        if(rescale == 1) {
            m_img = image;
        } else {
            resize(image, m_img, cv::Size(rescale * image.cols, rescale * image.rows));
        }
        
        // Convert input image to gray
        getGray(m_img, m_grayImg);
        
        // Extract feature points from input gray image
        extractFeatures(m_grayImg, m_queryKeypoints, m_queryDescriptors);
        
        // Rescale points set
        if(rescale != 1){
            for (auto & p : m_queryKeypoints) {
                p.pt.x /= rescale;
                p.pt.y /= rescale;
            }
        }
        
        // Get matches with current pattern
        getMatches(m_queryDescriptors, m_matches);
        
        // Find homography transformation and detect good matches
        bool homographyFound = refineMatchesWithHomography(m_queryKeypoints,
                                                           m_pattern.keypoints,
                                                           homographyReprojectionThreshold,
                                                           m_matches,
                                                           m_roughHomography);
        
        if (homographyFound)
        {
            // If homography refinement enabled improve found transformation
            if (enableHomographyRefinement)
            {
                // Warp image using found homography
                cv::warpPerspective(m_grayImg, m_warpedImg, m_roughHomography, m_pattern.size, cv::WARP_INVERSE_MAP | cv::INTER_CUBIC);
                
                // Get refined matches:
                std::vector<cv::KeyPoint> warpedKeypoints;
                std::vector<cv::DMatch> refinedMatches;
                
                // Detect features on warped image
                extractFeatures(m_warpedImg, warpedKeypoints, m_queryDescriptors);
                
                // Match with pattern
                getMatches(m_queryDescriptors, refinedMatches);
                
                // Estimate new refinement homography
                homographyFound = refineMatchesWithHomography(warpedKeypoints,
                                                              m_pattern.keypoints,
                                                              homographyReprojectionThreshold,
                                                              refinedMatches,
                                                              m_refinedHomography);

                // Get a result homography as result of matrix product of refined and rough homographies:
                m_info.homography = m_roughHomography * m_refinedHomography;
                
                // Transform contour with precise homography
                cv::perspectiveTransform(m_pattern.points2d, m_info.points2d, m_info.homography);
            }
            else
            {
                m_info.homography = m_roughHomography;
                
                // Transform contour with rough homography
                cv::perspectiveTransform(m_pattern.points2d, m_info.points2d, m_roughHomography);
            }
        }
        
        return homographyFound;
    }
    
    void PatternTracker::getPose(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, cv::Mat &rvec, cv::Mat &tvec){
        solvePnP(m_pattern.points3d, m_info.points2d, cameraMatrix, distCoeffs, rvec, tvec);
    }
    
    
#pragma mark - Private
    
    
    Pattern PatternTracker::buildPatternFromImage(const cv::Mat& image) const
    {
        Pattern pattern;
        
        // Store original image in pattern structure
        pattern.size = cv::Size(image.cols, image.rows);
        pattern.frame = image.clone();
        getGray(image, pattern.grayImg);
        
        // Build 2d and 3d contours (3d contour lie in XY plane since it's planar)
        pattern.points2d.resize(4);
        pattern.points3d.resize(4);
        
        // Image dimensions
        const float w = image.cols;
        const float h = image.rows;
        
        // Normalized dimensions:
        const float maxSize = std::max(w,h);
        const float unitW = w / maxSize;
        const float unitH = h / maxSize;
        
        pattern.points2d[0] = cv::Point2f(0,0);
        pattern.points2d[1] = cv::Point2f(w,0);
        pattern.points2d[2] = cv::Point2f(w,h);
        pattern.points2d[3] = cv::Point2f(0,h);
        
        pattern.points3d[0] = cv::Point3f(-unitW, -unitH, 0);
        pattern.points3d[1] = cv::Point3f( unitW, -unitH, 0);
        pattern.points3d[2] = cv::Point3f( unitW,  unitH, 0);
        pattern.points3d[3] = cv::Point3f(-unitW,  unitH, 0);
        
        extractFeatures(pattern.grayImg, pattern.keypoints, pattern.descriptors);
        
        return pattern;
    }

    void PatternTracker::getGray(const cv::Mat& image, cv::Mat& gray)
    {
        if (image.channels()  == 3)
            cv::cvtColor(image, gray, CV_BGR2GRAY);
        else if (image.channels() == 4)
            cv::cvtColor(image, gray, CV_BGRA2GRAY);
        else if (image.channels() == 1)
            gray = image;
    }

    bool PatternTracker::extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const
    {
        assert(!image.empty());
        assert(image.channels() == 1);
        
        m_detector->detect(image, keypoints);
        if (keypoints.empty())
            return false;
        
        m_extractor->compute(image, keypoints, descriptors);
        if (keypoints.empty())
            return false;
        
        return true;
    }
    
    void PatternTracker::getMatches(const cv::Mat& queryDescriptors, std::vector<cv::DMatch>& matches)
    {
        matches.clear();
        
        if (enableRatioTest)
        {
            // To avoid NaN's when best match has zero distance we will use inversed ratio.
            const float minRatio = 1.f / 1.5f;
            
            // KNN match will return 2 nearest matches for each query descriptor
            m_matcher->knnMatch(queryDescriptors, m_knnMatches, 2);
            
            for (size_t i=0; i<m_knnMatches.size(); i++)
            {
                const cv::DMatch& bestMatch   = m_knnMatches[i][0];
                const cv::DMatch& betterMatch = m_knnMatches[i][1];
                
                float distanceRatio = bestMatch.distance / betterMatch.distance;
                
                // Pass only matches where distance ratio between
                // nearest matches is greater than 1.5 (distinct criteria)
                if (distanceRatio < minRatio)
                {
                    matches.push_back(bestMatch);
                }
            }
        }
        else
        {
            // Perform regular match
            m_matcher->match(queryDescriptors, matches);
        }
    }

    bool PatternTracker::refineMatchesWithHomography
    (
     const std::vector<cv::KeyPoint>& queryKeypoints,
     const std::vector<cv::KeyPoint>& trainKeypoints,
     float reprojectionThreshold,
     std::vector<cv::DMatch>& matches,
     cv::Mat& homography
     )
    {
        if (matches.size() < minNumberMatchesAllowed)
            return false;
        
        // Prepare data for cv::findHomography
        std::vector<cv::Point2f> srcPoints(matches.size());
        std::vector<cv::Point2f> dstPoints(matches.size());
        
        for (size_t i = 0; i < matches.size(); i++)
        {
            srcPoints[i] = trainKeypoints[matches[i].trainIdx].pt;
            dstPoints[i] = queryKeypoints[matches[i].queryIdx].pt;
        }
        
        // Find homography matrix and get inliers mask
        std::vector<unsigned char> inliersMask(srcPoints.size());
        homography = cv::findHomography(srcPoints, 
                                        dstPoints, 
                                        CV_FM_RANSAC, 
                                        reprojectionThreshold, 
                                        inliersMask);
        
        std::vector<cv::DMatch> inliers;
        for (size_t i=0; i<inliersMask.size(); i++)
        {
            if (inliersMask[i])
                inliers.push_back(matches[i]);
        }
        
        matches.swap(inliers);
        return matches.size() > minNumberMatchesAllowed;
    }

}