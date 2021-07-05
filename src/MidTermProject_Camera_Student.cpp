/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[]) {
  /* INIT VARIABLES AND DATA STRUCTURES */

  // data location
  string dataPath = "../";

  // camera
  string imgBasePath = dataPath + "images/";
  string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
  string imgFileType = ".png";
  int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
  int imgEndIndex = 9;   // last file index to load
  int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

  // misc
  int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
  vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
  bool bVis = false;            // visualize results
  bool bFocusOnVehicle = true;
  bool bLimitKpts = false;

  // Detector Parameters
  vector<cv::KeyPoint> keypoints; // create empty feature list for current image
  string detectorName = "ORB"; // SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
  // std::cout << "(SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT)"<< '\n' << "Enter detector of choice: " << '\n';
  // std::cin >> detectorName;
  if (argc > 1) {
    detectorName = argv[1];
    // cout << "Setting the keypoint detector type based on the command line: " << detectorName << "\n";
  }
  // Descriptor Parameters
  cv::Mat descriptors;
  string descriptorName = "ORB"; // BRIEF, ORB, FREAK, AKAZE, SIFT
  // std::cout << "(BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT)"<< '\n' << "Enter descriptor of choice: " << '\n';
  // std::cin >> descriptorName;
  if (argc > 2) {
      descriptorName = argv[2];
      // cout << "Setting the descriptor type based on the command line: " << descriptorName << "\n";
  }
  // check for AKAZE detector - ensure AKAZE keypoints
  if (descriptorName.compare("AKAZE") == 0 && detectorName.compare("AKAZE") != 0) {
    std::cerr << "Warning: Need AKAZE keypoints for AKAZA detector" << '\n';
    return 0;
  }

  //Matching Parameters
  vector<cv::DMatch> matches;
  string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
  string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
  string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

  //performance evaluators
  double detectorTime = 0, descriptorTime = 0;
  vector<int> noKeypoints;
  vector<int> noMatches;
  vector<float> listDetectorTime;
  vector<float> listDescriptorTime;

  /* MAIN LOOP OVER ALL IMAGES */

  for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++) {
    /* LOAD IMAGE INTO BUFFER */

    // assemble filenames for current index
    ostringstream imgNumber;
    imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
    string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

    // load image from file and convert to grayscale
    cv::Mat img, imgGray;
    img = cv::imread(imgFullFilename);
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    //// STUDENT ASSIGNMENT
    //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize
    DataFrame frame;
    frame.cameraImg = imgGray;
    dataBuffer.push_back(frame);
    if(dataBuffer.size()>dataBufferSize){
        dataBuffer.erase(dataBuffer.begin());
    }
    // cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;
    /* DETECT IMAGE KEYPOINTS */

    //// STUDENT ASSIGNMENT
    //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorName
    //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

    if (detectorName.compare("SHITOMASI") == 0) {
      detKeypointsShiTomasi(keypoints, imgGray, detectorTime, bVis);
    }
    else if (detectorName.compare("HARRIS") == 0) {
      detKeypointsHarris(keypoints, imgGray, detectorTime, bVis);
    }
    else if (detectorName.compare("ORB") == 0) {
      detKeypointsOrb(keypoints, imgGray, detectorTime, bVis);
    }
    else if (detectorName.compare("FAST") == 0) {
      detKeypointsFast(keypoints, imgGray, detectorTime, bVis);
    }
    else if (detectorName.compare("BRISK") == 0) {
      detKeypointsBrisk(keypoints, imgGray, detectorTime, bVis);
    }
    else if (detectorName.compare("AKAZE") == 0) {
      detKeypointsAkaze(keypoints, imgGray, detectorTime, bVis);
    }
    else if (detectorName.compare("SIFT") == 0) {
      detKeypointsSift(keypoints, imgGray, detectorTime, bVis);
    }
    else {
      std::cerr << "Warning: Unkown keypoint detector provided" << '\n';
      return 0;
    }
    //// EOF STUDENT ASSIGNMENT

    //// STUDENT ASSIGNMENT
    //// TASK MP.3 -> only keep keypoints on the preceding vehicle
    // only keep keypoints on the preceding vehicle
    // std::cout << "number of keypoints before rect:" << keypoints.size() << '\n';
    cv::Rect vehicleRect(535, 180, 180, 150);
    if (bFocusOnVehicle) {
      for (auto it = keypoints.begin(); it != keypoints.end(); it++)
        if( !vehicleRect.contains(it->pt)) {
          keypoints.erase(it);
          --it;
        }
    }
    // std::cout << "number of keypoints after rect:" << keypoints.size() << '\n';
    //// EOF STUDENT ASSIGNMENT

    // optional : limit number of keypoints (helpful for debugging and learning)
    if (bLimitKpts) {
      int maxKeypoints = 50;

      if (detectorName.compare("SHITOMASI") == 0) {
        // there is no response info, so keep the first 50 as they are sorted in descending quality order
        keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
      }
      cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
      cout << " NOTE: Keypoints have been limited!" << endl;
    }

    // push keypoints and descriptor for current frame to end of data buffer
    (dataBuffer.end() - 1)->keypoints = keypoints;
    // cout << "#2 : DETECT KEYPOINTS done" << endl;

    /* EXTRACT KEYPOINT DESCRIPTORS */
    //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorName
    descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorName, descriptorTime);
    //// EOF STUDENT ASSIGNMENT

    // push descriptors for current frame to end of data buffer
    (dataBuffer.end() - 1)->descriptors = descriptors;

    // cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

    noKeypoints.push_back(keypoints.size());
    listDetectorTime.push_back(detectorTime);
    listDescriptorTime.push_back(descriptorTime);

    // wait until at least two images have been processed
    if (dataBuffer.size() > 1) {
      /* MATCH KEYPOINT DESCRIPTORS */
      //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
      //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

      matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                       (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                       matches, descriptorName, descriptorType, matcherType, selectorType);

      noMatches.push_back(matches.size());

      //// EOF STUDENT ASSIGNMENT

      // store matches in current data frame
      (dataBuffer.end() - 1)->kptMatches = matches;

      // cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

      // visualize matches between current and previous image
      bVis = true;
      if (bVis)
      {
          cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
          cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                          (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                          matches, matchImg,
                          cv::Scalar::all(-1), cv::Scalar::all(-1),
                          vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

          string windowName = "Matching keypoints between two camera images";
          cv::namedWindow(windowName, 7);
          cv::imshow(windowName, matchImg);
          // cout << "Press key to continue to next image" << endl;
          // cv::waitKey(0); // wait for key to be pressed
      }
      bVis = false;
    }
    keypoints.clear();
    matches.clear();
  } // eof loop over all images

  //evaluation
  std::cout << '\n' << '\n' << "###### EVALUATION OF TRACKING SYSTEM " << "Detector, Descriptor: " << detectorName << ", " << descriptorName << " ######" << '\n';

  // Detected keypoints
  // std::cout << "Keypoints per image" << '\n';
  // for (std::vector<int>::const_iterator i = noKeypoints.begin(); i != noKeypoints.end(); ++i)
  //   std::cout << *i << ' ';
  int sum_of_noKeypoints;
  for (auto& n : noKeypoints)
    sum_of_noKeypoints += n;
  std::cout << '\n' << "Sum keypoints: " << sum_of_noKeypoints << " | ";

  // Keypoint matches
  // std::cout << "Matches per image: " << '\n';
  // for (std::vector<int>::const_iterator i = noMatches.begin(); i != noMatches.end(); ++i)
  //   std::cout << *i << ' ';
  int sum_of_noMatches;
  for (auto& n : noMatches)
    sum_of_noMatches += n;
  std::cout << "Sum of keypoints matches: " << sum_of_noMatches << " | ";

  // Time taken for detector
  // std::cout << "Time taken by detector: " << '\n';
  // for (std::vector<float>::const_iterator i = listDetectorTime.begin(); i != listDetectorTime.end(); ++i)
      // std::cout << 1000 * *i / 1.0 << "ms" << ' ';
  float sum_of_detectiontime;
  for (auto& n : listDetectorTime)
    sum_of_detectiontime += n;
  // std::cout << '\n' << "Total time taken by detection system: " << 1000 * sum_of_detectiontime / 1.0 << "ms" << '\n';

  // Time taken for descriptor
  // std::cout << "Time taken by descriptor: " << '\n';
  // for (std::vector<float>::const_iterator i = listDescriptorTime.begin(); i != listDescriptorTime.end(); ++i)
      // std::cout << 1000 * *i / 1.0 << "ms" << ' ';
  float sum_of_descriptortime;
  for (auto& n : listDescriptorTime)
    sum_of_descriptortime += n;
  // std::cout << '\n' << "Total time taken by descriptor system: " << 1000 * sum_of_descriptortime / 1.0 << "ms" << '\n';

  std::cout << "Total time: " << 1000 * (sum_of_detectiontime + sum_of_descriptortime) / 1.0 << "ms" << '\n';
  return 0;
}
