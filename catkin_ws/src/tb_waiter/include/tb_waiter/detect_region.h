/*
 * DetectRegion.h
 *
 *  Created on: May 1, 2014
 *    Author: chd
 */

#ifndef DETECTREGION_H_
#define DETECTREGION_H_

#include <cv.h>
#include <highgui.h>
#include <cvaux.h>
#include <ml.h>

#include <iostream>
#include <math.h>
#include <string.h>
#include <sstream>

using namespace cv;
using namespace std;

const double PI = 3.14159265359;

struct RectRegion {
  Mat RegionImage;
  Mat cropImage;
  string text;
};

class DetectRegion
{
public:
  DetectRegion();
  virtual ~DetectRegion();

  void binariza(const Mat &InputImage, Mat &binImage);
  void findRect(const Mat &binImage, vector<vector<Point> > &mark);
  void createRegionMat(const Mat &normalImage, vector<Point> &contour, Mat &RegionImage);
  void cropRegionImage(const Mat &normalImage, vector<Point> &contour, Mat &cropImage);
  bool verifySize(vector<Point> &contour);
  string detectShape(const Mat &img, const Mat &img_crop);
  vector<string> runFigureDetection(const Mat &InputImage);

  //
  bool showBasicImages;
  bool showAllImages;

private:
  Point getCenter( vector<Point> points );
  float distanceBetweenPoints( Point p1, Point p2 );
  vector<Point> sortCorners( vector<Point> square );
  void cropImageWithMask(const Mat &img_orig, const Mat &mask, Mat &crop);
  void cropImageColor(const Mat &img, const Mat &cropImage, Mat & color_crop);
  Scalar regionAvgColor(const Mat &img, const Mat &mask);

  //
  Mat blankImage;
  vector<vector<Point> > segments;
  int MaxNumRegions;
  int RegionCounter;
};

#endif /* DETECTRegion_H_ */
