/*
 * DetectRegion.cpp
 *
 *  Created on: May 1, 2014
 *    Author: chd
 */

#include "tb_waiter/detect_region.h"

DetectRegion::DetectRegion()
{
  //constructor
  int blankWidth  = 640; //
  int blankHeight = 480; //

  blankImage = Mat::ones(blankHeight, blankWidth, CV_8UC3);
  blankImage.setTo(Scalar(255,255,255));

  showBasicImages = true;
  showAllImages = false;
  MaxNumRegions = 2;
  RegionCounter = 0;
}

DetectRegion::~DetectRegion()
{
  // Auto-generated destructor stub
}

void DetectRegion::binariza(const Mat &InputImage, Mat &binImage)
{
  Mat midImage;
  Mat Morph = getStructuringElement(MORPH_CROSS,Size( 5, 5 ) );
  cvtColor(InputImage, midImage, COLOR_RGB2GRAY);
  threshold(midImage, binImage ,80, 255, CV_THRESH_BINARY);
  if (showAllImages) {
    namedWindow("binImage",WINDOW_NORMAL);
    imshow("binImage",binImage);
  }
}

void DetectRegion::findRect(const Mat& inputImage, vector<vector<Point> > &mark)
{
  Mat edgesImage, binImage;
  vector<vector<Point> > contours;
  vector<Point> obj;
  double maxCosine = 0;

  Canny(inputImage, edgesImage, 10, 20, 3);
  dilate(edgesImage, edgesImage, Mat(), Point(-1,-1));

  findContours(edgesImage, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  for( size_t i = 0; i < contours.size(); i++ )
  {
    double objLen = arcLength(Mat(contours[i]), true);
    approxPolyDP(Mat(contours[i]), obj, objLen*0.02, true);
    if( obj.size() == 4 && fabs(contourArea(Mat(obj))) > 1000 && isContourConvex(Mat(obj)) )
      mark.push_back(obj);
  }
}

Point DetectRegion::getCenter( vector<Point> points )
{
  Point center = Point( 0.0, 0.0 );

  for( size_t i = 0; i < points.size(); i++ )
  {
    center.x += points[ i ].x;
    center.y += points[ i ].y;
  }
  center.x = center.x / points.size();
  center.y = center.y / points.size();
  return (center);
}

float DetectRegion::distanceBetweenPoints( Point p1, Point p2 )
{
  if( p1.x == p2.x ) {
    return ( abs( p2.y - p1.y ) );
  }
  else if( p1.y == p2.y ) {
    return ( abs( p2.x - p1.x ) );
  }
  else {
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    return ( sqrt( (dx*dx)+(dy*dy) ) );
  }
}

vector<Point> DetectRegion::sortCorners( vector<Point> square )
{
  // 0----1
  // |  |
  // |  |
  // 3----2
  Point center = getCenter( square );

  vector<Point> sorted_square;
  for( size_t i = 0; i < square.size(); i++ )
  {
    if ( (square[i].x - center.x) < 0 && (square[i].y - center.y) < 0 )
    {
      switch( i ) {
      case 0:
        sorted_square = square;
        break;
      case 1:
        sorted_square.push_back( square[1] );
        sorted_square.push_back( square[2] );
        sorted_square.push_back( square[3] );
        sorted_square.push_back( square[0] );
        break;
      case 2:
        sorted_square.push_back( square[2] );
        sorted_square.push_back( square[3] );
        sorted_square.push_back( square[0] );
        sorted_square.push_back( square[1] );
        break;
      case 3:
        sorted_square.push_back( square[3] );
        sorted_square.push_back( square[0] );
        sorted_square.push_back( square[1] );
        sorted_square.push_back( square[2] );
        break;
      }
      break;
    }
  }
  return (sorted_square);
}

void DetectRegion::cropImageWithMask(const Mat &img_orig, const Mat &mask, Mat &crop)
{
  Mat Image, maskModImage, cropImage, img_bw;
  Mat element = getStructuringElement(MORPH_CROSS,Size( 3, 3 ) );

  cvtColor(img_orig, Image, CV_BGR2GRAY);
  equalizeHist(Image, Image);
  cvtColor(mask, maskModImage, CV_BGR2GRAY);
  erode(maskModImage, maskModImage, element);

  bitwise_not(Image, cropImage);
  bitwise_and(maskModImage,cropImage,cropImage);
  GaussianBlur(cropImage, img_bw, Size(5,5),0);
  threshold(img_bw, crop ,150, 255, CV_THRESH_BINARY);
}

void DetectRegion::cropImageColor(const Mat &img, const Mat &cropImage, Mat & color_crop)
{
  Mat ImgCropRGB;
  cvtColor(cropImage, ImgCropRGB, CV_GRAY2RGB);
  bitwise_and(img, ImgCropRGB, color_crop);
}

Scalar DetectRegion::regionAvgColor(const Mat &img, const Mat &mask)
{
  Scalar avg_color;
  avg_color = mean(img, mask);
  return (avg_color);
}

void DetectRegion::createRegionMat(const Mat &normalImage, vector<Point> &contour, Mat &RegionImage)
{
  RegionImage = Mat::zeros(200, 400, CV_8UC3);
  Point2f RegionPoints[4], pPerspOrig[4], contourPersp[4];
  float distanceP0P1, distanceP0P2, distanceP0P3;

  //contour = sortCorners(contour);
  contourPersp[0] = contour[0];
  contourPersp[1] = contour[1];
  contourPersp[2] = contour[2];
  contourPersp[3] = contour[3];

  distanceP0P1 = distanceBetweenPoints( contourPersp[0], contourPersp[1] );
  distanceP0P2 = distanceBetweenPoints( contourPersp[0], contourPersp[2] );
  distanceP0P3 = distanceBetweenPoints( contourPersp[0], contourPersp[3] );

  if ( distanceP0P1 < distanceP0P3 )
  {
    RegionPoints[0]=(Point2f(0, 0));
    RegionPoints[1]=(Point2f(0, RegionImage.rows));
    RegionPoints[2]=(Point2f(RegionImage.cols, RegionImage.rows));
    RegionPoints[3]=(Point2f(RegionImage.cols, 0));
  }
  else
  {
    RegionPoints[1]=(Point2f(0, 0));
    RegionPoints[2]=(Point2f(0, RegionImage.rows));
    RegionPoints[3]=(Point2f(RegionImage.cols, RegionImage.rows));
    RegionPoints[0]=(Point2f(RegionImage.cols, 0));
  }

  Mat transmtx = getPerspectiveTransform(contourPersp, RegionPoints);
  warpPerspective(normalImage, RegionImage, transmtx, RegionImage.size());
}

void DetectRegion::cropRegionImage(const Mat &normalImage, vector<Point> &contour, Mat &cropImage)
{
  // declare used vars
  Point2f contourPersp[4], pPerspOrig[4];
  Mat arMask;

  contour = sortCorners(contour);
  //
  contourPersp[0] = contour[0];
  contourPersp[1] = contour[1];
  contourPersp[2] = contour[2];
  contourPersp[3] = contour[3];

  pPerspOrig[0] = Point( 0, 0 );
  pPerspOrig[1] = Point( blankImage.cols, 0 );
  pPerspOrig[2] = Point( blankImage.cols, blankImage.rows );
  pPerspOrig[3] = Point( 0, blankImage.rows);

  Mat warpMatrix = getPerspectiveTransform(pPerspOrig, contourPersp);
  warpPerspective(blankImage, arMask, warpMatrix, blankImage.size());

  cropImageWithMask(normalImage, arMask, cropImage);
  if (showAllImages) {
    namedWindow("arMask",WINDOW_NORMAL);
    imshow("arMask",arMask);
    namedWindow("cropImage",WINDOW_NORMAL);
    imshow("cropImage",cropImage);
  }
}

bool DetectRegion::verifySize(vector<Point> &contour)
{
  float MaxPerimeter = 1450;
  float MinPerimeter = 530;
  float MaxArea = 165000;
  float MinArea = 12000;

  float perimeter = arcLength(contour,true);
  float area = contourArea(contour);

  return  (area > MinArea && area < MaxArea);
}

string DetectRegion::detectShape(const Mat &img, const Mat &imgCrop)
{
  Mat img_shapes, img_crop;
  vector<vector<Point> > objects;
  string figure = "other";
  double circular;

  img_shapes = img.clone();
  img_crop = imgCrop.clone();

  findContours(img_crop, objects, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
  drawContours(img_shapes, objects, -1, Scalar(0, 255, 0), 3 );

  for( size_t i = 0; i < objects.size(); i++ )
  {
    double objArea = contourArea(Mat(objects[i]));
    double objLen = arcLength(Mat(objects[i]), true);
    if( objArea > 400 and objLen > 100 )
    {
      circular = 4*PI*objArea/(objLen*objLen);
      if (circular >= 0.75) {
        figure = "circle";
        break;
      }
      if (circular > 0.45 and circular < 0.75) {
        figure = "triangle";
        break;
      }
    }
  }
  if (showBasicImages || showAllImages) {
    imshow("normal",img_shapes);
  }
  return (figure);
}

vector<string> DetectRegion::runFigureDetection(const Mat &InputImage)
{
  Mat binImage, cropImage, colorCrop;
  vector<Point> contour;
  vector<string> figureAndColor;
  string figure = "" , color = "black";
  vector<vector<Point> > auxSegments; // segments
  int MaxIter;

  segments.clear();

  Mat modImage = InputImage.clone();
  binariza(InputImage, binImage);
  findRect(binImage, segments);

  if (showBasicImages || showAllImages)
  {
    namedWindow("normal",WINDOW_NORMAL);
    imshow("normal",InputImage);
  }

  if ( segments.size() > 0)
  {
    contour = segments[0];
    if ( verifySize(contour) )
    {
      cropRegionImage(InputImage, contour, cropImage);
      cropImageColor(InputImage,cropImage, colorCrop);
      figure = detectShape(InputImage,cropImage);
    }
  }

  figureAndColor.push_back(figure);
  figureAndColor.push_back(color);

  return (figureAndColor);
}
