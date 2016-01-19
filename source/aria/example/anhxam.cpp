#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
 
using namespace cv;
using namespace std;
 
int main( )
{
 Mat src1;
 src1 = imread("img.jpg", CV_LOAD_IMAGE_COLOR);
  
 Mat gray, dst;
 
 // convert to gray
 cvtColor(src1, gray, CV_BGR2GRAY);
 namedWindow( "Original image", CV_WINDOW_AUTOSIZE );
 imshow( "Original image", gray );
 
 // hisogram equalization
 equalizeHist( gray, dst );
 
 namedWindow("image", CV_WINDOW_AUTOSIZE);
 imshow("image", dst);
 
 waitKey(0);                                       
 return 0;
}
