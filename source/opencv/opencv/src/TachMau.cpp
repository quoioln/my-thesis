#include <stdio.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;
int main()
{
	Mat imageSrc = imread("carStdio.png", CV_LOAD_IMAGE_COLOR);
	vector<Mat> imageRGB;
	// Khởi tạo các biến lưu trữ 3 bênh màu
	Mat imageRed, imageGreen, imageBlue;

	int width = 250, height = 250;
	int sizeHistogram = 255;
	float range[] = { 0, 255 };
	const float* histogramRange = { range };

	// Hàm này có tác dụng tách imageSrc thành 3 kênh màu.
	split(imageSrc, imageRGB);

	// Tính toán cho từng kênh màu và vẽ biểu đồ Histogram
	calcHist(&imageRGB[0], 1, 0, Mat(), imageRed, 1, &sizeHistogram, &histogramRange, true, false);
	calcHist(&imageRGB[1], 1, 0, Mat(), imageGreen, 1, &sizeHistogram, &histogramRange, true, false);
	calcHist(&imageRGB[2], 1, 0, Mat(), imageBlue, 1, &sizeHistogram, &histogramRange, true, false);

	int bin = cvRound((double)width / sizeHistogram);

	Mat dispRed(width, height, CV_8UC3, Scalar(255, 255, 255));
	Mat dispGreen = dispRed.clone();
	Mat dispBlue = dispRed.clone();

	normalize(imageBlue, imageBlue, 0, dispBlue.rows, NORM_MINMAX, -1, Mat());
	normalize(imageGreen, imageGreen, 0, dispGreen.rows, NORM_MINMAX, -1, Mat());
	normalize(imageRed, imageRed, 0, dispRed.rows, NORM_MINMAX, -1, Mat());

	for (int i = 0; i < 255; i++) {
		line(dispRed, Point(bin*(i), height), Point(bin*(i), height - cvRound(imageRed.at<float>(i))), Scalar(0, 0, 255), 2, 8, 0);
		line(dispGreen, Point(bin*(i), height), Point(bin*(i), height - cvRound(imageGreen.at<float>(i))), Scalar(0, 255, 0), 2, 8, 0);
		line(dispBlue, Point(bin*(i), height), Point(bin*(i), height - cvRound(imageBlue.at<float>(i))), Scalar(255, 0, 0), 2, 8, 0);
	}

	namedWindow("src", 0);
	imshow("STDIO OpenCV Sample", imageSrc);
	imshow("H blue", dispBlue);
	imshow("H green", dispGreen);
	imshow("H red", dispRed);

	// Wait input and exit
	waitKey(0);

	return 0;
}
