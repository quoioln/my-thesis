#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
int main(int argc, char ** argv)
{
	IplImage * img = cvLoadImage("/home/quoioln/DATA/img.jpg");
	if ( img != NULL )
	{
	cvNamedWindow( "My window" );
	cvShowImage( "My window", img );
	cvWaitKey();//Đợi người dùng nhấn 1 phím bất kỳ
	cvReleaseImage( &img ); //Giải phóng vùng nhớ
	cvDestroyWindow( "My window" ); //Đóng cửa sổ
	} else {
		printf ("Error");
	}
	return 0;
}
