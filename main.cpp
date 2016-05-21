#include "head.h"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "iostream"
#include <vector>
#include "CameraCalibrator.h"

using namespace cv;
using namespace std;

int main( )
{
	//список изображений
	std::vector<std::string> filelist;
	string file("IM_00.jpg\0");
	for(int i=0;i<10;i++)
	{
		filelist.push_back(file);
		file[4]++;
	}

	//размерность шахматной доски
	Size boardsize(9,6);

	//экземпляр класса калибровки 
	CameraCalibrator Calibrator;

	//поиск и добавление в список особых точек углов шахматной доски на изображениях
	Calibrator.addChessboardPoints(filelist,boardsize);
	
	//внутренняя калибровка
	Mat im=imread(filelist[0],CV_LOAD_IMAGE_COLOR);
	Size ims=Size(im.cols,im.rows);
	Calibrator.calibrate(ims);

	//вывод получившейся матрицы внутренней калибровки
	Mat IntMat=Calibrator.showM();
	
	//загрузка двух изображений и поиск углов для построения гомографии
	Mat image10,image20;

	Mat image1 = imread(filelist[0],CV_LOAD_IMAGE_COLOR);
	cvtColor(image1,image10, CV_BGR2GRAY); 
	vector<cv::Point2f> points1;
	findChessboardCorners(image10,boardsize,points1);

	Mat image2 = imread(filelist[8],CV_LOAD_IMAGE_COLOR);
	cvtColor(image2,image20, CV_BGR2GRAY); 
	vector<cv::Point2f> points2;
	findChessboardCorners(image20,boardsize,points2);

	//уточнение углов
	cv::cornerSubPix(image10, points1,
		cv::Size(5,5),
		cv::Size(-1,-1),
		cv::TermCriteria(cv::TermCriteria::MAX_ITER +
		cv::TermCriteria::EPS,
		50,
		0.01));

	cornerSubPix(image20, points2,
		Size(5,5),
		Size(-1,-1),
		TermCriteria(cv::TermCriteria::MAX_ITER +
		TermCriteria::EPS,
		50,
		0.01));

	//построение гомографии
	vector<uchar> inliers(points1.size(),0);

	Mat Homo=findHomography((Mat)points1,(Mat)points2,inliers,CV_RANSAC,1.);

	 //обводка использованных при гомографии точек

	 std::vector<cv::Point2f>::const_iterator itPts=
		 points1.begin();
	 std::vector<uchar>::const_iterator itIn= inliers.begin();
	 while (itPts!=points1.end()) {
		 if (*itIn)
			 cv::circle(image1,*itPts,3,
			 cv::Scalar(255,255,255),2);
		 ++itPts;
		 ++itIn;
	 }
	 itPts= points2.begin();
	 itIn= inliers.begin();
	 while (itPts!=points2.end()) {
		 if (*itIn)
			 cv::circle(image2,*itPts,3,
			 cv::Scalar(255,255,255),2);
		 ++itPts;
		 ++itIn;
	 }

	//формирование совместного изображения
	 cv::Mat result;
	 cv::warpPerspective(image1,
		 result,
		 Homo,
		 cv::Size(2*image1.cols,
		 image1.rows));

	cv::Mat half(result,cv::Rect(0,0,image2.cols,image2.rows));
	image2.copyTo(half);

	//сохранение совместного ихображения
	imwrite( "result.jpg", result );
	
	Mat invK = IntMat.inv();
	
	//сингулярное разложение матрицы гомографии
	Mat H=invK*Homo*IntMat;
	Mat w,u,vt;
	SVD::compute(H,w,u,vt,1);

	//вычисление возможных значений параметров разложения
	Mat d1,d2,d3;
	d1=w(Rect(0,0,1,1));
	d2=w(Rect(0,1,1,1));
	d3=w(Rect(0,2,1,1));
	double a1,a2,a3;
	a1=d1.at<double>(0,0);
	a2=d2.at<double>(0,0);
	a3=d3.at<double>(0,0);

	double nn[8][3],dd[8],RR[8][3][3],tt[8][3];
	int e[8][3]={{1,1,1},{1,1,-1},{1,-1,1},{1,-1,-1},{-1,1,1},{-1,1,-1},{-1,-1,1},{-1,-1,-1}};
	for(int i=0;i<8;i++)
	{
		dd[i]=e[i][0]*a2;
		nn[i][1]=0;
		nn[i][0]=e[i][1]*sqrt((a1*a1-a2*a2)/(a1*a1-a3*a3));
		nn[i][2]=e[i][2]*sqrt((a2*a2-a3*a3)/(a1*a1-a3*a3));
		double si=(a1-e[i][0]*a3)*nn[i][0]*nn[i][2]/a2;
		double co=(a1*(nn[i][2]*nn[i][2])+e[i][0]*a3*(nn[i][0]*nn[i][0]))/a2;
		RR[i][0][0]=co;
		RR[i][0][1]=0;
		RR[i][0][2]=-e[i][0]*si;
		RR[i][1][0]=0;
		RR[i][1][1]=e[i][0];
		RR[i][1][2]=0;
		RR[i][2][0]=si;
		RR[i][2][1]=0;
		RR[i][2][2]=e[i][0]*co;
		tt[i][0]=(a1-e[i][0]*a2)*nn[i][0];
		tt[i][1]=0;
		tt[i][2]=(a1-e[i][0]*a2)*nn[i][2];
	}
	cout<<"Results:\n";
	for (int i=0;i<8;i++)
	{

		Mat R = (Mat_<double>(3,3) << RR[i][0][0],RR[i][0][1],RR[i][0][2],
			RR[i][1][0],RR[i][1][1],RR[i][1][2],
			RR[i][2][0],RR[i][2][1],RR[i][2][2]);

		Mat d = (Mat_<double>(1,1) << dd[i]);
		Mat n = (Mat_<double>(3,1) << nn[i][0],nn[i][1],nn[i][2]);
		Mat t = (Mat_<double>(3,1) << tt[i][0],tt[i][1],tt[i][2]);

		Mat s = (Mat_<double>(1,1) << determinant(u)*determinant(vt));
		R=u*R*vt;
		d=s*d;
		t=u*t;
		n=vt.inv()*n;
		//вывод 
		cout<<"\t #"<<i+1<<"\nR:\n"<<R<<"\nd:\n"<<d<<"\nn\n"<<n<<"\nt\n"<<t<<"\n\n";
	}







	//Mat Homo=findHomography(InputArray srcPoints, InputArray dstPoints, int method=0, double ransacReprojThreshold=3, OutputArray mask=noArray() )


	//     Mat image;
	//     image = imread("IMG_1645.jpg", CV_LOAD_IMAGE_COLOR);  

	//     if(! image.data )                             
	//     {
	//            cout <<  "Could not open or find the image" << std::endl ;
	//            return -1;
	//     }

	//     // Create a new matrix to hold the gray image
	//     Mat gray;

	//     // convert RGB image to gray
	//     cvtColor(image,gray, CV_BGR2GRAY);

	//  // output vectors of image points
	//std::vector<cv::Point2f> imageCorners;
	//// number of corners on the chessboard
	//cv::Size boardSize(9,6);
	//// Get the chessboard corners
	//bool found = cv::findChessboardCorners(gray,boardSize, imageCorners);

	//Draw the corners
	cv::drawChessboardCorners(image1,boardsize, points1,true); // corners have been found

	//Draw the corners
	cv::drawChessboardCorners(image2,boardsize, points2,true); // corners have been found


	// namedWindow( "Display window", CV_WINDOW_NORMAL );  
	// imshow( "Display window", image1 );                 

	//  namedWindow( "Result window", CV_WINDOW_NORMAL );   
	//  imshow( "Result window", image2 );

	waitKey(0);
	system("Pause");
	return 0;
}
