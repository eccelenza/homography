#include "CameraCalibrator.h"

int CameraCalibrator::addChessboardPoints(const std::vector<std::string>& filelist,cv::Size & boardSize) 
{

	std::vector<cv::Point2f> imageCorners;
	std::vector<cv::Point3f> objectCorners;

	// Углы в 3D пространстве (X,Y,Z)= (i,j,0)
	for (int i=0; i<boardSize.height; i++) {
		for (int j=0; j<boardSize.width; j++) {
			objectCorners.push_back(cv::Point3f(i, j, 0.0f));
		}
	}

	cv::Mat image,image1;

	int successes = 0; //счетчик удачных изображений

	for (int i=0; i<filelist.size(); i++) {
		//загрузка изображения и перевод в черно-белое  
		image1 = cv::imread(filelist[i],CV_LOAD_IMAGE_COLOR);
		cvtColor(image1,image, CV_BGR2GRAY);  

		// поиск углов в пикселях
		bool found = cv::findChessboardCorners(image, boardSize, imageCorners);
		if (found)
		{
		//уточнение
		cv::cornerSubPix(image, imageCorners,
			cv::Size(5,5),
			cv::Size(-1,-1),
			cv::TermCriteria(cv::TermCriteria::MAX_ITER +
			cv::TermCriteria::EPS,
			30,
			0.1));

		//добавление особых точек
		if (imageCorners.size() == boardSize.area()) {
			addPoints(imageCorners, objectCorners);
			successes++;

		}
		}
	}
	return successes;
}

void CameraCalibrator::addPoints(const std::vector<cv::Point2f>&imageCorners, const std::vector<cv::Point3f>& objectCorners) {
		 //добавление особой точки
		imagePoints.push_back(imageCorners);
		objectPoints.push_back(objectCorners);
}

double CameraCalibrator::calibrate(cv::Size &imageSize)
{
	std::vector<cv::Mat> rvecs, tvecs;
	return
		calibrateCamera(objectPoints,
		imagePoints,
		imageSize,
		cameraMatrix,
		distCoeffs,
		rvecs, tvecs,
		flag);
}

cv::Mat CameraCalibrator::showM()
{
	return cameraMatrix;
}