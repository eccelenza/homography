#pragma once
#include"head.h"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include<iostream>
#include<vector>
#include<string>



class CameraCalibrator
{
	//точки в 3D пространстве
	std::vector<std::vector<cv::Point3f>> objectPoints;

	//точки в пикселях
	std::vector<std::vector<cv::Point2f>> imagePoints;
	
	//матрица калибровки, коэффициенты дисторсии
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;

	//спецификатор для функции нахождения гомографии
	int flag;
public:
	//конструктор
	CameraCalibrator() : flag(0) {};

	//загрузка углов, проверка на корректность
	int addChessboardPoints(const std::vector<std::string>& filelist,cv::Size & boardSize);

	//добавление точек
	void addPoints(const std::vector<cv::Point2f>&imageCorners, const std::vector<cv::Point3f>& objectCorners);

	// калибровка
	double calibrate(cv::Size &imageSize);

	//возвращение матрицы внутренней калибровки
	cv::Mat showM();

};

