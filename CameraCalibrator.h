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
	//����� � 3D ������������
	std::vector<std::vector<cv::Point3f>> objectPoints;

	//����� � ��������
	std::vector<std::vector<cv::Point2f>> imagePoints;
	
	//������� ����������, ������������ ���������
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;

	//������������ ��� ������� ���������� ����������
	int flag;
public:
	//�����������
	CameraCalibrator() : flag(0) {};

	//�������� �����, �������� �� ������������
	int addChessboardPoints(const std::vector<std::string>& filelist,cv::Size & boardSize);

	//���������� �����
	void addPoints(const std::vector<cv::Point2f>&imageCorners, const std::vector<cv::Point3f>& objectCorners);

	// ����������
	double calibrate(cv::Size &imageSize);

	//����������� ������� ���������� ����������
	cv::Mat showM();

};

