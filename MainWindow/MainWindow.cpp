#include "mainwindow.h"
#include "x64\Debug\uic\ui_MainWindow.h"

#include<mainwindow.h>

static bool readCameraParameters(std::string filename, cv::Mat& camMatrix, cv::Mat& distCoeffs) {
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	return true;
}

void getCameraParams(std::string path, bool estimatePose, cv::Mat& camMatrix, cv::Mat& distCoeffs) {
	if (estimatePose) {
		bool readOk = readCameraParameters(path, camMatrix, distCoeffs);
		if (!readOk) {
			std::cerr << "Invalid camera file" << std::endl;
			//   return 0;
		}
	}
}

cv::Point getCenter(std::vector< std::vector< cv::Point2f > > corners, int i)
{
	return corners[i][0] + (corners[i][2] - corners[i][0]) / 2;
}

cv::Point getCenter(std::vector< cv::Point2f > corners)
{
	return corners[0] + (corners[2] - corners[0]) / 2;
}

// ���������, �� �������� �� �� ���� �� ��� ����������� � ������
cv::Scalar checkColor(cv::Scalar currColor, std::vector< std::vector< cv::Point2f > > corners,
	std::vector<std::pair <cv::Scalar, std::pair <cv::Point, cv::Point> > > colors, int zeroAruco) {
	cv::Point center = getCenter(corners, zeroAruco);
	for (auto color : colors)
	{
		if (center.x > color.second.first.x&& center.y > color.second.first.y&&
			center.x < color.second.second.x && center.y < color.second.second.y) {
			currColor = color.first;
		}
	}

	return currColor;
}


MainWindow::MainWindow(QWidget* parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	ui->graphicsView->setScene(new QGraphicsScene(this));
	ui->graphicsView->scene()->addItem(&pixmap);
	port = new QSerialPort("COM1");
	if (port->open(QIODevice::ReadWrite))
	{
		port->setBaudRate(QSerialPort::Baud9600);
		port->setDataBits(QSerialPort::Data8);
		port->setFlowControl(QSerialPort::NoFlowControl);
		port->setParity(QSerialPort::NoParity);
		port->setStopBits(QSerialPort::OneStop);
		ui->statusBar->setText("STATUS: PORT OPENED");
	}
	else {
		ui->statusBar->setText("STATUS: PORT ERROR");
	}
}

MainWindow::~MainWindow()
{
	delete ui;
}

void MainWindow::on_startBtn_pressed()
{
	using namespace cv;

	if (video.isOpened())
	{
		ui->startBtn->setText("Start");
		video.release();
		return;
	}

	bool isCamera;
	int cameraIndex = ui->videoEdit->text().toInt(&isCamera);
	if (isCamera)
	{
		if (!video.open(cameraIndex))
		{
			QMessageBox::critical(this,
				"Camera Error",
				"Make sure you entered a correct camera index,"
				"<br>or that the camera is not being accessed by another program!");
			return;
		}
	}
	
	ui->startBtn->setText("Stop");
	ui->statusBar->setText("Test");
	int dictionaryId = 0;
	bool showRejected = false;
	float markerLength = 0.1;
	bool estimatePose = true;
	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
	detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; //

	// ���������� 4�4 ������� �� 50
	Ptr<aruco::Dictionary> dictionary =
		aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

	cv::Mat camMatrix, distCoeffs;

	// ���� ���������� ������
	QString path = QCoreApplication::instance()->applicationDirPath() + "/out_camera_data 4x4.xml";

	getCameraParams(path.toStdString(),
		estimatePose, camMatrix, distCoeffs);

	// ���� ��� ���������
	cv::Mat drawingLayer = Mat(Size(640, 480), CV_8UC3, Scalar(0));

	// ���� � ����� ���������� ������
	cv::Mat overLay = cv::Mat(Size(640, 480), CV_8UC3, Scalar(0));
	bool isCorner = false;

	// ������ ��� �������� ��������� �������� �� ���������� �����
	std::vector< Point2f > prevCorners;

	// ������� ���� ���������
	cv::Scalar drawColor = Scalar(0, 0, 255);

	// ���� � ������
	cv::Mat frame;

	// ������ � ������������ �������� � �������
	std::vector<std::pair <Scalar, std::pair <Point, Point> > > colors;
	colors.push_back(std::make_pair(Scalar(255, 0, 0), std::make_pair(Point(50, 400), Point(100, 450))));
	colors.push_back(std::make_pair(Scalar(0, 255, 0), std::make_pair(Point(115, 400), Point(165, 450))));
	colors.push_back(std::make_pair(Scalar(0, 0, 255), std::make_pair(Point(180, 400), Point(230, 450))));
	cv::rectangle(overLay, colors[0].second.first, colors[0].second.second, colors[0].first, FILLED, 8);
	cv::rectangle(overLay, colors[1].second.first, colors[1].second.second, colors[1].first, FILLED, 8);
	cv::rectangle(overLay, colors[2].second.first, colors[2].second.second, colors[2].first, FILLED, 8);

	// ������ �����
	while (video.isOpened())

	{
		bool sendingData = false;
		video >> frame;
		if (!frame.empty())
		{
			
			std::vector< int > ids;
			std::vector< std::vector< Point2f > > corners, rejected;
			std::vector< Vec3d > rvecs, tvecs;


			// detect markers and estimate pose
			aruco::detectMarkers(frame, dictionary, corners, ids, detectorParams, rejected);
			if (estimatePose && ids.size() > 0)
				aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs,
					tvecs);


			// ����� �����. ���� ������ ��� �����������
			if (ids.size() > 0) {
				aruco::drawDetectedMarkers(frame, corners, ids);

				if (estimatePose) {
					for (unsigned int i = 0; i < ids.size(); i++)
						aruco::drawAxis(frame, camMatrix, distCoeffs, rvecs[i], tvecs[i],
							markerLength * 0.5f);
				}
			}

			/*��� ��� ��� �����. ������ �������� � id=0 ������ �����, ����� � �����
			���� ������ � id=1. �� ���� �������, ��� ������� ������ "��������", � ������ � 
			�������� �� �� ���� ��� �� ������ �����.
			������� ���, ��������, ����� �������� ������� ����� � ������ �� �������� �
			��� �����. �� � ������ �������� ������ �� �� ������ ��� ���������.*/

			if (corners.size() > 0) {
				bool isFirstAruco = false;
				int zeroAruco = -1;

				// ���������, ���� �� � ��� id=1 ������, � ���� �� ������ id=0, ����� ��������
				for (int i = 0; i < ids.size(); ++i) {
					if (ids[i] == 1)
						isFirstAruco = true;
					if (ids[i] == 0)
						zeroAruco = i;
				}

				// ������ ��� �������, ����� � ��� ���� �������
				if (zeroAruco != -1)
					drawColor = checkColor(drawColor, corners, colors, zeroAruco);

				/* ���� �� �� �����, �� ������ ����� ���� ���������� id=0 ������� �� �������
				�����, � �� �������.
				isCorner ������ ��� ����, ����� � ����� ������ ���� � ��� ������ �� ���������
				*/
				if (isCorner && corners.size() > 0 && isFirstAruco && zeroAruco != -1) {
					line(drawingLayer, getCenter(prevCorners),
						getCenter(corners, zeroAruco),
						drawColor, 5, 8);
					sendingData = true;
				}

				// ��������� ������ ���������� ��������� ������� prevCorners
				if (zeroAruco != -1) {
					prevCorners = corners[zeroAruco];
					if (isFirstAruco)
						isCorner = true;
				}
			}

			// ������ �� � ���� ��������
			cv::add(drawingLayer, frame, frame);
			//cv::addWeighted(overLay, 1.0, frame, 1.0, 0.0, frame);
			cv::add(overLay, frame, frame);


			// ��������� � qt ������ � �������
			QImage qimg(frame.data,
				frame.cols,
				frame.rows,
				frame.step,
				QImage::Format_RGB888);
			pixmap.setPixmap(QPixmap::fromImage(qimg.rgbSwapped()));
			ui->graphicsView->fitInView(&pixmap, Qt::KeepAspectRatio);

			////////////////////////////////////////////////////////////////////////
			imageTcpSocket = new QTcpSocket();
			imageTcpSocket->abort();
			imageTcpSocket->connectToHost("127.0.0.1", 8082);
			QPixmap image = QPixmap::fromImage(qimg);
			QByteArray ba;
			QBuffer buffer(&ba);
			image.save(&buffer, "JPG");
			imageTcpSocket->write(ba);
			/////////////////////////////////////////////////////////////////////////

			if (port->isOpen())
			{
				char buf[14] = { 0, 22, 22, 33, 33, 111, 'A', 'A', 'A', 'A', 'A', 'A', 'A', 'A' };
				char hj[] = { '2' };
				port->write(buf, 14);
			}

			
		
		
		}
		QCoreApplication::instance()->processEvents();
	}
	
	ui->startBtn->setText("Start");

	
}

void MainWindow::closeEvent(QCloseEvent* event)
{
	if (video.isOpened())
	{
		QMessageBox::warning(this,
			"Warning",
			"Stop the video before closing the application!");
		event->ignore();
	}
	else
	{
		event->accept();
	}
}
