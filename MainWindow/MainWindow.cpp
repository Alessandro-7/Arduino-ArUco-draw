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

// провер€ем, не навелись ли на один из трЄх квадратиков с цветом
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

void updateThickness(int& thickness, cv::Mat& thicknessLay, int i) {
	thickness = i;
	thicknessLay = cv::Mat(cv::Size(640, 480), CV_8UC3, cv::Scalar(0));
}

//////////////////////////////////////////////////////////////////////////////
MainWindow::MainWindow(QWidget* parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	ui->graphicsView->setScene(new QGraphicsScene(this));
	ui->graphicsView->scene()->addItem(&pixmap);
	port = new QSerialPort("COM4");
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
	
	int dictionaryId = 0;
	bool showRejected = false;
	float markerLength = 0.1;
	bool estimatePose = true;
	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
	detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; //

	// используем 4х4 маркеры на 50
	Ptr<aruco::Dictionary> dictionary =
		aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

	cv::Mat camMatrix, distCoeffs;

	// файл калибровки камеры
	QString path = QCoreApplication::instance()->applicationDirPath() + "/out_camera_data 4x4.xml";

	getCameraParams(path.toStdString(),
		estimatePose, camMatrix, distCoeffs);

	// слой дл€ рисовани€
	cv::Mat drawingLayer = Mat(Size(640, 480), CV_8UC3, Scalar(0));

	// слой с трем€ квадратами цветом
	cv::Mat overLay = cv::Mat(Size(640, 480), CV_8UC3, Scalar(0));
	cv::Mat thicknessLay = cv::Mat(Size(640, 480), CV_8UC3, Scalar(0));

	bool isCorner = false;

	// вектор дл€ хранени€ положений маркеров на пердыдущем кадре
	std::vector< Point2f > prevCorners;

	// текущий цвет рисовани€
	cv::Scalar drawColor = Scalar(0, 0, 255);
	int thickness = 2;

	// кадр с камеры
	cv::Mat frame;

	// создаЄм и отрисовываем квадраты с цветами
	std::vector<std::pair <Scalar, std::pair <Point, Point> > > colors;
	colors.push_back(std::make_pair(Scalar(255, 0, 0), std::make_pair(Point(50, 400), Point(100, 450))));
	colors.push_back(std::make_pair(Scalar(0, 255, 0), std::make_pair(Point(115, 400), Point(165, 450))));
	colors.push_back(std::make_pair(Scalar(0, 0, 255), std::make_pair(Point(180, 400), Point(230, 450))));
	cv::rectangle(overLay, colors[0].second.first, colors[0].second.second, colors[0].first, FILLED, 8);
	cv::rectangle(overLay, colors[1].second.first, colors[1].second.second, colors[1].first, FILLED, 8);
	cv::rectangle(overLay, colors[2].second.first, colors[2].second.second, colors[2].first, FILLED, 8);


	char id = 7;
	cv::Point xy;
	

	// читаем видео
	while (video.isOpened())

	{
		char flag = 0;
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


			
			if (ids.size() > 0) {
				aruco::drawDetectedMarkers(frame, corners, ids);
				/*
				if (estimatePose) {
					for (unsigned int i = 0; i < ids.size(); i++)
						aruco::drawAxis(frame, camMatrix, distCoeffs, rvecs[i], tvecs[i],
							markerLength * 0.5f);
				}
				*/
			}
			

			if (corners.size() > 0) {
				bool isFirstAruco = false;
				int zeroAruco = -1;

				// провер€ем, есть ли у нас id=1 маркер, и есть ли вообще id=0, чтобы рисовать
				for (int i = 0; i < ids.size(); ++i) {
					if (ids[i] == 0)
						zeroAruco = i;
					if (ids[i] == 1)
						isFirstAruco = true;
					
					// регулируем толщину
					if (ids[i] == 2)
						updateThickness(thickness, thicknessLay, 2);
					if (ids[i] == 3)
						updateThickness(thickness, thicknessLay, 7);
					if (ids[i] == 4)
						updateThickness(thickness, thicknessLay, 14);

					// очищаем область рисовани€
					if (ids[i] == 5)
						drawingLayer = Mat(Size(640, 480), CV_8UC3, Scalar(0));

				}

				
				if (zeroAruco != -1) {

					xy = getCenter(corners, zeroAruco);
					drawColor = checkColor(drawColor, corners, colors, zeroAruco);
					cv:circle(frame, xy, thickness / 2, drawColor, FILLED);

					/* ≈сли всЄ на месте, то рисуем линию межу положением id=0 маркера на прошлом
					кадре, и на текущем.
					isCorner служит дл€ того, чтобы в самый первый кадр у нас ничего не сломалось
					*/
					if (isCorner && isFirstAruco) {
						line(drawingLayer, getCenter(prevCorners), xy, drawColor, thickness, 8);
						flag = 1;
					}

					// обновл€ем вектор предыдущих положений маркера prevCorners
					prevCorners = corners[zeroAruco];
					if (isFirstAruco)
						isCorner = true;
				}
				
		
			}

			// рисуем линию текущего цвета и толщины
			line(thicknessLay, cv::Point(605, 450), cv::Point(605, 400),
				drawColor, thickness, 8);
			// сводим всЄ в одну картинку
			cv::add(drawingLayer, frame, frame);
			//cv::addWeighted(overLay, 1.0, frame, 1.0, 0.0, frame);
			cv::add(thicknessLay, frame, frame);
			cv::add(overLay, frame, frame);


			// переводим в qt формат и выводим
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



			if (port->isOpen()) {
				char xHigh = (xy.x & 0xFF00) >> 8;
				char xLow = (xy.x & 0x00FF);
				char yHigh = (xy.y & 0xFF00) >> 8;
				char yLow = (xy.y & 0x00FF);
				char color = drawColor.val[0] / 255 + drawColor.val[1] / 255 * 2 +
								drawColor.val[2] / 255 * 3;
				char buf[16] = { id, xLow, xHigh, yLow, yHigh, flag, color, thickness,
								'A', 'A', 'A', 'A', 'A', 'A', 'A', 'A' };
				port->write(buf, 16);
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
