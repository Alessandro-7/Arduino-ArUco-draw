#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QtSerialPort/QserialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QTimer>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QImage>
#include <QPixmap>
#include <QCloseEvent>
#include <QMessageBox>
#include <QTcpSocket>
#include <QBuffer>
#include <qstring.h>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>
namespace Ui {
	class MainWindow;
}

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(QWidget* parent = 0);
	~MainWindow();

protected:
	void closeEvent(QCloseEvent* event);

private slots:
	void on_startBtn_pressed();

private:
	Ui::MainWindow* ui;

	QGraphicsPixmapItem pixmap;
	cv::VideoCapture video;

	QTcpSocket* imageTcpSocket;
	QByteArray imageData;

	QSerialPort* port;

};

#endif // MAINWINDOW_H
