#include "mainwindow.h"
#include "/home/hy/visual_window519/build/ui_mainwindow.h"
#include <QTextCodec>
#include "yolov8_det.h"
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("可视化界面输出"); // 设置窗口标题
    ui->frame_2->setFrameStyle(QFrame::Box | QFrame::Raised);

    // 相机子线程
    this->camera = new Camera;
    this->cameraThread = new QThread;
    this->camera->moveToThread(this->cameraThread);
    connect(this->cameraThread, &QThread::finished, this->cameraThread, &QThread::deleteLater);
    connect(this->cameraThread, &QThread::finished, this->camera, &Camera::deleteLater);
    connect(this->camera, &Camera::cameraShowImage, ui->cameraWidget, &CameraWidget::setImage);		// 更新图像
    this->cameraThread->start();
    connect(ui->openCamera, &QPushButton::clicked, this->camera, &Camera::openCamera);      // 打开相机
    connect(ui->closeCamera, &QPushButton::clicked, this->camera, &Camera::closeCamera);    // 关闭相机
    connect(this->camera, &Camera::cameraIsOpen, ui->cameraWidget, &CameraWidget::showImage);
    connect(this->camera , &Camera::updateTree , this , &MainWindow::UpdateTreewidget);

    /*modify muti write*/
    QThread *writeThread = new QThread;
    Write *writetask = new Write;
    writetask->moveToThread(writeThread);
    connect(camera , &Camera::writeMessage , writetask , &Write::writemessage); 
    connect(writetask , &Write::SendWrite , this , &MainWindow::on_sendfile_clicked);
    connect(writetask , &Write::StopCamera , this->camera , &Camera::Camerastop);
    writeThread->start();

    connect(this , &MainWindow::changestatus , this , &MainWindow::ChangeStatus);
    connect(this->camera , &Camera::changestatus , this , &MainWindow::ChangeStatus);

    connect(this->camera , &Camera::startConnect , this , &MainWindow::on_pushButton_Connect_clicked);
    connect(this , &MainWindow::preStart , this->camera , &Camera::PreStart);

    emit changestatus(0,"NO");
    emit changestatus(1,"空闲");

    /*pre*/
    yolov8_start();
    emit preStart();  
}

MainWindow::~MainWindow()
{
     if(this->cameraThread->isRunning())
     {
        this->cameraThread->quit();
        this->cameraThread->wait();
    }
    delete ui;
}

void MainWindow::on_pushButtonTrans_clicked()
{
    if(ui->pushButton_Connect->text() == "断开服务器")
    {
        // emit sendMessage(ui->textEdit_SendMess->toPlainText())
    }
}

void MainWindow::on_pushButton_Connect_clicked()
{
    //点击连接服务器按钮后，主线程发送开始连接服务器的信号
    // QString ip = this->ui->lineEdit_ip->text();
    // unsigned short port = this->ui->lineEdit_port->text().toUShort();
    // emit StartConnect(ip, port);
    // emit sendMessage(DataType + DataLength + Data);
    
    //m_timer->setInterval(5000);
    //m_timer->start();

}
 

 
void MainWindow::on_sendfile_clicked()
{
    // QString filename = "/home/hy/Desktop/result_r/USTB-305Team-R1.txt";
    // // QFile file(filename);
    // // QByteArray fileData = file.readAll();
    // // file.close();

    // QFileInfo fileinfo(filename);
    
    // // 输出文件大小
    // qint64 fileSize = fileinfo.size();
    // qint32 byteSize = static_cast<qint32>(fileSize); // 文件大小转换为32位整数

    // // 将文件大小转换为4字节的QByteArray
    // bit_type2.resize(4);
    // bit_type2[0] = 0x00;
    // bit_type2[1] = 0x00;
    // bit_type2[2] = 0x00;
    // bit_type2[3] = 0x01;
    // bit_length2.resize(4);
    // // QDataStream Stream(&bit_length2 , QIODevice::WriteOnly);
    // // Stream.setByteOrder(QDataStream::BigEndian);
    // // Stream << (quint32)fileData.size();
    // bit_length2[0] = (byteSize >> 24) & 0xFF; // 高字节到低字节
    // bit_length2[1] = (byteSize >> 16) & 0xFF;
    // bit_length2[2] = (byteSize >> 8) & 0xFF;
    // bit_length2[3] = byteSize & 0xFF;

    // emit sendMessage(bit_type2 + bit_length2);
    // qDebug() << "File size:" << byteSize << "bytes";
    // qDebug() << "File size:" << byteSize << "bytes";
    // qDebug() << "File size:" << byteSize << "bytes";
    // emit sendFileSignal(filename);
}

void MainWindow::UpdateTreewidget(int row,QString text2)
{
    if(row<ui->treeWidget->topLevelItemCount())
    {
        QTreeWidgetItem *item = ui->treeWidget->topLevelItem(row);
        if(item)
        {
            item->setText(1,text2);
        }
    }
}
void MainWindow::ChangeStatus(int column,QString text)
{
    QTreeWidgetItem *item = ui->treeWidget_2->topLevelItem(0);
    if(item)
    {
        item->setText(column,text);
    }
}

void MainWindow::onTimeout()
{
    // ui->arrow->setPixmap(QPixmap(":/arrow.jpg").scaled(60,60));
}

