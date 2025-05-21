#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QThread>
#include <QTimer>
#include <QMessageBox>
#include <QTreeWidget>
#include "send.h"
// #include "clientsocket.h"
#include "camerawidget.h"
#include "camera.h"
#include "write.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

signals:
    //主函数开始连接服务器信号
    void StartConnect(QString ip, unsigned short port);
    //主线程发送文件信号
    void sendFileSignal(QString filename);
    //send message
    void sendMessage(QByteArray bit);
    // void WriteMessage(QString text);
    // void WriteMessage(QStringList textList);
    void changestatus(int column,QString text);
    void preStart();  



private slots:
    // void on_pushButton_Connect_clicked();
    void on_pushButtonTrans_clicked();
    // void recvData(const QString &ip, const QByteArray &data);
    //连接服务器
    void on_pushButton_Connect_clicked();
    //发送文件
    void on_sendfile_clicked();
    //
    void UpdateTreewidget(int row,QString text2);
    //
    void onTimeout();
    //
    void ChangeStatus(int column,QString text);

private:
    Ui::MainWindow *ui;
    Camera *camera;
    QThread *cameraThread;
    Send *task;
    QThread *tcpThread;

    Write *writetask;
    QThread *writeThread;
    QTimer *m_timer;
    int row = 0;
    QByteArray DataType;
    QByteArray DataLength;
    QByteArray Data;
    QByteArray bit_type2;
    QByteArray bit_length2;
};



#endif // MAINWINDOW_H
