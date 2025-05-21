#ifndef SEND_H
#define SEND_H
 
#include <QObject>
#include <QTcpSocket>
#include <QFile>
#include <QFileInfo>
#include <QHostAddress>

 
class Send : public QTcpSocket
{
    Q_OBJECT
public:
    explicit Send(QObject *parent = nullptr);
    //连接服务器
    void connectServer(QString ip, unsigned short port);
    //发送文件
    void sendFile(QString filePath);
    void sendmessage(QByteArray bit);
 
private:
    QTcpSocket *m_socket;
signals:
    //成功连接服务器的信号
    void connectSuccessfully();
    //断开服务器连接信号
    void connectDestroyed();
 
};
 
#endif // SEND_H