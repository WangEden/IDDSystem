#include "send.h"

 
Send::Send(QObject *parent)
    : QTcpSocket(parent)
{
 
}
 
void Send::connectServer( QString ip,unsigned short port)
{
    m_socket=new QTcpSocket;
    m_socket->connectToHost(QHostAddress(ip.trimmed()),port);
    //QTcpSocket::connected该信号属于子线程，无法直接在主线程中调用
    //于是通过信号槽机制，定义一个SendFile的信号，将子线程的信号传递给主线程
    connect(m_socket, &QTcpSocket::connected, this, &Send::connectSuccessfully);
    connect(m_socket, &QTcpSocket::disconnected, this, [=](){
        m_socket->close();
        m_socket->deleteLater();
        emit connectDestroyed();
    });
}
 
void Send::sendFile(QString filePath)
{
    QFile file(filePath);
    file.open(QFile::ReadOnly);
    QFileInfo fileinfo(filePath);
    
    // 输出文件大小
    qint64 filesize = fileinfo.size();
    qDebug() << "File size:" << filesize << "bytes";
    qint64 len = 0;
    do
    {
        /* code */
        char buf[4*1024] = {0};
        len = file.read(buf,sizeof(buf));
        m_socket->write(buf,len);
    } while (len>0);  
}

void Send::sendmessage(QByteArray bit)
{
    m_socket->write(bit);
}
 