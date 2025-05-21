#ifndef WRITE_H
#define WRITE_H

#include <QObject>
#include <QFile>
#include <QDebug>
#include <QFileInfo>
#include <QTextStream>
#include <QApplication>

class Write : public QObject
{
    Q_OBJECT
public:
    explicit Write(QObject *parent = nullptr);
    void writemessage(int i,QString text);
    //  void writemessage(QStringList messages);
private:
    int reserve_cout = 0;
signals:    
    void SendWrite(); 
    void StopCamera();  

};











#endif 

