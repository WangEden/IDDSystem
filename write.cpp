#include "write.h"


Write::Write(QObject *parent)
    : QObject(parent)
{
 
}

void Write::writemessage(int i,QString text)
{
    QString filePath = "/home/hy/Desktop/result_r/USTB-305Team-R1.txt";
    if(QFile::exists(filePath) && (reserve_cout == 0)) 
    {
        QFile::remove(filePath);
        reserve_cout ++ ;
    }
    QFile outputFile(filePath);
    if(outputFile.open(QIODevice::ReadWrite | QIODevice::Append))
    {
        QTextStream out(&outputFile);
        if(i == -1)
            out << text << '\n';
        else if(i == -2)
        {
            out << text;
            emit SendWrite();
            emit StopCamera();
        } 
        else
        {
            switch(i)
            {
                case 0:
                out << "Goal_ID=CA001;" << "Num=" << text << '\n';break;
                case 1:
                out << "Goal_ID=CA002;" << "Num=" << text << '\n';break;
                case 2:
                out << "Goal_ID=CA003;" << "Num=" << text << '\n';break;
                case 3:
                out << "Goal_ID=CA004;" << "Num=" << text << '\n';break;
                case 4:
                out << "Goal_ID=CB001;" << "Num=" << text << '\n';break;
                case 5:
                out << "Goal_ID=CB002;" << "Num=" << text << '\n';break;
                case 6:
                out << "Goal_ID=CB003;" << "Num=" << text << '\n';break;
                case 7:
                out << "Goal_ID=CB004;" << "Num=" << text << '\n';break;
                case 8:
                out << "Goal_ID=CC001;" << "Num=" << text << '\n';break;
                case 9:
                out << "Goal_ID=CC002;" << "Num=" << text << '\n';break;
                case 10:
                out << "Goal_ID=CC003;" << "Num=" << text << '\n';break;
                case 11:
                out << "Goal_ID=CC004;" << "Num=" << text << '\n';break;
                case 12:
                out << "Goal_ID=CD001;" << "Num=" << text << '\n';break;
                case 13:
                out << "Goal_ID=CD002;" << "Num=" << text << '\n';break;
                case 14:
                out << "Goal_ID=CD003;" << "Num=" << text << '\n';break;
                case 15:
                out << "Goal_ID=CD004;" << "Num=" << text << '\n';break;
                case 17:
                out << "Goal_ID=W001;" << "Num=" << text << '\n';break;
                case 18:
                out << "Goal_ID=W002;" << "Num=" << text << '\n';break;
                case 19:
                out << "Goal_ID=W003;" << "Num=" << text << '\n';break;
                case 20:
                out << "Goal_ID=W004;" << "Num=" << text << '\n';break;
                default:
                break;
            }
        }
    }
   outputFile.flush();
   outputFile.close();
}

 