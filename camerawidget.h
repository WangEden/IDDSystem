#ifndef CAMERAWIDGET_H
#define CAMERAWIDGET_H

#include <QWidget>
#include <QTimer>
#include <QPixmap>
#include <QDebug>
#include <QPainter>

class CameraWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CameraWidget(QWidget *parent = nullptr);

    void showImage(bool b);      // 显示图像
    void setImage(QPixmap img);     // 更新图像

protected:
    void resizeEvent(QResizeEvent *event) override;
    void paintEvent(QPaintEvent *event) override;

private:
    QPixmap cvImage;                // 相机图像
    QTimer *noShowCameraImage;      // 不显示图像定时器
    bool showCameraImage = true;    // 显示图像
    
    bool camerIsOpened = false;     // 相机打开

signals:

};

#endif // CAMERAWIDGET_H

