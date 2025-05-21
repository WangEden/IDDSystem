#pragma execution_character_set("utf-8")
#include "camerawidget.h"


#define CAMERA_IMAGE_WIDTH 640
#define CAMERA_IMAGE_HEIGHT 480

CameraWidget::CameraWidget(QWidget *parent) : QWidget(parent)
{
    this->noShowCameraImage = new QTimer;
    connect(this->noShowCameraImage, &QTimer::timeout, this, [=](){
        this->showCameraImage = true;
        this->noShowCameraImage->stop();
        this->update();
    }); 

    this->update();
}

void CameraWidget::showImage(bool b)
{
    this->camerIsOpened = b;
    update();
}

void CameraWidget::setImage(QPixmap img)
{
    this->cvImage = img;
    update();
}

/**********************************************************
 * @brief label大小变化事件
**********************************************************/
void CameraWidget::resizeEvent(QResizeEvent *event)
{
    this->showCameraImage = false;
    // 定时器刷新时间，当窗口大小不变后多长时间恢复图像
    // 这涉及到用户体验，设置适当的值即可
    this->noShowCameraImage->setInterval(200);
    this->noShowCameraImage->start();

    QWidget::resizeEvent(event);
}

void CameraWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);

    // 背景颜色
    painter.setPen(QColor(192, 192, 192));
    painter.setBrush(QColor(192, 192, 192));
    painter.drawRect(0, 0, this->width(), this->height());

    // 等比缩放，绘制图像
    if(this->showCameraImage && this->camerIsOpened){
        // 寻找图像大小和widget大小的缩放关系
        QRectF imgRect = this->cvImage.rect();
        QRectF widgetRect = this->rect();
        double factor = qMin(widgetRect.width() / imgRect.width(), widgetRect.height() / imgRect.height());
        // 计算新的Rect
        int imgWidth = imgRect.width() * factor;
        int imgHeight = imgRect.height() * factor;
        int startX = (this->width() - imgWidth) / 2;
        int startY = (this->height() - imgHeight) / 2;
        // 显示图像
        painter.drawPixmap(startX, startY, imgWidth, imgHeight, this->cvImage);
    }

    QWidget::paintEvent(event);
}

