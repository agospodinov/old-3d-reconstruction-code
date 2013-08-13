#include "CVImageWidget.h"
#include <qt4/QtCore/QDebug>

CVImageWidget::CVImageWidget(QWidget *parent) :
    QWidget(parent)
{
    this->image = QImage();
    connect(this, SIGNAL(QueueRepaint()), SLOT(repaint()), Qt::QueuedConnection);
}

void CVImageWidget::ShowImage(const std::unique_ptr<Xu::Vision::Core::IImage> &image)
{
    cv::Mat mat = image->GetMatrix();

    this->image = QImage(mat.data,
                         mat.size().width,
                         mat.size().height,
                         mat.step,
                         QImage::Format_RGB888).copy();

    emit QueueRepaint();
}

void CVImageWidget::ShowImage(const std::shared_ptr<Xu::Vision::Core::IImage> &image)
{
    cv::Mat mat = image->GetMatrix();

    this->image = QImage(mat.data,
                         mat.size().width,
                         mat.size().height,
                         mat.step,
                         QImage::Format_RGB888).copy();

    emit QueueRepaint();
}

void CVImageWidget::paintEvent(QPaintEvent *)
{
    // Display the image
    QPainter painter(this);
    painter.drawImage(QPoint(0,0), image.rgbSwapped().scaledToHeight(this->height()));
    painter.end();
}
