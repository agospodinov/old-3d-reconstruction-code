#ifndef CVIMAGEWIDGET_H
#define CVIMAGEWIDGET_H

#include <qt4/QtGui/QWidget>
#include <qt4/QtGui/QImage>
#include <qt4/QtGui/QPainter>

#include <memory>

#ifndef Q_MOC_RUN
#include <opencv2/opencv.hpp>
#endif

#include "Vision/Core/IImage.h"

class CVImageWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CVImageWidget(QWidget *parent = 0);

    QSize sizeHint() const { return image.size(); }
    QSize minimumSizeHint() const { return image.size(); }

signals:
    void QueueRepaint();

public slots:
    void ShowImage(const std::unique_ptr<Xu::Vision::Core::IImage> &image);
    void ShowImage(const std::shared_ptr<Xu::Vision::Core::IImage> &image);

protected:
    void paintEvent(QPaintEvent *event);


private:
    QImage image;
    
};

#endif // CVIMAGEWIDGET_H
