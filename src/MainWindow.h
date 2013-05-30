#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <memory>

#include <qt4/QtGui/QMainWindow>

#include "Vision/Input/CameraInputSource.h"
#include "Vision/Input/ImageListInputSource.h"
#include "Vision/Core/SingleViewCamera.h"
#include "Vision/Core/PointOfView.h"
#include "Vision/Core/Scene.h"
#include "Vision/Reconstruction/SceneReconstructor.h"
#include "Vision/Recognition/IObjectDetector.h"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
        Q_OBJECT

    public:
        explicit MainWindow(QWidget *parent = 0);
        ~MainWindow();

    public slots:
        void OnRecordButtonClicked();
        void OnOpenButtonClicked();
        void OnImagesButtonClicked();

        void OnReconstructButtonClicked();
        void OnDetectObjectsButtonClicked();

        void OnFrameSliderMoved(int position);
        void OnDetectedObjectSelected(int row);

    protected:
        void OnFrameProcessed(std::shared_ptr<Xu::Vision::Core::PointOfView> pointOfView);

    private:
        Ui::MainWindow *ui;

        std::shared_ptr<Xu::Vision::Core::SingleViewCamera> camera;
        std::unique_ptr<Xu::Vision::Reconstruction::SceneReconstructor> sceneReconstructor;
        std::unique_ptr<Xu::Vision::Recognition::IObjectDetector> objectDetector;

        std::vector<Xu::Core::Object> objects;
        std::vector<std::shared_ptr<Xu::Vision::Core::PointOfView> > pointsOfView;
};

#endif // MAINWINDOW_H
