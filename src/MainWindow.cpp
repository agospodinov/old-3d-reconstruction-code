#include "MainWindow.h"
#include "ui_MainWindow.h"

#include <typeinfo>
#include <functional>
#include <stdexcept>

#include <qt4/QtGui/QFileDialog>
#include <qt4/QtGui/QMessageBox>

#include <pcl/visualization/pcl_visualizer.h>

#include "Vision/Core/IImage.h"
#include "Vision/Core/Scene.h"
#include "Vision/Core/FeatureSet.h"
#include "Vision/Core/PointCloud.h"
#include "Vision/Recognition/StaticObjectDetector.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    camera(std::make_shared<Xu::Vision::Core::SingleViewCamera>())
{
    ui->setupUi(this);

    camera->RegisterFrameVisualizationHandler(std::bind(&MainWindow::OnFrameProcessed, this, std::placeholders::_1));

    connect(ui->recordButton, SIGNAL(clicked()), this, SLOT(OnRecordButtonClicked()));
    connect(ui->openButton, SIGNAL(clicked()), this, SLOT(OnOpenButtonClicked()));
    connect(ui->imagesButton, SIGNAL(clicked()), this, SLOT(OnImagesButtonClicked()));

    connect(ui->reconstructButton, SIGNAL(clicked()), this, SLOT(OnReconstructButtonClicked()));
    connect(ui->detectObjectsButton, SIGNAL(clicked()), this, SLOT(OnDetectObjectsButtonClicked()));

    connect(ui->frameSelectorSlider, SIGNAL(sliderMoved(int)), this, SLOT(OnFrameSliderMoved(int)));
    connect(ui->detectedObjectsListView, SIGNAL(currentRowChanged(int)), this, SLOT(OnDetectedObjectSelected(int)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::OnRecordButtonClicked()
{
    if (camera->IsFinished() || (!camera->IsFinished() &&
            QMessageBox::warning(this, "Already running.", "The camera is already running, do you wish to stop it?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes))
    {
        cv::VideoCapture cap(0);
        camera->StartProcessing(std::unique_ptr<Xu::Vision::Input::CameraInputSource>(new Xu::Vision::Input::CameraInputSource(cap)));
    }
}

void MainWindow::OnOpenButtonClicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Video Stream"), QDir::homePath(), tr("Video Files (*.avi *.mp4)"));

    if (!fileName.isEmpty())
    {
        if (camera->IsFinished() || (!camera->IsFinished() &&
                QMessageBox::warning(this, "Already running.", "The camera is already running, do you wish to stop it?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes))
        {
            cv::VideoCapture cap(fileName.toStdString());
            camera->StartProcessing(std::unique_ptr<Xu::Vision::Input::CameraInputSource>(new Xu::Vision::Input::CameraInputSource(cap)));
        }
    }
}

void MainWindow::OnImagesButtonClicked()
{
    QStringList images = QFileDialog::getOpenFileNames(this, tr("Open Images"), QDir::homePath(), tr("Image Files (*.jpg *.png)"));

    if (!images.isEmpty())
    {
        if (camera->IsFinished() || (!camera->IsFinished() &&
                QMessageBox::warning(this, "Already running.", "The camera is already running, do you wish to stop it?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes))
        {
            std::list<std::string> stdImageList;
            foreach (QString string, images)
            {
                stdImageList.push_back(string.toStdString());
            }
            camera->StartProcessing(std::unique_ptr<Xu::Vision::Input::ImageListInputSource>(new Xu::Vision::Input::ImageListInputSource(stdImageList)));
        }
    }
}

void MainWindow::OnReconstructButtonClicked()
{
    sceneReconstructor = std::unique_ptr<Xu::Vision::Reconstruction::SceneReconstructor>(new Xu::Vision::Reconstruction::SceneReconstructor(camera));

    sceneReconstructor->Run();

    std::shared_ptr<Xu::Vision::Core::Scene> scene = sceneReconstructor->GetScene();

    bool update = false;
    if (scene->GetFeatures()->Size() != 0)
    {
        std::thread viewerThread([scene]()
        {
            std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

//            for (auto it = scene->GetFeatures()->Begin(); it != scene->GetFeatures()->End(); ++it)
            for (auto it = scene->GetPointCloud()->GetPoints().begin(); it != scene->GetPointCloud()->GetPoints().end(); ++it)
            {
                if (it->IsTriangulated())
                {
                    pcl::PointXYZRGB point;
                    point.x = (float) it->GetX(); point.y = (float) it->GetY(); point.z = (float) it->GetZ();
                    uint32_t rgb = (static_cast<uint32_t>(it->GetR()) << 16 | static_cast<uint32_t>(it->GetG()) << 8 | static_cast<uint32_t>(it->GetB()));
                    point.rgb = *reinterpret_cast<float*>(&rgb);

                    std::cout << point.x << " " << point.y << " " << point.z << std::endl;
                    std::cout << point.rgb << std::endl;

                    pclPointCloud->points.push_back(point);
                }
            }

            viewer = std::make_shared<pcl::visualization::PCLVisualizer>("3D viewer");
            viewer->setBackgroundColor(0.1, 0.1, 0.1);
            viewer->addPointCloud(pclPointCloud, "features");
//            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
            viewer->addCoordinateSystem(1.0);
            viewer->initCameraParameters();

            while (!viewer->wasStopped())
            {
                viewer->spinOnce(100);
            }
        });

        viewerThread.detach();
    }
    else
    {
        QMessageBox::warning(this, "Scene empty", "The scene has no points to display.", QMessageBox::Ok);
    }
}

void MainWindow::OnDetectObjectsButtonClicked()
{
    if (pointsOfView.size() > 0)
    {
        int index = ui->frameSelectorSlider->sliderPosition();
        const std::shared_ptr<Xu::Vision::Core::PointOfView> &pointOfView = pointsOfView.at(index);

        Xu::Core::Data<Xu::Vision::Core::PointOfView> data(*pointOfView);
        objectDetector = std::unique_ptr<Xu::Vision::Recognition::IObjectDetector<Xu::Vision::Core::PointOfView> >(new Xu::Vision::Recognition::StaticObjectDetector());

        objects = objectDetector->Detect(data);

        if (objects.size() > 0)
        {
            ui->detectedObjectsListView->addItem("Whole picture");
            for (const Xu::Core::Object &object : objects)
            {
                QString label = QString::fromStdString(object.GetLabel());
                if (object.GetConfidenceLevel() < 0.5)
                {
                    label += " - Uncertain";
                }
                ui->detectedObjectsListView->addItem(label);
            }
        }
    }
}

void MainWindow::OnFrameSliderMoved(int position)
{
    const std::shared_ptr<Xu::Vision::Core::PointOfView> &pointOfView = pointsOfView.at(position);
    ui->cvImageWidget->ShowImage(pointOfView->GetImage());
}

void MainWindow::OnDetectedObjectSelected(int row)
{
    if (row == 0)
    {
        int index = ui->frameSelectorSlider->sliderPosition();
        const std::shared_ptr<Xu::Vision::Core::PointOfView> &pointOfView = pointsOfView.at(index);
        ui->cvImageWidget->ShowImage(pointOfView->GetImage());
    }
    else
    {
        Xu::Core::Object &object = objects.at(row - 1);
        Xu::Core::Data<std::shared_ptr<Xu::Vision::Core::IImage> > visualData = object.GetData<std::shared_ptr<Xu::Vision::Core::IImage> >().at(0);
        ui->cvImageWidget->ShowImage(visualData.GetItem());
    }
}

void MainWindow::OnFrameProcessed(std::shared_ptr<Xu::Vision::Core::PointOfView> pointOfView)
{
    const std::unique_ptr<Xu::Vision::Core::IImage> &frame = pointOfView->GetImage();
    pointsOfView.push_back(pointOfView);

    ui->frameSelectorSlider->setMaximum(pointsOfView.size() - 1);
    ui->frameSelectorSlider->setSliderPosition(pointsOfView.size() - 1);

    OnFrameSliderMoved(ui->frameSelectorSlider->sliderPosition());
//    emit ui->frameSelectorSlider->sliderMoved(ui->frameSelectorSlider->sliderPosition());
}
