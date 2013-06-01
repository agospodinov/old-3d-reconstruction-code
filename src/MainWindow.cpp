#include "MainWindow.h"
#include "ui_MainWindow.h"

#include <typeinfo>
#include <functional>
#include <stdexcept>

#include <qt4/QtGui/QFileDialog>
#include <qt4/QtGui/QMessageBox>

#include "Vision/Core/IImage.h"
#include "Vision/Core/VisualData.h"
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
}

void MainWindow::OnDetectObjectsButtonClicked()
{
    if (pointsOfView.size() > 0)
    {
        int index = ui->frameSelectorSlider->sliderPosition();
        const std::shared_ptr<Xu::Vision::Core::PointOfView> &pointOfView = pointsOfView.at(index);
        std::shared_ptr<Xu::Vision::Core::IImage> image = pointOfView->GetImage()->Copy();

        Xu::Vision::Core::VisualData data(image, NULL);
        objectDetector = std::unique_ptr<Xu::Vision::Recognition::IObjectDetector>(new Xu::Vision::Recognition::StaticObjectDetector());

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
        std::shared_ptr<Xu::Vision::Core::VisualData> visualData = std::dynamic_pointer_cast<Xu::Vision::Core::VisualData>(object.GetData().at(0));
        ui->cvImageWidget->ShowImage(visualData->GetImage());
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
