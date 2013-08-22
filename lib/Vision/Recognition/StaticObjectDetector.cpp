#include "Vision/Recognition/StaticObjectDetector.h"

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/gpu/gpu.hpp>

#include "Vision/Core/IImage.h"
#include "Vision/Core/PointOfView.h"
#include "Vision/Core/SingleViewImage.h"

namespace Xu
{
    namespace Vision
    {
        namespace Recognition
        {
            StaticObjectDetector::StaticObjectDetector(int spatialRadius, int colorRadius, int minSize)
                : spatialRadius(spatialRadius),
                  colorRadius(colorRadius),
                  minSize(minSize)
            {
            }

            StaticObjectDetector::~StaticObjectDetector()
            {
            }

            std::vector<Xu::Core::Object> StaticObjectDetector::Detect(Xu::Core::Data<Vision::Core::PointOfView> data)
            {
                std::cout << "Running static object detector..." << std::endl;
                std::vector<Xu::Core::Object> objects;

                cv::Mat image = data.GetItem().GetImage()->GetMatrix();

                cv::resize(image, image, cv::Size(), 0.2, 0.2);
                cv::Mat tmp; cv::cvtColor(image, tmp, CV_BGR2BGRA, 4);
                cv::gpu::GpuMat gpuImage(tmp);
                cv::Mat segmented;

                std::cout << "Running mean shift segmentation... " << std::flush;
                cv::gpu::meanShiftSegmentation(gpuImage, segmented, spatialRadius, colorRadius, minSize);
                std::cout << "Done." << std::endl;

                cv::erode(segmented, segmented, cv::Mat());
                cv::dilate(segmented, segmented, cv::Mat());
                cv::Mat segmentedGray; cv::cvtColor(segmented, segmentedGray, CV_BGRA2GRAY);
                cv::blur(segmentedGray, segmentedGray, cv::Size(3, 3));

                cv::Mat cannied;
                cv::Canny(segmentedGray, cannied, 100, 200, 3, true);

                std::vector<std::vector<cv::Point2i> > contours;
                std::vector<cv::Vec4i> hierarchy;
                cv::findContours(cannied, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

                std::vector<cv::Point2i> allPoints;
                for (int i = 0; i < contours.size(); i++)
                {
                    for (int j = 0; j < contours[i].size(); j++)
                    {
                        allPoints.push_back(contours[i][j]);
                    }
                }

                cv::Rect boundingBox = cv::boundingRect(allPoints);

                cv::Mat segmentedRgb; cv::cvtColor(segmented, segmentedRgb, CV_BGRA2BGR, 3);

                cv::Mat bgModel, fgModel;
                cv::Mat mask; mask.create(segmented.size(), CV_8UC1);

                std::cout << "Running grab cut algorithm... " << std::flush;
                cv::grabCut(segmentedRgb, mask, boundingBox, bgModel, fgModel, 3, cv::GC_INIT_WITH_RECT);
                std::cout << "Done." << std::endl;

                cv::Mat foreground;
                cv::Mat binMask = mask & 1;

                std::cout << "Applying mask... " << std::flush;
                image.copyTo(foreground, binMask);

                std::shared_ptr<Vision::Core::IImage> foregroundImage(new Vision::Core::SingleViewImage(foreground));
                Xu::Core::Data<std::shared_ptr<Vision::Core::IImage> > objectData(foregroundImage);
                std::cout << "Creating objects... " << std::flush;
                Xu::Core::Object detectedObject(0.2, "Foreground");
                detectedObject.AddData(objectData);

                objects.push_back(std::move(detectedObject));
                std::cout << "Done. " << std::endl;

                return objects;
            }
        }
    }
}
