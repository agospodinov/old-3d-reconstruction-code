#include "Vision/Core/SingleViewImage.h"

#include <stdexcept>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            SingleViewImage::SingleViewImage(const cv::Mat &image)
                : image(image),
                  imageSize(Size(image.size().width, image.size().height))
            {
            }

            IImage::Pixel SingleViewImage::GetPixel(int x, int y) const
            {
                if (x < 0 || x > GetSize().width || y < 0 || y > GetSize().height)
                {
                    std::stringstream ss;
                    ss << "Pixel requested (" << x << ", " << y << ") outside image.";
                    throw std::out_of_range(ss.str());
                }

                Pixel rgb;
                if (HasColor())
                {
                    uchar r = image.at<uchar>(y, x);
                    uchar g = image.at<uchar>(y, x);
                    uchar b = image.at<uchar>(y, x);
                    rgb = Pixel(r, g, b);
                }
                else
                {
                    uchar intensity = image.at<uchar>(y, x);
                    rgb = Pixel(intensity, intensity, intensity);
                }
                return rgb;
            }

            int SingleViewImage::GetDepthPixel(int, int) const
            {
                return 0;
            }

            cv::Mat SingleViewImage::GetMatrix() const
            {
                return image;
            }

            cv::Mat SingleViewImage::GetDepthMatrix() const
            {
                return cv::Mat();
            }

            IImage *SingleViewImage::Copy() const
            {
                return new SingleViewImage(this->image);
            }

            std::shared_ptr<IImage> SingleViewImage::ApplyMask(const IImage::Mask &mask) const
            {
//                Math::LinearAlgebra::Matrix zeroMatrix = Math::LinearAlgebra::Matrix::Zeros(image.GetRows(), image.GetColumns());
//                std::shared_ptr<IImage> maskedImage(new SingleViewImage(zeroMatrix));
                cv::Mat masked; image.copyTo(masked, mask.GetMask());
                std::shared_ptr<IImage> maskedImage(std::make_shared<SingleViewImage>(masked));

                return maskedImage;
            }

            bool SingleViewImage::HasColor() const
            {
                return (image.channels() == 3);
            }

            bool SingleViewImage::HasDepth() const
            {
                return false;
            }

            void SingleViewImage::ClearMemory()
            {
                image.release();
            }

            IImage::Size SingleViewImage::GetSize() const
            {
                return imageSize;
            }

        }
    }
}
