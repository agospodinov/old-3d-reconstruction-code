#include "Vision/Core/SingleViewImage.h"

#include <stdexcept>

#include <opencv2/imgproc/imgproc.hpp>

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            SingleViewImage::SingleViewImage(const Math::Matrix &image)
                : image(image),
                  imageSize(Size(image.GetColumns(), image.GetRows()))
            {
            }

            SingleViewImage::SingleViewImage(const cv::Mat &image)
                : imageSize(Size(image.size().width, image.size().height))
            {
                cv::Mat tmp;
                cv::cvtColor(image, tmp, CV_BGR2RGB);
                this->image = Math::Matrix(tmp);
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
                if (image.GetChannels() == 3)
                {
                    uchar r = image.Get(x, y, 0) * 255.0;
                    uchar g = image.Get(x, y, 1) * 255.0;
                    uchar b = image.Get(x, y, 2) * 255.0;
                    rgb = Pixel(r, g, b);
                }
                else if (image.GetChannels() == 1)
                {
                    uchar intensity = image.Get(x, y) * 255.0;
                    rgb = Pixel(intensity, intensity, intensity);
                }
                return rgb;
            }

            int SingleViewImage::GetDepthPixel(int, int) const
            {
                return 0;
            }

            Math::Matrix SingleViewImage::GetMatrix() const
            {
                return image;
            }

            Math::Matrix SingleViewImage::GetDepthMatrix() const
            {
                return Math::Matrix();
            }

            std::shared_ptr<IImage> SingleViewImage::Copy() const
            {
                return std::shared_ptr<IImage>(new SingleViewImage(this->image));
            }

            std::shared_ptr<IImage> SingleViewImage::ApplyMask(const IImage::Mask &mask) const
            {
                Math::Matrix zeroMatrix = Math::Matrix::Zeros(image.GetRows(), image.GetColumns());
                std::shared_ptr<IImage> maskedImage(new SingleViewImage(zeroMatrix));

                for (int i = 0; i < GetSize().width; i++)
                {
                    for (int j = 0; j < GetSize().height; j++)
                    {
                        if (static_cast<uchar>(mask.GetMask().Get(i, j)) == 1)
                        {
                            maskedImage->GetMatrix().Set(i, j, image.Get(i, j));
                        }
                    }
                }

                return maskedImage;
            }

            bool SingleViewImage::HasColor() const
            {
                return (image.GetChannels() == 3);
            }

            bool SingleViewImage::HasDepth() const
            {
                return false;
            }

            cv::Mat SingleViewImage::ToOpenCVMat() const
            {
                cv::Mat result;
                GetMatrix().ToCvMat().convertTo(result, CV_8UC3, 255.0);
                return result;
            }

            void SingleViewImage::ClearMemory()
            {
                image.Release();
            }

            IImage::Size SingleViewImage::GetSize() const
            {
                return imageSize;
            }

        }
    }
}
