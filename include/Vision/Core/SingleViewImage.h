#ifndef SINGLEVIEWIMAGE_H
#define SINGLEVIEWIMAGE_H

#include "IImage.h"

#include <opencv2/core/core.hpp>

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class SingleViewImage : public IImage
            {
                public:
                    SingleViewImage(const Math::Matrix &image);
                    SingleViewImage(const cv::Mat &image);

                    virtual IImage::Pixel GetPixel(int x, int y) const;
                    virtual int GetDepthPixel(int x, int y) const;

                    virtual Math::Matrix GetMatrix() const;
                    virtual Math::Matrix GetDepthMatrix() const;

                    virtual std::shared_ptr<IImage> Copy() const;
                    virtual std::shared_ptr<IImage> ApplyMask(const Mask &mask) const;

                    virtual IImage::Size GetSize() const;

                    virtual bool HasColor() const;
                    virtual bool HasDepth() const;

                    virtual cv::Mat ToOpenCVMat() const;

                    virtual void ClearMemory();

                private:
                    Math::Matrix image;
                    IImage::Size imageSize;
            };
        }
    }
}

#endif // SINGLEVIEWIMAGE_H
