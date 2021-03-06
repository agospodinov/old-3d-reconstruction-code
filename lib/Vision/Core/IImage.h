#ifndef IIMAGE_H
#define IIMAGE_H

#include "Math/LinearAlgebra/Matrix.h"
#include <opencv2/core/core.hpp>

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class IImage
            {
                public:
                    /// an interface class doesn't seem like the best place
                    /// for definitions. move out of here.

                    class Mask
                    {
                        public:
                            Mask(const Mask &other) = default;
                            Mask(const cv::Mat &mask)
                                : mask(mask)
                            {
                            }

                            Mask &operator =(const Mask &other) = default;

                            inline const cv::Mat &GetMask() const
                            {
                                return mask;
                            }

                        private:
                            cv::Mat mask;
                    };

                    struct Size
                    {
                        public:
                            Size(int width, int height)
                                : width(width),
                                  height(height)
                            {
                            }

                            int width, height;
                    };

                    struct Pixel
                    {
                        public:
                            Pixel()
                                : red(0),
                                  green(0),
                                  blue(0)
                            {
                            }

                            Pixel(uchar red, uchar green, uchar blue)
                                : red(red),
                                  green(green),
                                  blue(blue)
                            {
                            }

                            uchar red, green, blue;
                    };

                    virtual ~IImage();

                    virtual Pixel GetPixel(int x, int y) const = 0;
                    virtual int GetDepthPixel(int x, int y) const = 0;

                    virtual cv::Mat GetMatrix() const = 0;
                    virtual cv::Mat GetDepthMatrix() const = 0;

                    virtual IImage *Copy() const = 0;
                    virtual std::shared_ptr<IImage> ApplyMask(const Mask &mask) const = 0;

                    virtual Size GetSize() const = 0;

                    virtual bool HasColor() const = 0;
                    virtual bool HasDepth() const = 0;

                    virtual void ClearMemory() = 0;
            };
        }
    }
}

#endif // IIMAGE_H
