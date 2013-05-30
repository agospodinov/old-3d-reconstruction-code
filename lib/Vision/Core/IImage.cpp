#include "Vision/Core/IImage.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            IImage::~IImage()
            {
            }

            IImage::Mask::Mask(const IImage::Mask &other)
                : mask(other.mask)
            {
            }

            IImage::Mask::Mask(const Math::Matrix &matrix)
                : mask(matrix)
            {
            }

            IImage::Mask::Mask(const cv::Mat &matrix)
            {
                cv::Mat tmp;
                matrix.convertTo(tmp, CV_64FC1);
                mask = Math::Matrix(tmp);
            }

            IImage::Mask &IImage::Mask::operator =(const Mask &other)
            {
                Mask tmp(other);
                std::swap(mask, tmp.mask);
                return *this;
            }

            const Math::Matrix &IImage::Mask::GetMask() const
            {
                return mask;
            }
        }
    }
}
