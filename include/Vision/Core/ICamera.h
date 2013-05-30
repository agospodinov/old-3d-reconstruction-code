#ifndef ICAMERA_H
#define ICAMERA_H

#include <memory>

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class ICamera : public std::enable_shared_from_this<ICamera>
            {
                public:

                    // TODO should contain common things for all types of cameras
                    // Currently only SingleViewCamera is implemented, but in the
                    // future something like StereoViewCamera and KinectCamera
                    // may be added.
            };
        }
    }
}

#endif // ICAMERA_H
