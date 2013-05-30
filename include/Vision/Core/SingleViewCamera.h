#ifndef CAMERA_H
#define CAMERA_H

#include <queue>
#include <memory>
#include <mutex>
#include <thread>
#include <functional>
#include <future>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/gpu/gpu.hpp>

#include "Vision/Input/IInputSource.h"
#include "ICamera.h"
#include "PointOfView.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class PointOfView;
            class Scene;

            class SingleViewCamera : public ICamera
            {
                public:
                    SingleViewCamera(std::unique_ptr<Input::IInputSource> inputSource = NULL);
                    virtual ~SingleViewCamera();

                    typedef std::function<void(std::shared_ptr<PointOfView>)> FrameVisualizationHandler;

                    void RegisterFrameVisualizationHandler(FrameVisualizationHandler);

                    void StartProcessing(std::unique_ptr<Input::IInputSource> inputSource);
                    bool IsFinished() const;

                    std::shared_ptr<PointOfView> GetNextPointOfView();
                    bool HasNextPointOfView() const;

                private:
                    std::unique_ptr<Input::IInputSource> capture;

                    FrameVisualizationHandler frameVisualizationHandler;
            };

        }
    }
}

#endif // CAMERA_H
