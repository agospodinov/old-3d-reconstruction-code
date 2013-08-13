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

#include "Vision/Core/ICamera.h"
#include "Vision/Core/PointOfView.h"
#include "Vision/Input/IInputSource.h"

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

                    /// refactor
                    typedef std::function<void(std::shared_ptr<PointOfView>)> FrameVisualizationHandler;

                    void RegisterFrameVisualizationHandler(FrameVisualizationHandler);
                    /// end refactor

                    /// prepare std::future<POV::Ptr>?
                    void StartProcessing(std::unique_ptr<Input::IInputSource> inputSource);

                    /// delete
                    bool IsFinished() const;
                    /// end delete

                    /// refactor. possibly use inversion of control
                    std::shared_ptr<PointOfView> GetNextPointOfView();
                    bool HasNextPointOfView() const;
                    /// end refactor

                private:
                    std::unique_ptr<Input::IInputSource> capture;

                    FrameVisualizationHandler frameVisualizationHandler;
            };

        }
    }
}

#endif // CAMERA_H
