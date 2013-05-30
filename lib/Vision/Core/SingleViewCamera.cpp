#include "Vision/Core/SingleViewCamera.h"

#include "Vision/Core/IImage.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            SingleViewCamera::SingleViewCamera(std::unique_ptr<Input::IInputSource> inputSource)
            {
                if (inputSource != NULL)
                {
                    StartProcessing(std::move(inputSource));
                }
            }

            SingleViewCamera::~SingleViewCamera()
            {
            }

            void SingleViewCamera::RegisterFrameVisualizationHandler(FrameVisualizationHandler frameHandler)
            {
                this->frameVisualizationHandler = frameHandler;
            }

            void SingleViewCamera::StartProcessing(std::unique_ptr<Input::IInputSource> inputSource)
            {
                capture = std::move(inputSource);
            }

            bool SingleViewCamera::IsFinished() const
            {
                return (capture == NULL || !HasNextPointOfView());
            }

            std::shared_ptr<PointOfView> SingleViewCamera::GetNextPointOfView()
            {
                std::shared_ptr<PointOfView> pointOfView;
                if (HasNextPointOfView())
                {
                    pointOfView = std::make_shared<PointOfView>(shared_from_this(), capture->GetNextFrame());
                    if (frameVisualizationHandler != NULL)
                    {
                        frameVisualizationHandler(pointOfView);
                    }
                }
                return pointOfView;
            }

            bool SingleViewCamera::HasNextPointOfView() const
            {
                return capture->IsNextFrameAvailable();
            }
        }
    }
}
