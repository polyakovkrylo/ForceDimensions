#include "OmegaDriver.h"
#include <dhdc.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/events/SimulationStartEvent.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/Node.h>

using sofa::simulation::Node;
using sofa::simulation::AnimateBeginEvent;
using sofa::simulation::SimulationStartEvent;

using sofa::core::objectmodel::BaseContext;
using sofa::core::objectmodel::Event;

using sofa::defaulttype::Vec3;
using sofa::defaulttype::Quat;
typedef sofa::defaulttype::RigidTypes::Coord Coord;
typedef sofa::defaulttype::RigidTypes::Deriv Deriv;

using sofa::component::controller::ForceFeedback;

void * OmegaDriver::updateForceFeedback(void *args) {
    OmegaDriver *driver = reinterpret_cast<OmegaDriver*>(args);

    for (;;) {
        if (!driver->active)
            continue;

        driver->updateDevicePose();

	if (driver->d_computeForceFeedback.getValue()) {
	  double fx,fy,fz;
	  auto pos = driver->devicePose.getCenter();

	  // maybe compute wrench instead?
	  driver->forceFeedback->computeForce(pos[0], pos[1], pos[2],
					      0,0,0,0,
					      fx, fy, fz);

	  auto fdev = driver->d_baseFrame.getValue().unprojectVector(Vec3(fx,fy,fz));
	  driver->d_forceFeedback.setValue(Deriv(fdev,Vec3()));

	  if (driver->forceFeedback && driver->d_enableForceFeedback.getValue()) {
	    dhdSetForce(fdev[2], fdev[0], fdev[1]);
	  }
	}
    }

    return nullptr;
}

OmegaDriver::OmegaDriver() :
  d_pose(initData(&d_pose, "pose", "Pose of the end-effector")),
  d_gripperState(initData(&d_gripperState, "gripperState", "State of the gripper")),
  d_baseFrame(initData(&d_baseFrame, "baseFrame", "Pose of the base frame of the device")),
  d_forceFeedback(initData(&d_forceFeedback, "forceFeedback", "Rendered force")),
  d_scale(initData(&d_scale, "scale", "Scale of the device in the simulation world")),
  d_forceScale(initData(&d_forceScale, "forceScale", "Force feedback scale")),
  d_maxForce(initData(&d_maxForce, "maxForce", "Maximum rendered force")),
  d_gammaOffset(initData(&d_gammaOffset, "gammaOffset", "Wrist gamma angle offset")),
  d_enableForceFeedback(initData(&d_enableForceFeedback, "enableForceFeedback", "Enable force feedback")),
  d_computeForceFeedback(initData(&d_computeForceFeedback, "computeForceFeedback", "Enable force feedback computation")),
  error(0),
  forceFeedback(nullptr)
{
  f_listening.setValue(true);
  d_gammaOffset.setValue(1.15);
  d_computeForceFeedback.setValue(true);
  d_enableForceFeedback.setValue(true);
}

OmegaDriver::~OmegaDriver()
{
    dhdClose();
}

void OmegaDriver::init()
{

}

void OmegaDriver::bwdInit()
{ 
    Node *context = dynamic_cast<Node *>(this->getContext()); // access to current node
    forceFeedback = context->get<ForceFeedback>(this->getTags(), BaseContext::SearchRoot);

    try {
        initDevice();
    }  catch (std::runtime_error &e) {
        msg_warning() << "Failed to initialize the device: " << e.what();
    }
}

void OmegaDriver::reinit()
{
  active = false;
}

void OmegaDriver::initDevice()
{
    auto handleError = [&]() {
        error = dhdErrorGetLast();
        throw std::runtime_error(dhdErrorGetLastStr());
    };

    // get device count
    if (dhdGetDeviceCount() <= 0) {
        handleError();
    }

    // open the first available device
    if (dhdOpen() < 0)
        handleError();

    if(dhdSetGravityCompensation(DHD_ON))
        handleError();

    if(dhdEnableForce(DHD_ON))
        handleError();

    reinit();
    
    dhdSetForceAndTorqueAndGripperForce (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    dhdStartThread(updateForceFeedback,this, DHD_THREAD_PRIORITY_HIGH);

    updatePose();
}

void OmegaDriver::updateDevicePose()
{
    // retrive the pose from the device
    double x, y, z;
    double oa, ob, og;

    dhdGetPositionAndOrientationRad(&x, &y, &z, &oa, &ob, &og);

    og -= d_gammaOffset.getValue();

    x *= d_scale.getValue();
    y *= d_scale.getValue();
    z *= d_scale.getValue();

    // update the pose data
    devicePoseMutex.lock();
    devicePose.getCenter() = Vec3(y, z, x);
    devicePose.getOrientation() = Quat::fromEuler(ob, og, oa, Quat::EulerOrder::XZY);
    devicePose = d_baseFrame.getValue().mult(devicePose);
    devicePoseMutex.unlock();
}

void OmegaDriver::updatePose()
{
    // update the pose data
    auto pose = d_pose.beginEdit();
    devicePoseMutex.lock();
    *pose = devicePose;
    devicePoseMutex.unlock();
    d_pose.endEdit();
}

void OmegaDriver::onBeginAnimationStep(const double)
{
    if(!active)
        active = true;

    if (!error)
        updatePose();
}

int OmegaDriverClass = sofa::core::RegisterObject("ForceDimensions Omega haptic interface driver.")
.add< OmegaDriver >()
.addAlias("HapticDevice")
;
