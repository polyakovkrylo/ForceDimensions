#pragma once

#include "initForceDimensions.h"

#include <SofaUserInteraction/Controller.h>
#include <SofaHaptics/ForceFeedback.h>

#include <mutex>


/**
 * Omega driver
 */
class SOFA_FORCEDIMENSIONS_API OmegaDriver : public sofa::component::controller::Controller
{
 public:
  SOFA_CLASS(OmegaDriver, Controller);
  OmegaDriver();
  virtual ~OmegaDriver();

  virtual void init() override;
  virtual void bwdInit() override;
  virtual void reinit() override;

  sofa::core::objectmodel::Data<sofa::defaulttype::RigidTypes::Coord> d_pose;
  sofa::core::objectmodel::Data<double> d_gripperState;
  sofa::core::objectmodel::Data<sofa::defaulttype::RigidTypes::Coord> d_baseFrame;
  sofa::core::objectmodel::Data<sofa::defaulttype::RigidTypes::Deriv> d_forceFeedback;
  sofa::core::objectmodel::Data<double> d_scale;
  sofa::core::objectmodel::Data<double> d_forceScale;
  sofa::core::objectmodel::Data<double> d_maxForce;
  sofa::core::objectmodel::Data<double> d_gammaOffset;
  sofa::core::objectmodel::Data<bool> d_enableForceFeedback;
  sofa::core::objectmodel::Data<bool> d_computeForceFeedback;

 private:
  void initDevice();
  void updateDevicePose();
  void updatePose();
  virtual void onBeginAnimationStep(const double) override;

  static void * updateForceFeedback(void *args);

  bool active;
  int error;
  sofa::defaulttype::RigidTypes::Coord devicePose;
  std::mutex devicePoseMutex;

  sofa::component::controller::ForceFeedback::SPtr forceFeedback;
};
