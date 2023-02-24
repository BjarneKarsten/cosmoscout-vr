////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

// SPDX-FileCopyrightText: German Aerospace Center (DLR) <cosmoscout@dlr.de>
// SPDX-License-Identifier: MIT

#ifndef CS_SCENE_CELESTIAL_OBSERVER_HPP
#define CS_SCENE_CELESTIAL_OBSERVER_HPP

#include "../cs-utils/AnimatedValue.hpp"
#include "CelestialAnchor.hpp"
#include <vector>
#include <tinysplinecxx.h>

namespace cs::scene {

/// The CelestialObserver represents the camera in the scene. It provides methods for moving the
/// camera around, where the camera position and rotation is being interpolated from the start to
/// the end location.
class CS_SCENE_EXPORT CelestialObserver : public CelestialAnchor {
 public:
  explicit CelestialObserver(std::string const& sCenterName = "Solar System Barycenter",
      std::string const&                        FrameName   = "J2000");

  /// A glm vector and a corresponding time stamp in the real world, in TDB.
  struct timedVector;

  /// Updates position and rotation according to the last moveTo call.
  virtual void updateMovementAnimation(double tTime, double speed);

  /// Returns a time between startTime and endTime, such that it eases in and out.
  double easeInOut(double tTime, double startTime, double endTime);

  /// These are overidden here because they are ignored if any animation done by MoveTo is in
  /// progress.
  void setPosition(glm::dvec3 vPos) override;
  void setRotation(glm::dquat qRot) override;

  /// Calls setCenterName() and setFrameName() but updates position and rotation in such a way that
  /// the universal position and orientation does not change. This may throw a std::runtime_error if
  /// no sufficient SPICE data is available.
  void changeOrigin(std::string sCenterName, std::string sFrameName, double dSimulationTime);

  /// Gradually moves the observer's position and rotation from their current values along a spline
  /// defined by the given values.
  ///
  /// @param sCenterName      The SPICE name of the targets center.
  /// @param sFrameName       The SPICE reference frame of the targets location.
  /// @param positionControl  The vector listing the control points for the movement together with
  ///                         corresponding real world time stamps.
  /// @param lookAtControl    The vector listing the control points towards which the camera should 
  ///                         point, together with corresponding real world time stamps.
  /// @param upControl        The vector listing up vectors together with corresponding time stamps.
  /// @param dSimulationTime  The current time of the simulation in Barycentric Dynamical Time.
  /// @param dRealStartTime   The time in the real world, when the animation should start, in TDB.    
  void moveTo(std::string const& sCenterName, std::string const& sFrameName, 
      std::vector<struct timedVector> positionControl, 
      std::vector<struct timedVector> lookAtControl, std::vector<struct timedVector> upControl, 
      double dSimulationTime, double dRealStartTime);

  /// Gradually moves the observer's position and rotation from their current values to the given
  /// values.
  ///
  /// @param sCenterName      The SPICE name of the targets center.
  /// @param sFrameName       The SPICE reference frame of the targets location.
  /// @param position         The target position in the targets coordinate system.
  /// @param rotation         The target rotation in the targets coordinate system.
  /// @param dSimulationTime  The current time of the simulation in Barycentric Dynamical Time.
  /// @param dRealStartTime   The time in the real world, when the animation should start, in TDB.
  /// @param dRealEndTime     The time in the real world, when the animation should finish, in TDB.
  void moveTo(std::string const& sCenterName, std::string const& sFrameName,
      glm::dvec3 const& position, glm::dquat const& rotation, double dSimulationTime,
      double dRealStartTime, double dRealEndTime);

  /// @return true, if the observer is currently being moved.
  bool isAnimationInProgress() const;

 protected:
  utils::AnimatedValue<glm::dvec3> mAnimatedPosition;
  utils::AnimatedValue<glm::dquat> mAnimatedRotation;
  tinyspline::BSpline positionSpline;
  tinyspline::BSpline lookAtSpline;
  tinyspline::BSpline upSpline;
  
  double startTime;
  double endTime;
  double posStartTime;
  double posEndTime;
  double lAStartTime;
  double lAEndTime;
  double upStartTime;
  double upEndTime;
  bool mAnimationInProgress = false;


};
} // namespace cs::scene

#endif // CS_SCENE_CELESTIAL_OBSERVER_HPP
