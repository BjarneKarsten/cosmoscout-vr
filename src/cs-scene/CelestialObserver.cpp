////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

// SPDX-FileCopyrightText: German Aerospace Center (DLR) <cosmoscout@dlr.de>
// SPDX-License-Identifier: MIT

#include "CelestialObserver.hpp"

#include "logger.hpp"
#include <iostream>

namespace cs::scene {

////////////////////////////////////////////////////////////////////////////////////////////////////

CelestialObserver::CelestialObserver(std::string const& sCenterName, std::string const& sFrameName)
    : CelestialAnchor(sCenterName, sFrameName) {
}

////////////////////////////////////////////////////////////////////////////////////////////////////

struct CelestialObserver::timedVector {
  double time;
  glm::dvec3 vec;
} typedef timedVector;

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::updateMovementAnimation(double tTime) {
  if (mAnimationInProgress) {
    std::vector<tinyspline::real> pos = positionSpline.bisect(tTime).result();
    mPosition = glm::dvec3(pos[1], pos[2], pos[3]); //mAnimatedPosition.get(tTime);
    std::vector<tinyspline::real> lA = lookAtSpline.bisect(tTime).result();
    std::vector<tinyspline::real> up = upSpline.bisect(tTime).result();
    mRotation = glm::lookAt(mPosition, glm::dvec3(lA[1], lA[2], lA[3]), glm::dvec3(up[1], up[2], up[3]));//mAnimatedRotation.get(tTime);

    if (dRealEndTime < tTime) {
      mAnimationInProgress = false;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::setPosition(glm::dvec3 vPos) {
  if (!mAnimationInProgress) {
    CelestialAnchor::setPosition(std::move(vPos));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::setRotation(glm::dquat qRot) {
  if (!mAnimationInProgress) {
    CelestialAnchor::setRotation(std::move(qRot));
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::changeOrigin(
    std::string sCenterName, std::string sFrameName, double dSimulationTime) {

  mAnimationInProgress = false;

  cs::scene::CelestialAnchor target(sCenterName, sFrameName);

  glm::dvec3 pos = target.getRelativePosition(dSimulationTime, *this);
  glm::dquat rot = target.getRelativeRotation(dSimulationTime, *this);

  setCenterName(std::move(sCenterName));
  setFrameName(std::move(sFrameName));

  setRotation(rot);
  setPosition(pos);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::moveTo(std::string const& sCenterName, std::string const& sFrameName, 
    std::vector<timedVector> positionControl, std::vector<timedVector> lookAtControl,
    std::vector<timedVector> upControl, double dSimulationTime, double dRealStartTime) {
  mAnimationInProgress = false;

  dRealEndTime = std::max(std::max(positionControl.back().time, lookAtControl.back().time), 
    upControl.back().time);

  // Perform no animation at all if end time is not greater than start time.
  if (dRealStartTime >= dRealEndTime) {
    setCenterName(sCenterName);
    setFrameName(sFrameName);
    setRotation(glm::lookAt(positionControl.back().vec, lookAtControl.back().vec,
      upControl.back().vec));
    setPosition(positionControl.back().vec);

  } else {
    cs::scene::CelestialAnchor target(sCenterName, sFrameName);

    try {
      glm::dvec3 startPos = target.getRelativePosition(dSimulationTime, *this);
      glm::dquat startRot = target.getRelativeRotation(dSimulationTime, *this);
      glm::dvec3 startLookAt = (glm::mat4_cast(startRot) * glm::dvec4(0,0,-1,0)).xyz() + startPos;
      glm::dvec3 startUp = (glm::mat4_cast(startRot) * glm::dvec4(0,1,0,0)).xyz();

      setCenterName(sCenterName);
      setFrameName(sFrameName);

      /*double cosTheta = glm::dot(startRot, rotation);

      // If cosTheta < 0, the interpolation will take the long way around the sphere.
      // To fix this, one quat must be negated.
      if (cosTheta < 0.0) {
        startRot = -startRot;
      }*/

      setRotation(startRot);
      setPosition(startPos);

      ////// <------ spline erzeugen
      std::vector<tinyspline::real> positions;
      positions.push_back(dRealStartTime);
      positions.push_back(startPos.x);
      positions.push_back(startPos.y);
      positions.push_back(startPos.z);
      for(timedVector tv : positionControl) {
        positions.push_back(tv.time);
        positions.push_back(tv.vec.x);
        positions.push_back(tv.vec.y);
        positions.push_back(tv.vec.z);
      }
      positionSpline = tinyspline::BSpline::interpolateCatmullRom(positions, 4);

      std::vector<tinyspline::real> lookAts;
      lookAts.push_back(dRealStartTime);
      lookAts.push_back(startLookAt.x);
      lookAts.push_back(startLookAt.y);
      lookAts.push_back(startLookAt.z);
      for(timedVector tv : lookAtControl) {
        lookAts.push_back(tv.time);
        lookAts.push_back(tv.vec.x);
        lookAts.push_back(tv.vec.y);
        lookAts.push_back(tv.vec.z);
      }
      lookAtSpline = tinyspline::BSpline::interpolateCatmullRom(lookAts, 4);

      std::vector<tinyspline::real> ups;
      ups.push_back(dRealStartTime);
      ups.push_back(startUp.x);
      ups.push_back(startUp.y);
      ups.push_back(startUp.z);
      for(timedVector tv : upControl) {
        ups.push_back(tv.time);
        ups.push_back(tv.vec.x);
        ups.push_back(tv.vec.y);
        ups.push_back(tv.vec.z);
      }
      upSpline = tinyspline::BSpline::interpolateCatmullRom(ups, 4);


      /*
      mAnimatedPosition = utils::AnimatedValue<glm::dvec3>(
          startPos, position, dRealStartTime, dRealEndTime, utils::AnimationDirection::eInOut);

      mAnimatedRotation = utils::AnimatedValue<glm::dquat>(
          startRot, rotation, dRealStartTime, dRealEndTime, utils::AnimationDirection::eInOut);
      */

      mAnimationInProgress = true;
    } catch (std::exception const& e) {
      // Getting the relative transformation may fail due to insufficient SPICE data.
      logger().warn("CelestialObserver::moveTo failed: {}", e.what());
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::moveTo(std::string const& sCenterName, std::string const& sFrameName,
    glm::dvec3 const& position, glm::dquat const& rotation, double dSimulationTime,
    double dRealStartTime, double dRealEndTime) {
  std::vector<timedVector> pos;
  //timedVector tv = {dRealEndTime, position};
  pos.push_back(timedVector{dRealEndTime, position});
  std::vector<timedVector> lA;
  lA.push_back(timedVector{dRealEndTime, (glm::mat4_cast(rotation) * glm::dvec4(0,0,-1,0)).xyz() 
    + position});
  std::vector<timedVector> up;
  up.push_back(timedVector{dRealEndTime, (glm::mat4_cast(rotation) * glm::dvec4(0,1,0,0)).xyz()});
  moveTo(sCenterName, sFrameName, pos, lA, up, dSimulationTime, dRealStartTime);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool CelestialObserver::isAnimationInProgress() const {
  return mAnimationInProgress;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace cs::scene
