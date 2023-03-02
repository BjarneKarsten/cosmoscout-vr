////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

// SPDX-FileCopyrightText: German Aerospace Center (DLR) <cosmoscout@dlr.de>
// SPDX-License-Identifier: MIT

#include "CelestialObserver.hpp"
#include "CelestialObject.hpp"
#include "../cs-core/SolarSystem.hpp"
#include "logger.hpp"
#include <iostream>
#include <fstream>

namespace cs::scene {

////////////////////////////////////////////////////////////////////////////////////////////////////

CelestialObserver::CelestialObserver(std::string const& sCenterName, std::string const& sFrameName)
    : CelestialAnchor(sCenterName, sFrameName) {
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::updateMovementAnimation(double tTime, double speed) {
  if (animationType == spline) {
    //logger().info(tTime);
    std::ofstream f;
    std::ofstream f2;
    f.open("plot3d.json", std::ios::out | std::ios::app);
    f2.open("lookat.json", std::ios::out | std::ios::app);
    //f2 << "\t\t" << speed << ",\n";
    std::vector<tinyspline::real> pos = positionSpline.bisect(easeInOut(tTime - startTime, 0, endTime - startTime)).result();
    mPosition = glm::dvec3(pos[1], pos[2], pos[3]); //mAnimatedPosition.get(tTime);
    f << "\t\t[" << mPosition.x << ", " << mPosition.y << ", " << mPosition.z << "],\n";
    std::vector<tinyspline::real> lA = lookAtSpline.bisect(easeInOut(tTime - startTime, 0, endTime - startTime)).result();
    f2 << "\t\t[" << lA[1] << ", " << lA[2] << ", " << lA[3] << "],\n";
    std::vector<tinyspline::real> up = upSpline.bisect(easeInOut(tTime - startTime, 0, endTime - startTime)).result();
    glm::dvec3 z = -glm::normalize(glm::dvec3(lA[1], lA[2], lA[3]) - mPosition);
    glm::dvec3 x = -glm::normalize(glm::cross(z, glm::dvec3(up[1], up[2], up[3])));
    glm::dvec3 y = glm::normalize(glm::cross(z, x));
    mRotation = glm::quat_cast(glm::dmat3(x, y, z));

    if (endTime < tTime) {
      animationType = none;
      f  << "\t]\n}";
      f2 << "\t]\n}";
    }
    f.close();
    f2.close();
  }
  else if(animationType == animatedValue) {
    mPosition = mAnimatedPosition.get(tTime);
    mRotation = mAnimatedRotation.get(tTime);

    if (mAnimatedPosition.mEndTime < tTime) {
      animationType = none;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

double CelestialObserver::easeInOut(double tTime, double startTime, double endTime) {
  double t = (tTime - startTime) / (endTime - startTime);
  double progress;
  if(t < 0) progress = 0.0;
  else if(t < 0.5) progress = std::pow(1.74113 * t, 5);
  else if(t <= 1) progress = std::pow(1.74113 * t - 1.74113, 5) + 1;
  else progress = 1.0;
  return progress * (endTime - startTime);// + startTime;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::setPosition(glm::dvec3 vPos) {
  if (!animationType) {
    CelestialAnchor::setPosition(std::move(vPos));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::setRotation(glm::dquat qRot) {
  if (!animationType) {
    CelestialAnchor::setRotation(std::move(qRot));
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::changeOrigin(
    std::string sCenterName, std::string sFrameName, double dSimulationTime) {

  animationType = none;

  cs::scene::CelestialAnchor target(sCenterName, sFrameName);

  glm::dvec3 pos = target.getRelativePosition(dSimulationTime, *this);
  glm::dquat rot = target.getRelativeRotation(dSimulationTime, *this);

  setCenterName(std::move(sCenterName));
  setFrameName(std::move(sFrameName));

  setRotation(rot);
  setPosition(pos);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::moveToSpline(std::string const& sCenterName, std::string const& sFrameName, 
    std::vector<timedVector> positionControl, std::vector<timedVector> lookAtControl,
    std::vector<timedVector> upControl, double dSimulationTime, double dRealStartTime) {
  animationType = none;

  endTime = std::max(std::max(positionControl.back().time, lookAtControl.back().time), 
    upControl.back().time) + dRealStartTime;
  startTime = dRealStartTime;
  logger().info("endTime: {}", endTime);

  // Perform no animation at all if end time is not greater than start time.
  if (startTime >= endTime) {
    setCenterName(sCenterName);
    setFrameName(sFrameName);
    glm::dvec3 z = -glm::normalize(lookAtControl.back().vec - positionControl.back().vec);
    glm::dvec3 x = -glm::normalize(glm::cross(z, upControl.back().vec));
    glm::dvec3 y = glm::normalize(glm::cross(z, x));
    setRotation(glm::quat_cast(glm::mat3(x,y,z)));
    setPosition(positionControl.back().vec);

  } else {
    cs::scene::CelestialAnchor target(sCenterName, sFrameName);

    try {
      glm::dvec3 actPos = target.getRelativePosition(dSimulationTime, 
        cs::scene::CelestialObject(getCenterName(), getFrameName()));
      glm::dvec3 startPos = target.getRelativePosition(dSimulationTime, *this);
      glm::dquat startRot = target.getRelativeRotation(dSimulationTime, *this);
      glm::dvec3 startLookAt = (glm::mat4_cast(startRot) 
        * glm::dvec4(0,0,-glm::length(startPos - actPos) * 100,0)).xyz() + startPos;
      glm::dvec3 startUp = (glm::mat4_cast(startRot) * glm::dvec4(0,1,0,0)).xyz();
      std::ofstream f;
      f.open("lookat.json", std::ios::out | std::ios::app);
      f << "\t\"startLA\" : [" << startLookAt.x << ", " << startLookAt.y << ", " << startLookAt.z << "],\n"
        << "\t\"lookats\" : [\n";
      f.close();
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

      std::vector<tinyspline::real> positions;
      positions.push_back(-(endTime - startTime));
      positions.push_back(startPos.x);
      positions.push_back(startPos.y);
      positions.push_back(startPos.z);
      positions.push_back(0/*dRealStartTime*/);
      positions.push_back(startPos.x);
      positions.push_back(startPos.y);
      positions.push_back(startPos.z);
      for(timedVector tv : positionControl) {
        positions.push_back(tv.time);
        positions.push_back(tv.vec.x);
        positions.push_back(tv.vec.y);
        positions.push_back(tv.vec.z);
      }
      positions.push_back(2*(endTime - startTime));
      positions.push_back(positionControl.back().vec.x);
      positions.push_back(positionControl.back().vec.y);
      positions.push_back(positionControl.back().vec.z);
      positionSpline = tinyspline::BSpline::interpolateCubicNatural(positions, 4);

      std::vector<tinyspline::real> lookAts;
      lookAts.push_back(0/*dRealStartTime*/);
      lookAts.push_back(startLookAt.x);
      lookAts.push_back(startLookAt.y);
      lookAts.push_back(startLookAt.z);
      for(timedVector tv : lookAtControl) {
        lookAts.push_back(tv.time);
        //tv.vec = glm::normalize(tv.vec);
        lookAts.push_back(tv.vec.x);
        lookAts.push_back(tv.vec.y);
        lookAts.push_back(tv.vec.z);
      }
      lookAtSpline = tinyspline::BSpline::interpolateCatmullRom(lookAts, 4);

      std::vector<tinyspline::real> ups;
      ups.push_back(0/*dRealStartTime*/);
      ups.push_back(startUp.x);
      ups.push_back(startUp.y);
      ups.push_back(startUp.z);
      for(timedVector tv : upControl) {
        ups.push_back(tv.time);
        ups.push_back(tv.vec.x);
        ups.push_back(tv.vec.y);
        ups.push_back(tv.vec.z);
      }
      upSpline = tinyspline::BSpline::interpolateCubicNatural(ups, 4);

      animationType = spline;
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
  animationType = none;

  // Perform no animation at all if end time is not greater than start time.
  if (dRealStartTime >= dRealEndTime) {
    setCenterName(sCenterName);
    setFrameName(sFrameName);
    setRotation(rotation);
    setPosition(position);

  } else {
    cs::scene::CelestialAnchor target(sCenterName, sFrameName);

    try {
      glm::dvec3 startPos = target.getRelativePosition(dSimulationTime, *this);
      glm::dquat startRot = target.getRelativeRotation(dSimulationTime, *this);

      setCenterName(sCenterName);
      setFrameName(sFrameName);

      double cosTheta = glm::dot(startRot, rotation);

      // If cosTheta < 0, the interpolation will take the long way around the sphere.
      // To fix this, one quat must be negated.
      if (cosTheta < 0.0) {
        startRot = -startRot;
      }

      setRotation(startRot);
      setPosition(startPos);

      mAnimatedPosition = utils::AnimatedValue<glm::dvec3>(
          startPos, position, dRealStartTime, dRealEndTime, utils::AnimationDirection::eInOut);

      mAnimatedRotation = utils::AnimatedValue<glm::dquat>(
          startRot, rotation, dRealStartTime, dRealEndTime, utils::AnimationDirection::eInOut);

      animationType = animatedValue;
    } catch (std::exception const& e) {
      // Getting the relative transformation may fail due to insufficient SPICE data.
      logger().warn("CelestialObserver::moveTo failed: {}", e.what());
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool CelestialObserver::isAnimationInProgress() const {
  return animationType;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace cs::scene
