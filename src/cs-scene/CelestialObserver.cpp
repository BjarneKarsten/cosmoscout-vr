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

void CelestialObserver::updateMovementAnimation(double tTime) {
  if (mAnimationType == spline) {
    //logger().info(tTime);
    std::ofstream f;
    std::ofstream f2;
    f.open("plot3d.json", std::ios::out | std::ios::app);
    f2.open("lookat.json", std::ios::out | std::ios::app);
    //f2 << "\t\t" << speed << ",\n";
    std::vector<double> pos = mPositionSpline.bisect(easeInOut(tTime - mStartTime, mEndTime - mStartTime, 20)).result();
    mPosition = glm::dvec3(pos[1], pos[2], pos[3]); //mAnimatedPosition.get(tTime);
    f << "\t\t[" << mPosition.x << ", " << mPosition.y << ", " << mPosition.z << "],\n";
    std::vector<double> lA = mLookAtSpline.bisect(easeInOut(tTime - mStartTime, mEndTime - mStartTime, 10)).result();
    f2 << "\t\t[" << lA[1] << ", " << lA[2] << ", " << lA[3] << "],\n";
    std::vector<double> up = mUpSpline.bisect(easeInOut(tTime - mStartTime, mEndTime - mStartTime, 5)).result();
    glm::dvec3 z = -glm::normalize(glm::dvec3(lA[1], lA[2], lA[3]) - mPosition);
    glm::dvec3 x = -glm::normalize(glm::cross(z, glm::dvec3(up[1], up[2], up[3])));
    glm::dvec3 y = glm::normalize(glm::cross(z, x));
    mRotation = glm::quat_cast(glm::dmat3(x, y, z));

    if (mEndTime < tTime) {
      mAnimationType = none;
      f  << "\t]\n}";
      f2 << "\t]\n}";
    }
    f.close();
    f2.close();
  }
  else if(mAnimationType == animatedValue) {
    mPosition = mAnimatedPosition.get(tTime);
    mRotation = mAnimatedRotation.get(tTime);

    if (mAnimatedPosition.mEndTime < tTime) {
      mAnimationType = none;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

double CelestialObserver::easeInOut(double time, double duration, int steepness) {
  double a = steepness * 0.2;
  double t = time / duration;
  double progress;
  if(t < 0.0) progress = 0.0;
  else if(t < 0.5) progress = std::pow(2, a) * std::pow(t, a + 1);
  else if(t <= 1.0) progress = std::pow(-2, a) * std::pow(t-1, a + 1) + 1.0;
  else progress = 1.0;
  return progress * duration;// + mStartTime;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::setPosition(glm::dvec3 vPos) {
  if (!mAnimationType) {
    CelestialAnchor::setPosition(std::move(vPos));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::setRotation(glm::dquat qRot) {
  if (!mAnimationType) {
    CelestialAnchor::setRotation(std::move(qRot));
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::changeOrigin(
    std::string sCenterName, std::string sFrameName, double dSimulationTime) {

  mAnimationType = none;

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
  mAnimationType = none;

  mEndTime = std::max(std::max(positionControl.back().time, lookAtControl.back().time), 
    upControl.back().time) + dRealStartTime;
  mStartTime = dRealStartTime;

  // Perform no animation at all if end time is not greater than start time.
  if (mStartTime >= mEndTime) {
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
      cs::scene::CelestialObject origin("Solar System Barycenter", "J2000");
      glm::dvec3 actPos = origin.getRelativePosition(dSimulationTime, 
        cs::scene::CelestialObject(getCenterName(), getFrameName()));
      glm::dvec3 startPos = origin.getRelativePosition(dSimulationTime, *this);
      glm::dquat startRot = origin.getRelativeRotation(dSimulationTime, *this);
      glm::dvec3 startLookAt = startPos + startRot * glm::dvec3(0,0,-glm::length(startPos - actPos)
        * 100);
      glm::dvec3 startUp = startRot * glm::dvec3(0,1,0);
      std::ofstream f;
      f.open("lookat.json", std::ios::out | std::ios::app);
      f << "\t\"startLA\" : [" << startLookAt.x << ", " << startLookAt.y << ", " << startLookAt.z << "],\n"
        << "\t\"lookats\" : [\n";
      f.close();
      setCenterName("Solar System Barycenter");
      setFrameName("J2000");

      /*double cosTheta = glm::dot(startRot, rotation);

      // If cosTheta < 0, the interpolation will take the long way around the sphere.
      // To fix this, one quat must be negated.
      if (cosTheta < 0.0) {
        startRot = -startRot;
      }*/

      setRotation(startRot);
      setPosition(startPos);

      std::vector<double> positions;
      /*positions.push_back(-0.5 * (mEndTime - mStartTime));
      positions.push_back(actPos.x);
      positions.push_back(actPos.y);
      positions.push_back(actPos.z);*/
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
      mPositionSpline = tinyspline::BSpline::interpolateCubicNatural(positions, 4);

      std::vector<double> lookAts;
      lookAts.push_back(0/*dRealStartTime*/);
      lookAts.push_back(startLookAt.x);
      lookAts.push_back(startLookAt.y);
      lookAts.push_back(startLookAt.z);
      for(timedVector tv : lookAtControl) {
        lookAts.push_back(tv.time);
        lookAts.push_back(tv.vec.x);
        lookAts.push_back(tv.vec.y);
        lookAts.push_back(tv.vec.z);
      }
      mLookAtSpline = tinyspline::BSpline::interpolateCatmullRom(lookAts, 4);

      std::vector<double> ups;
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
      mUpSpline = tinyspline::BSpline::interpolateCubicNatural(ups, 4);

      mAnimationType = spline;
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
  mAnimationType = none;

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

      mAnimationType = animatedValue;
    } catch (std::exception const& e) {
      // Getting the relative transformation may fail due to insufficient SPICE data.
      logger().warn("CelestialObserver::moveTo failed: {}", e.what());
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool CelestialObserver::isBehind(cs::scene::CelestialObject frame, 
    cs::scene::CelestialObject object, double dSimulationTime, double allowedRadii) {
  glm::dvec3 objPos = frame.getRelativePosition(dSimulationTime, object);
  glm::dvec3 obsPos = frame.getRelativePosition(dSimulationTime, *this);
  if(glm::length(obsPos) < glm::length(objPos)) return false;
  // arbitrary vector orthogonal to objPos:
  glm::dvec3 p = glm::cross(objPos, glm::dvec3(1,0,0));
  // p is null vector if objPos is parallel to (1,0,0), use (0,1,0) instead 
  if(p.x == 0.0 && p.y == 0.0 && p.z == 0.0) {
    p = glm::cross(objPos, glm::dvec3(0,1,0));
  }
  p = objPos + glm::normalize(p) * allowedRadii * object.getRadii()[0];
  double alpha = std::acos(std::abs(glm::dot(glm::normalize(obsPos), glm::normalize(objPos))));
  double beta = std::acos(std::abs(glm::dot(glm::normalize(p), glm::normalize(objPos))));
  return alpha <= beta; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool CelestialObserver::isAnimationInProgress() const {
  return mAnimationType;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace cs::scene
