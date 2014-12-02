/*
 * Copyright (c) 2013-2014, Dario Lodi Rizzini, Riccardo Monica
 *   RIMLab, Department of Information Engineering, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "RayTracer.h"
#include <cassert>
#include <algorithm>    // std::copy_if
#include <iterator>     // std::back_insert_iterator

// http://ray-tracer-concept.blogspot.it/2011/11/ray-box-intersection.html
bool intersectRayBox(const Eigen::Vector3f& rayo,const Eigen::Vector3f& rayd,
                     const Eigen::Vector3f& bmin,const Eigen::Vector3f& bmax,
                     Eigen::Vector3f& rayi)
{
  Eigen::Vector3f bnear;
  Eigen::Vector3f bfar;
  // Checks for intersection testing on each direction coordinate
  // Computes 
  float t1, t2, tnear = -1e+6f, tfar = 1e+6f; //, tCube;
  bool intersectFlag = true;
  for (int i = 0; i < 3; ++i) {
//    std::cout << "coordinate " << i << ": bmin " << bmin(i) << ", bmax " << bmax(i) << std::endl; 
    assert(bmin(i) <= bmax(i));
    if (::fabs(rayd(i)) < 1e-6) {   // Ray parallel to axis i-th
      if (rayo(i) < bmin(i) || rayo(i) > bmax(i)) {
        intersectFlag = false;
      }
    }
    else {
      // Finds the nearest and the farthest vertices of the box from the ray origin
      if (::fabs(bmin(i) - rayo(i)) < ::fabs(bmax(i) - rayo(i))) {
        bnear(i) = bmin(i);
        bfar(i) = bmax(i);
      }
      else {
        bnear(i) = bmax(i);
        bfar(i) = bmin(i);
      }
//      std::cout << "  bnear " << bnear(i) << ", bfar " << bfar(i) << std::endl;
      // Finds the distance parameters t1 and t2 of the two ray-box intersections:
      // t1 must be the closest to the ray origin rayo. 
      t1 = (bnear(i) - rayo(i)) / rayd(i);
      t2 = (bfar(i) - rayo(i)) / rayd(i);
      if (t1 > t2) {
        std::swap(t1,t2);
      } 
      // The two intersection values are used to saturate tnear and tfar
      if (t1 > tnear) {
        tnear = t1;
      }
      if (t2 < tfar) {
        tfar = t2;
      }
//      std::cout << "  t1 " << t1 << ", t2 " << t2 << ", tnear " << tnear << ", tfar " << tfar 
//        << "  tnear > tfar? " << (tnear > tfar) << ", tfar < 0? " << (tfar < 0) << std::endl;
      if(tnear > tfar) {
         intersectFlag = false;
      }
      if(tfar < 0) {
       intersectFlag = false;
      }
    }
  }
  if (intersectFlag == true) {
    rayi = rayo + rayd * tnear;
//    std::cout << " intersection rayi:\n" << rayi << std::endl;
  }
  // Checks whether intersection occurs or not
  return intersectFlag;
}

bool targetViewpoint(const Eigen::Vector3f& rayo,const Eigen::Vector3f& target,const Eigen::Vector3f& down,
                     Eigen::Affine3f& transf)
{
  // uz: versor pointing toward the destination
  Eigen::Vector3f uz = target - rayo;
  if (std::abs(uz.norm()) < 1e-3) {
    std::cout << __FILE__ << "," << __LINE__ << ": target point on ray origin!" << std::endl;
    return false;
  }
  uz.normalize();
  //std::cout << "uz " << uz.transpose() << ", norm " << uz .norm() << std::endl;
  // ux: versor pointing toward the ground
  Eigen::Vector3f ux = down - down.dot(uz) * uz;  
  if (std::abs(ux.norm()) < 1e-3) {
    std::cout << __FILE__ << "," << __LINE__ << ": ray to target toward ground direction!" << std::endl;
    return false;
  }
  ux.normalize();
  //std::cout << "ux " << ux.transpose() << ", norm " << ux.norm() << std::endl;
  Eigen::Vector3f uy = uz.cross(ux);
  //std::cout << "uy " << uy.transpose() << ", norm " << uy.norm() << std::endl;
  Eigen::Matrix3f rot;
  rot << ux.x(), uy.x(), uz.x(),
         ux.y(), uy.y(), uz.y(),
         ux.z(), uy.z(), uz.z();
  transf.setIdentity();
  transf.translate(rayo);
  transf.rotate(rot);
  //std::cout << __FILE__ << "\nrotation\n" << rot << "\ntranslation\n" << rayo << "\naffine\n" << transf.matrix() << std::endl;
  return true;
}

// --------------------------------------------------------
// RAY TRACING
// --------------------------------------------------------

PinholeRayTracer::PinholeRayTracer()
 : viewpoint_(), hfov_(HFOV), vfov_(VFOV), hsize_(HSIZE), vsize_(VSIZE), res_(RES), rangeMax_(RANGE_MAX)
{
  viewpoint_ = Eigen::Affine3f::Identity();
  //std::cout << "************************\nviewpoint\n" << viewpoint_.matrix() << std::endl;
}

PinholeRayTracer::~PinholeRayTracer()
{
}

void PinholeRayTracer::setViewpoint(const Eigen::Affine3f& vp)
{
  viewpoint_ = vp;
  //std::cout << "************************\nCHANGED viewpoint\n" << viewpoint_.matrix() << std::endl;
}

Eigen::Affine3f PinholeRayTracer::getViewPoint() const
{
  return viewpoint_;
}

void PinholeRayTracer::setHalfFoV(float hfov,float vfov)
{
  hfov_ = hfov;
  vfov_ = vfov;
}

float PinholeRayTracer::getHFoV() const
{
  return hfov_;
}

float PinholeRayTracer::getVFoV() const
{
  return vfov_;
}

void PinholeRayTracer::setImageSize(int hsize,float vsize)
{
  hsize_ = hsize;
  vsize_ = vsize;
}

int PinholeRayTracer::getHSize() const
{
  return hsize_;
}

int PinholeRayTracer::getVSize() const
{
  return vsize_;
}


void PinholeRayTracer::setBoxRes(float res)
{
  res_ = res;
}

float PinholeRayTracer::getBoxRes() const
{
  return res_;
}

void PinholeRayTracer::setRangeMax(float rmax)
{
  rangeMax_ = rmax;
}

float PinholeRayTracer::getRangeMax() const
{
  return rangeMax_;
}

void PinholeRayTracer::castRays(std::vector<Eigen::Vector3f>& rayds)
{
  // Approximation: the field of view is sampled using equal division of tangent.
  Eigen::Vector3f rayd;
  float dh = tan(hfov_) / hsize_;
  float dv = tan(vfov_) / vsize_;
  Eigen::Matrix3f rot = viewpoint_.rotation();
  for (int h =-hsize_/2; h <= hsize_/2; ++h) {
    for (int v = -vsize_/2; v <= vsize_/2; ++v) {
      rayd << dv * v, dh * h, 1.0;
      if (h==0 && v ==0) {
        //std::cout << "CENTRAL RAY\n" << rayd.transpose() << std::endl;
      }
      rayd = rot * rayd;
      rayds.push_back(rayd);
      if (h==0 && v ==0) {
        //std::cout << "ROTATION\n" << rot.matrix() << std::endl;
        //std::cout << "h " << h << ", v " << v << ", rayd " << rayd.transpose() << std::endl;
      }
      //std::cout << "h " << h << ", v " << v << ", rayd " << rayd.transpose() << std::endl;
    }
  }
}

std::ostream& operator<<(std::ostream& out,const PinholeRayTracer& prt)
{
  out << "viewpoint:\n" << prt.viewpoint_.matrix() << std::endl
      << "hfov: " << prt.hfov_ << " rad" << std::endl
      << "vfox: " << prt.vfov_ << " rad" << std::endl
      << "hsize: " << prt.hsize_ << std::endl
      << "vsize: " << prt.vsize_ << std::endl
      << "res: " << prt.res_ << std::endl;
  return out;
}

// --------------------------------------------------------
// KINEMATIC FILTER
// --------------------------------------------------------


// IMPLEMENTATION OF NoKinematicFilter

bool NoKinematicFilter::isFeasible(const Eigen::Affine3f& transform)
{
  return true;
}

void NoKinematicFilter::selectFeasible(const std::vector<Eigen::Affine3f>& candidates,std::vector<Eigen::Affine3f>& feasibles)
{
  feasibles.clear();
  //std::copy_if(candidates.begin(),candidates.end(),std::back_inserter(feasibles),&ComauKinematicFilter::isFeasible);
  std::vector<Eigen::Affine3f>::const_iterator it;
  for (it = candidates.begin(); it != candidates.end(); ++it) {
    feasibles.push_back(*it);
  }
}



