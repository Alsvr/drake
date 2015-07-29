/*
 * FixedJoint.h
 *
 *  Created on: Mar 26, 2015
 *      Author: twan
 */

#ifndef DRAKE_SYSTEMS_PLANTS_JOINTS_FIXEDJOINT_H_
#define DRAKE_SYSTEMS_PLANTS_JOINTS_FIXEDJOINT_H_

#include "DrakeJointImpl.h"
//#include "../../controllers/controlUtil.h"

class DLLEXPORT_DRAKEJOINT FixedJoint: public DrakeJointImpl<FixedJoint> {
public:
  FixedJoint(const std::string &name, const Eigen::Isometry3d &transform_to_parent_body)
          : DrakeJointImpl(*this, name, transform_to_parent_body, 0, 0) { };

  virtual ~FixedJoint() { };


//  virtual Eigen::Transform<Scalar, 3, Eigen::Isometry> jointTransform(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q) const Specifier; \
//  virtual GradientVar<Scalar, 6, Eigen::Dynamic> motionSubspace(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q) const Specifier; \
//  virtual GradientVar<Scalar, 6, 1> motionSubspaceDotTimesV(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q, const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& v, int gradient_order) const Specifier; \
//  virtual GradientVar<Scalar, Eigen::Dynamic, Eigen::Dynamic> qdot2v(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q, int gradient_order) const Specifier; \
//  virtual GradientVar<Scalar, Eigen::Dynamic, Eigen::Dynamic> v2qdot(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q, int gradient_order) const Specifier; \
//  virtual GradientVar<Scalar, Eigen::Dynamic, 1> frictionTorque(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& v, int gradient_order) const Specifier;
//

  template<typename Scalar>
  Eigen::Transform<Scalar, 3, Eigen::Isometry> jointTransform(
          const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> > &q) const {
    return Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();
  };

  template<typename Scalar>
  void motionSubspace(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> > &q,
                      MotionSubspaceType &motion_subspace,
                      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> *dmotion_subspace = nullptr) const {
    motion_subspace.resize(TWIST_SIZE, getNumVelocities());
    if (dmotion_subspace) {
      dmotion_subspace->resize(motion_subspace.size(), getNumPositions());
    }
  };

  template<typename Scalar>
  void motionSubspaceDotTimesV(const Eigen::Ref<const Eigen::VectorXd> &q, const Eigen::Ref<const Eigen::VectorXd> &v,
                               Eigen::Matrix<Scalar, 6, 1> &motion_subspace_dot_times_v,
                               Gradient<Eigen::Matrix<Scalar, 6, 1>, Eigen::Dynamic>::type *dmotion_subspace_dot_times_vdq = nullptr,
                               Gradient<Eigen::Matrix<Scalar, 6, 1>, Eigen::Dynamic>::type *dmotion_subspace_dot_times_vdv = nullptr) const {
    motion_subspace_dot_times_v.setZero();

    if (dmotion_subspace_dot_times_vdq) {
      dmotion_subspace_dot_times_vdq->setZero(TWIST_SIZE, 1);
    }

    if (dmotion_subspace_dot_times_vdv) {
      dmotion_subspace_dot_times_vdv->setZero(TWIST_SIZE, 1);
    }
  };

  template<typename Scalar>
  void qdot2v(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> > &q,
              Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &qdot_to_v,
              Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> *dqdot_to_v) const {
    qdot_to_v.resize(getNumVelocities(), getNumPositions());
    if (dqdot_to_v) {
      dqdot_to_v->setZero(qdot_to_v.size(), getNumPositions());
    }
  };

  template<typename Scalar>
  void v2qdot(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> &q,
              Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &v_to_qdot,
              Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> *dv_to_qdot) const {
    v_to_qdot.resize(getNumPositions(), getNumVelocities());
    if (dv_to_qdot) {
      dv_to_qdot->setZero(v_to_qdot.size(), getNumPositions());
    }
  };
};

#endif /* DRAKE_SYSTEMS_PLANTS_JOINTS_FIXEDJOINT_H_ */
