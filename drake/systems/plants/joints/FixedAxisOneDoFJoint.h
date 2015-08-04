#ifndef ONEDOFJOINT_H_
#define ONEDOFJOINT_H_

#include "DrakeJointImpl.h"
#include <cmath>
#include <Eigen/Core>
#include <limits>
#include <exception>
#include <stdexcept>
#include "drakeFloatingPointUtil.h"
#include "drakeGradientUtil.h"

template <typename Derived>
class DLLEXPORT_DRAKEJOINT FixedAxisOneDoFJoint : public DrakeJointImpl<Derived>
{
  // disable copy construction and assignment
  // not available in MSVC2010...
  // FixedAxisOneDoFJoint(const DrakeJoint&) = delete;
  // FixedAxisOneDoFJoint& operator=(const FixedAxisOneDoFJoint&) = delete;

private:
  Eigen::Matrix<double, TWIST_SIZE, 1> joint_axis;
  double damping;
  double coulomb_friction;
  double coulomb_window;

protected:
  FixedAxisOneDoFJoint(Derived& derived, const std::string& name, const Eigen::Isometry3d& transform_to_parent_body, const Eigen::Matrix<double, TWIST_SIZE, 1>& joint_axis) :
      DrakeJointImpl<Derived>(derived, name, transform_to_parent_body, 1, 1),
      joint_axis(joint_axis),
      damping(0.0),
      coulomb_friction(0.0),
      coulomb_window(0.0) { };

public:

  using DrakeJoint::getNumPositions;
  using DrakeJoint::getNumVelocities;

  template <typename DerivedQ, typename DerivedMS>
  void motionSubspace(const Eigen::MatrixBase<DerivedQ> & q,
                      Eigen::MatrixBase<DerivedMS>& motion_subspace,
                      typename Gradient<DerivedMS, Eigen::Dynamic>::type* dmotion_subspace = nullptr) const {
    motion_subspace = joint_axis;
    if (dmotion_subspace) {
      dmotion_subspace->setZero(motion_subspace.size(), getNumPositions());
    }
  };

  template<typename DerivedQ, typename DerivedV>
  void motionSubspaceDotTimesV(const Eigen::MatrixBase<DerivedQ> &q, const Eigen::MatrixBase<DerivedV> &v,
                               Eigen::Matrix<typename DerivedQ::Scalar, 6, 1> &motion_subspace_dot_times_v,
                               typename Gradient<Eigen::Matrix<typename DerivedQ::Scalar, 6, 1>, Eigen::Dynamic>::type *dmotion_subspace_dot_times_vdq = nullptr,
                               typename Gradient<Eigen::Matrix<typename DerivedQ::Scalar, 6, 1>, Eigen::Dynamic>::type *dmotion_subspace_dot_times_vdv = nullptr) const {
    motion_subspace_dot_times_v.setZero();

    if (dmotion_subspace_dot_times_vdq) {
      dmotion_subspace_dot_times_vdq->setZero(TWIST_SIZE, 1);
    }

    if (dmotion_subspace_dot_times_vdv) {
      dmotion_subspace_dot_times_vdv->setZero(TWIST_SIZE, 1);
    }
  };

  template<typename DerivedQ>
  void qdot2v(const Eigen::MatrixBase<DerivedQ> & q,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic> &qdot_to_v,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic> *dqdot_to_v) const {
    qdot_to_v.setIdentity(getNumVelocities(), getNumPositions());
    if (dqdot_to_v) {
      dqdot_to_v->setZero(qdot_to_v.size(), getNumPositions());
    }
  };

  template<typename DerivedQ>
  void v2qdot(const Eigen::MatrixBase<DerivedQ> & q,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic> &v_to_qdot,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic> *dv_to_qdot) const {
    v_to_qdot.setIdentity(getNumPositions(), getNumVelocities());
    if (dv_to_qdot) {
      dv_to_qdot->setZero(v_to_qdot.size(), getNumPositions());
    }
  };

  GradientVar<double, Eigen::Dynamic, 1> frictionTorque(const Eigen::Ref<const Eigen::VectorXd>& v, int gradient_order) const {
    GradientVar<double, Eigen::Dynamic, 1> ret(getNumVelocities(), 1, getNumVelocities(), gradient_order);
    ret.value()[0] = damping * v[0];
    ret.value()[0] += std::min(1.0, std::max(-1.0, v[0] / coulomb_window)) * coulomb_friction;
    if (gradient_order > 0) {
      ret.gradient().value()(0, 0) = damping;
      if (std::abs(v[0]) < coulomb_window)
        ret.gradient().value()(0, 0) += sign(v[0]) * (coulomb_friction / coulomb_window);
    }
    return ret;
  }

  virtual ~FixedAxisOneDoFJoint() { } ;
  void setDynamics(double damping, double coulomb_friction, double coulomb_window);
  void setJointLimits(double joint_limit_min, double joint_limit_max);
  virtual Eigen::VectorXd randomConfiguration(std::default_random_engine& generator) const; //override;
  virtual std::string getPositionName(int index) const { if (index!=0) throw std::runtime_error("bad index"); return DrakeJoint::name; }

};

#endif /* ONEDOFJOINT_H_ */
