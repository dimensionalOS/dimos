// This is an advanced implementation of the algorithm described in the
// following paper:
//    C. Hertzberg,  R.  Wagner,  U.  Frese,  and  L.  Schroder.  Integratinggeneric   sensor   fusion   algorithms   with   sound   state   representationsthrough  encapsulation  of  manifolds.
//    CoRR,  vol.  abs/1107.1119,  2011.[Online]. Available: http://arxiv.org/abs/1107.1119

/*
 *  Copyright (c) 2019--2023, The University of Hong Kong
 *  All rights reserved.
 *
 *  Modifier: Dongjiao HE <hdj65822@connect.hku.hk>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  Copyright (c) 2008--2011, Universitaet Bremen
 *  All rights reserved.
 *
 *  Author: Christoph Hertzberg <chtz@informatik.uni-bremen.de>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file mtk/types/SEn.hpp
 * @brief Standard Orthogonal Groups i.e.\ rotatation groups.
 */
#ifndef SEN_H_
#define SEN_H_

#include <Eigen/Geometry>

#include "SOn.hpp"
#include "vect.hpp"
#include "../src/mtkmath.hpp"


namespace MTK {


/**
 * Three-dimensional orientations represented as Quaternion.
 * It is assumed that the internal Quaternion always stays normalized,
 * should this not be the case, call inherited member function @c normalize().
 */
template<class _scalar = double, int num_of_vec_plus1 = 6, int dim_of_mat = 4, int Options = Eigen::AutoAlign>
struct SEN {
	enum {DOF = num_of_vec_plus1, DIM = num_of_vec_plus1, TYP = 4};
	typedef _scalar scalar;
	typedef Eigen::Matrix<scalar, dim_of_mat, dim_of_mat> base;
	typedef SO3<scalar> SO3_type;
	// typedef Eigen::Quaternion<scalar, Options> base;
	// typedef Eigen::Quaternion<scalar> Quaternion;
	typedef vect<DIM, scalar, Options> vect_type;
	SO3_type SO3_data;
	base mat;

	/**
	 * Construct from real part and three imaginary parts.
	 * Quaternion is normalized after construction.
	 */
	// SEN(const base& src) : mat(src) {
	// 	// base::normalize();
	// }

	/**
	 * Construct from Eigen::Quaternion.
	 * @note Non-normalized input may result result in spurious behavior.
	 */
	SEN(const base& src = base::Identity()) : mat(src) {}

	/**
	 * Construct from rotation matrix.
	 * @note Invalid rotation matrices may lead to spurious behavior.
	 */
	// template<class Derived>
	// SO3(const Eigen::MatrixBase<Derived>& matrix) : base(matrix) {}

	/**
	 * Construct from arbitrary rotation type.
	 * @note Invalid rotation matrices may lead to spurious behavior.
	 */
	// template<class Derived>
	// SO3(const Eigen::RotationBase<Derived, 3>& rotation) : base(rotation.derived()) {}

	//! @name Manifold requirements

	void boxplus(MTK::vectview<const scalar, DOF> vec, scalar scale=1) {
		SEN delta = exp(vec, scale); // ?
		mat = mat * delta.mat;
	}
	void boxminus(MTK::vectview<scalar, DOF> res, const SEN<scalar,num_of_vec_plus1,dim_of_mat, Options>& other) const {
		base error_mat = other.mat.inverse() * mat;
		res = log(error_mat);
	}
	//}

	void oplus(MTK::vectview<const scalar, DOF> vec, scalar scale=1) {
		SEN delta = exp(vec, scale);
		mat = mat * delta.mat;
	}

	// void hat(MTK::vectview<const scalar, DOF>& v, Eigen::Matrix<scalar, dim_of_mat, dim_of_mat> &res) {
	void hat(Eigen::VectorXd& v, Eigen::MatrixXd &res) {
		res = Eigen::Matrix<scalar, dim_of_mat, dim_of_mat>::Zero();
		Eigen::Matrix<scalar, 3, 3> psi;
		psi << 0, -v[2], v[1],
			v[2], 0, -v[0],
			-v[1], v[0], 0;
		res.block<3, 3>(0, 0) = psi;
		for(int i = 3; i < v.size() / 3 + 2; i++)
		{
			res.block<3, 1>(0, i) = v.segment<3>(i + (i-3)*3);
		}
		// return res;
	}

	// void Jacob_right_inv(MTK::vectview<const scalar, DOF> vec, Eigen::Matrix<scalar, dim_of_mat, dim_of_mat> & res){
	void Jacob_right_inv(Eigen::VectorXd& vec, Eigen::MatrixXd &res){
		res = Eigen::Matrix<scalar, dim_of_mat, dim_of_mat>::Zero();
    	Eigen::Matrix<scalar, 3, 3> M_v;
    	Eigen::VectorXd vec_psi, vec_ro;
    	Eigen::MatrixXd jac_v;
		Eigen::MatrixXd hat_v, hat_ro;
		vec_psi = vec.segment<3>(0);
		// Eigen::Matrix<scalar, 3, 1> ;
		SO3_data.hat(vec_psi, hat_v);
		SO3_data.Jacob_right_inv(vec_psi, jac_v);
		double norm = vec_psi.norm();
		for(int i = 0; i < vec.size() / 3; i++)
		{
			res.block<3, 3>(i*3, i*3) = jac_v;
		}
		for(int i = 1; i < vec.size() / 3; i++)
		{
			vec_ro = vec.segment<3>(i * 3);
			SO3_data.hat(vec_ro, hat_ro);
			if(norm > MTK::tolerance<scalar>())
			{
				res.block<3,3>(i*3, 0) = 0.5 * hat_ro + (1 - norm * std::cos(norm / 2) / 2 / std::sin(norm / 2))/norm/norm * (hat_ro * hat_v + hat_v * hat_ro) + ((2 - norm * std::cos(norm / 2) / 2 / std::sin(norm / 2)) / 2 / norm / norm / norm / norm - 1 / 8 / norm / norm / std::sin(norm / 2) / std::sin(norm / 2)) * hat_v * (hat_ro * hat_v + hat_v * hat_ro) * hat_v;
			}
			else
			{
				res.block<3,3>(i*3, 0) = 0.5 * hat_ro;
			}

		}
    	// return res;
	}

	// void Jacob_right(MTK::vectview<const scalar, DOF> & vec, Eigen::Matrix<scalar, dim_of_mat, dim_of_mat> &res){
	void Jacob_right(Eigen::VectorXd& vec, Eigen::MatrixXd &res){
		res = Eigen::Matrix<scalar, dim_of_mat, dim_of_mat>::Zero();
		Eigen::MatrixXd hat_v, hat_ro;
    	Eigen::VectorXd vec_psi, vec_ro;
		Eigen::MatrixXd jac_v;
		vec_psi = vec.segment<3>(0);
		// Eigen::Matrix<scalar, 3, 1> ;
		SO3_data.hat(vec_psi, hat_v);
		SO3_data.Jacob_right(vec_psi, jac_v);
		// double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
		// double norm = std::sqrt(squaredNorm);
		double norm = vec_psi.norm();
		for(int i = 0; i < vec.size() / 3; i++)
		{
			res.block<3, 3>(i*3, i*3) = jac_v;
		}
		for(int i = 1; i < vec.size() / 3; i++)
		{
			vec_ro = vec.segment<3>(i * 3);
			SO3_data.hat(vec_ro, hat_ro);
			if(norm > MTK::tolerance<scalar>())
			{
				res.block<3,3>(i*3, 0) = -1 * jac_v * (0.5 * hat_ro + (1 - norm * std::cos(norm / 2) / 2 / std::sin(norm / 2))/norm/norm * (hat_ro * hat_v + hat_v * hat_ro) + ((2 - norm * std::cos(norm / 2) / 2 / std::sin(norm / 2)) / 2 / norm / norm / norm / norm - 1 / 8 / norm / norm / std::sin(norm / 2) / std::sin(norm / 2)) * hat_v * (hat_ro * hat_v + hat_v * hat_ro) * hat_v) * jac_v;
			}
			else
			{
				res.block<3,3>(i*3, 0) = -0.5 * jac_v * hat_ro * jac_v;
			}

		}
		// return res;
	}

	void S2_hat(Eigen::Matrix<scalar, 3, 3> &res)
	{
		std::cerr << "wrong idx for S2" << std::endl;
		res = Eigen::Matrix<scalar, 3, 3>::Zero();
	}
	void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3> &res)
	{
		std::cerr << "wrong idx for S2" << std::endl;
		std::exit(100);
    	res = Eigen::Matrix<scalar, 2, 3>::Zero();
	}

	void S2_Mx(Eigen::Matrix<scalar, 3, 2> &res, MTK::vectview<const scalar, 2> delta)
	{
		std::cerr << "wrong idx for S2" << std::endl;
		std::exit(100);
    	res = Eigen::Matrix<scalar, 3, 2>::Zero();
	}

	friend std::ostream& operator<<(std::ostream &os, const SEN<scalar, DOF, dim_of_mat, Options>& q){
		for(int i=0; i<dim_of_mat; i++)
		{
			for(int j = 0; j < dim_of_mat; j++)
			{
				os << q.mat(i, j) << " ";
			}
		}
		return os;
	}

	friend std::istream& operator>>(std::istream &is, SEN<scalar, DOF, dim_of_mat, Options>& q){
		// vect<dim_of_mat * dim_of_mat,scalar> coeffs;
		for(int i=0; i<dim_of_mat; i++)
		{
			for(int j = 0; j < dim_of_mat; j++)
			{
				is >> q.mat(i, j);
			}
		}
		// is >> q.mat;
		// coeffs;
		// q.coeffs() = coeffs.normalized();
		return is;
	}

	//! @name Helper functions
	//{
	/**
	 * Calculate the exponential map. In matrix terms this would correspond
	 * to the Rodrigues formula.
	 */
	// FIXME vectview<> can't be constructed from every MatrixBase<>, use const Vector3x& as workaround
//	static SO3 exp(MTK::vectview<const scalar, 3> dvec, scalar scale = 1){
	static SEN exp(const Eigen::Matrix<scalar, DOF, 1>& dvec, scalar scale = 1){
		SEN res;
		res.mat = Eigen::Matrix<scalar, dim_of_mat, dim_of_mat>::Identity();
		Eigen::Matrix<scalar, 3, 3> exp_; //, jac;
		Eigen::MatrixXd jac;
		Eigen::Matrix<scalar, 3, 1> psi;
		Eigen::VectorXd minus_psi;
		psi = dvec.template block<3,1>(0, 0);
		minus_psi = -psi;
		SO3_type SO3_temp;
		exp_ = SO3_type::exp(psi);
		SO3_temp.Jacob_right(minus_psi, jac);
		res.mat.template block<3,3>(0, 0) = exp_;
		for(int i = 3; i < DOF / 3 + 2; i++)
		{
			res.mat.template block<3, 1>(0, i) = jac * dvec.template block<3,1>(i + (i-3)*3,0);
		}
		return res;
	}
	/**
	 * Calculate the inverse of @c exp.
	 * Only guarantees that <code>exp(log(x)) == x </code>
	 */
	static Eigen::Matrix<scalar, DOF, 1> log(base &orient){
		Eigen::Matrix<scalar, DOF, 1> res;
		Eigen::Matrix<scalar, 3, 1> psi;
		Eigen::VectorXd minus_psi;
		Eigen::Matrix<scalar, 3, 3> mat_psi;
		Eigen::MatrixXd jac;
		mat_psi = orient.template block<3, 3>(0, 0);
		SO3_type SO3_temp;
		SO3_type exp_psi(mat_psi);
		psi = SO3_type::log(exp_psi);
		minus_psi = -psi;
		SO3_temp.Jacob_right_inv(minus_psi, jac);
		for(int i = 3; i < dim_of_mat; i++)
		{
			res.template block<3,1>(i + (i-3)*3,0) = jac * orient.template block<3, 1>(0, i);
		}
		return res;
	}
};


}  // namespace MTK

#endif /*SON_H_*/
