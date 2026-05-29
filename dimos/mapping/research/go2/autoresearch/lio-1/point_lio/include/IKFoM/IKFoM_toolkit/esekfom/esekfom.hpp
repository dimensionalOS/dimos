/*
 *  Copyright (c) 2019--2023, The University of Hong Kong
 *  All rights reserved.
 *
 *  Author: Dongjiao HE <hdj65822@connect.hku.hk>
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

#ifndef ESEKFOM_EKF_HPP
#define ESEKFOM_EKF_HPP


#include <vector>
#include <cstdlib>

#include <boost/bind.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Sparse>

#include "../mtk/types/vect.hpp"
#include "../mtk/types/SOn.hpp"
#include "../mtk/types/S2.hpp"
#include "../mtk/types/SEn.hpp"
#include "../mtk/startIdx.hpp"
#include "../mtk/build_manifold.hpp"
#include "util.hpp"

namespace esekfom {

using namespace Eigen;

template<typename T>
struct dyn_share_modified
{
	bool valid;
	bool converge;
	T M_Noise;
	Eigen::Matrix<T, Eigen::Dynamic, 1> z;
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> h_x;
	Eigen::Matrix<T, 6, 1> z_IMU;
	Eigen::Matrix<T, 6, 1> R_IMU;
	bool satu_check[6];
};

template<typename state, int process_noise_dof, typename input = state, typename measurement=state, int measurement_noise_dof=0>
class esekf{

	typedef esekf self;
	enum{
		n = state::DOF, m = state::DIM, l = measurement::DOF
	};

public:

	typedef typename state::scalar scalar_type;
	typedef Matrix<scalar_type, n, n> cov;
	typedef Matrix<scalar_type, m, n> cov_;
	typedef SparseMatrix<scalar_type> spMt;
	typedef Matrix<scalar_type, n, 1> vectorized_state;
	typedef Matrix<scalar_type, m, 1> flatted_state;
	typedef flatted_state processModel(state &, const input &);
	typedef Eigen::Matrix<scalar_type, m, n> processMatrix1(state &, const input &);
	typedef Eigen::Matrix<scalar_type, m, process_noise_dof> processMatrix2(state &, const input &);
	typedef Eigen::Matrix<scalar_type, process_noise_dof, process_noise_dof> processnoisecovariance;

	typedef void measurementModel_dyn_share_modified(state &, dyn_share_modified<scalar_type> &);
	typedef Eigen::Matrix<scalar_type ,l, n> measurementMatrix1(state &);
	typedef Eigen::Matrix<scalar_type , Eigen::Dynamic, n> measurementMatrix1_dyn(state &);
	typedef Eigen::Matrix<scalar_type ,l, measurement_noise_dof> measurementMatrix2(state &);
	typedef Eigen::Matrix<scalar_type ,Eigen::Dynamic, Eigen::Dynamic> measurementMatrix2_dyn(state &);
	typedef Eigen::Matrix<scalar_type, measurement_noise_dof, measurement_noise_dof> measurementnoisecovariance;
	typedef Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> measurementnoisecovariance_dyn;

	esekf(const state &x = state(),
		const cov  &P = cov::Identity()): x_(x), P_(P){};

	void init_dyn_share_modified(processModel f_in, processMatrix1 f_x_in, measurementModel_dyn_share_modified h_dyn_share_in)
	{
		f = f_in;
		f_x = f_x_in;
		// f_w = f_w_in;
		h_dyn_share_modified_1 = h_dyn_share_in;
		maximum_iter = 1;
		x_.build_S2_state();
		x_.build_SO3_state();
		x_.build_vect_state();
		x_.build_SEN_state();
	}

	void init_dyn_share_modified_2h(processModel f_in, processMatrix1 f_x_in, measurementModel_dyn_share_modified h_dyn_share_in1, measurementModel_dyn_share_modified h_dyn_share_in2)
	{
		f = f_in;
		f_x = f_x_in;
		// f_w = f_w_in;
		h_dyn_share_modified_1 = h_dyn_share_in1;
		h_dyn_share_modified_2 = h_dyn_share_in2;
		maximum_iter = 1;
		x_.build_S2_state();
		x_.build_SO3_state();
		x_.build_vect_state();
		x_.build_SEN_state();
	}

	// iterated error state EKF propogation
	void predict(double &dt, processnoisecovariance &Q, const input &i_in, bool predict_state, bool prop_cov){
		if (predict_state)
		{
			flatted_state f_ = f(x_, i_in);
			x_.oplus(f_, dt);
		}

		if (prop_cov)
		{
			flatted_state f_ = f(x_, i_in);
			// state x_before = x_;

			cov_ f_x_ = f_x(x_, i_in);
			cov f_x_final;
			F_x1 = cov::Identity();
			for (std::vector<std::pair<std::pair<int, int>, int> >::iterator it = x_.vect_state.begin(); it != x_.vect_state.end(); it++) {
				int idx = (*it).first.first;
				int dim = (*it).first.second;
				int dof = (*it).second;
				for(int i = 0; i < n; i++){
					for(int j=0; j<dof; j++)
					{f_x_final(idx+j, i) = f_x_(dim+j, i);}
				}
			}

			Matrix<scalar_type, 3, 3> res_temp_SO3;
			MTK::vect<3, scalar_type> seg_SO3;
			for (std::vector<std::pair<int, int> >::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++) {
				int idx = (*it).first;
				int dim = (*it).second;
				for(int i = 0; i < 3; i++){
					seg_SO3(i) = -1 * f_(dim + i) * dt;
				}
				MTK::SO3<scalar_type> res;
				res.w() = MTK::exp<scalar_type, 3>(res.vec(), seg_SO3, scalar_type(1/2));
				F_x1.template block<3, 3>(idx, idx) = res.normalized().toRotationMatrix();
				res_temp_SO3 = MTK::A_matrix(seg_SO3);
				for(int i = 0; i < n; i++){
					f_x_final. template block<3, 1>(idx, i) = res_temp_SO3 * (f_x_. template block<3, 1>(dim, i));
				}
			}

			F_x1 += f_x_final * dt;
			P_ = F_x1 * P_ * (F_x1).transpose() + Q * (dt * dt);
		}
	}

	bool update_iterated_dyn_share_modified() {
		dyn_share_modified<scalar_type> dyn_share;
		state x_propagated = x_;
		int dof_Measurement;
		double m_noise;
		for(int i=0; i<maximum_iter; i++)
		{
			dyn_share.valid = true;
			h_dyn_share_modified_1(x_, dyn_share);
			if(! dyn_share.valid)
			{
				return false;
				// continue;
			}
			Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> z = dyn_share.z;
			// Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> R = dyn_share.R;
			Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> h_x = dyn_share.h_x;
			// Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> h_v = dyn_share.h_v;
			dof_Measurement = h_x.rows();
			m_noise = dyn_share.M_Noise;
			// dof_Measurement_noise = dyn_share.R.rows();
			// vectorized_state dx, dx_new;
			// x_.boxminus(dx, x_propagated);
			// dx_new = dx;
			// P_ = P_propagated;

			Matrix<scalar_type, n, Eigen::Dynamic> PHT;
			Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> HPHT;
			Matrix<scalar_type, n, Eigen::Dynamic> K_;
			// if(n > dof_Measurement)
			{
				PHT = P_. template block<n, 12>(0, 0) * h_x.transpose();
				HPHT = h_x * PHT.topRows(12);
				for (int m = 0; m < dof_Measurement; m++)
				{
					HPHT(m, m) += m_noise;
				}
				K_= PHT*HPHT.inverse();
			}
			Matrix<scalar_type, n, 1> dx_ = K_ * z; // - h) + (K_x - Matrix<scalar_type, n, n>::Identity()) * dx_new;
			// state x_before = x_;

			x_.boxplus(dx_);
			dyn_share.converge = true;

			// L_ = P_;
			// Matrix<scalar_type, 3, 3> res_temp_SO3;
			// MTK::vect<3, scalar_type> seg_SO3;
			// for(typename std::vector<std::pair<int, int> >::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++) {
			// 	int idx = (*it).first;
			// 	for(int i = 0; i < 3; i++){
			// 		seg_SO3(i) = dx_(i + idx);
			// 	}
			// 	res_temp_SO3 = A_matrix(seg_SO3).transpose();
			// 	for(int i = 0; i < n; i++){
			// 		L_. template block<3, 1>(idx, i) = res_temp_SO3 * (P_. template block<3, 1>(idx, i));
			// 	}
			// 	{
			// 		for(int i = 0; i < dof_Measurement; i++){
			// 			K_. template block<3, 1>(idx, i) = res_temp_SO3 * (K_. template block<3, 1>(idx, i));
			// 		}
			// 	}
			// 	for(int i = 0; i < n; i++){
			// 		L_. template block<1, 3>(i, idx) = (L_. template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
			// 		// P_. template block<1, 3>(i, idx) = (P_. template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
			// 	}
			// 	for(int i = 0; i < n; i++){
			// 		P_. template block<1, 3>(i, idx) = (P_. template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
			// 	}
			// }
			// Matrix<scalar_type, 2, 2> res_temp_S2;
			// MTK::vect<2, scalar_type> seg_S2;
			// for(typename std::vector<std::pair<int, int> >::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++) {
			// 	int idx = (*it).first;

			// 	for(int i = 0; i < 2; i++){
			// 		seg_S2(i) = dx_(i + idx);
			// 	}

			// 	Eigen::Matrix<scalar_type, 2, 3> Nx;
			// 	Eigen::Matrix<scalar_type, 3, 2> Mx;
			// 	x_.S2_Nx_yy(Nx, idx);
			// 	x_propagated.S2_Mx(Mx, seg_S2, idx);
			// 	res_temp_S2 = Nx * Mx;

			// 	for(int i = 0; i < n; i++){
			// 		L_. template block<2, 1>(idx, i) = res_temp_S2 * (P_. template block<2, 1>(idx, i));
			// 	}

			// 	{
			// 		for(int i = 0; i < dof_Measurement; i++){
			// 			K_. template block<2, 1>(idx, i) = res_temp_S2 * (K_. template block<2, 1>(idx, i));
			// 		}
			// 	}
			// 	for(int i = 0; i < n; i++){
			// 		L_. template block<1, 2>(i, idx) = (L_. template block<1, 2>(i, idx)) * res_temp_S2.transpose();
			// 	}
			// 	for(int i = 0; i < n; i++){
			// 		P_. template block<1, 2>(i, idx) = (P_. template block<1, 2>(i, idx)) * res_temp_S2.transpose();
			// 	}
			// }
			// if(n > dof_Measurement)
			{
				P_ = P_ - K_*h_x*P_. template block<12, n>(0, 0);
			}
		}
		return true;
	}

	void update_iterated_dyn_share_IMU() {

		dyn_share_modified<scalar_type> dyn_share;
		for(int i=0; i<maximum_iter; i++)
		{
			dyn_share.valid = true;
			h_dyn_share_modified_2(x_, dyn_share);

			Matrix<scalar_type, 6, 1> z = dyn_share.z_IMU;

			Matrix<double, 30, 6> PHT;
            Matrix<double, 6, 30> HP;
            Matrix<double, 6, 6> HPHT;
			PHT.setZero();
			HP.setZero();
			HPHT.setZero();
			for (int l_ = 0; l_ < 6; l_++)
			{
				if (!dyn_share.satu_check[l_])
				{
					PHT.col(l_) = P_.col(15+l_) + P_.col(24+l_);
					HP.row(l_) = P_.row(15+l_) + P_.row(24+l_);
				}
			}
			for (int l_ = 0; l_ < 6; l_++)
			{
				if (!dyn_share.satu_check[l_])
				{
					HPHT.col(l_) = HP.col(15+l_) + HP.col(24+l_);
				}
				HPHT(l_, l_) += dyn_share.R_IMU(l_); //, l);
			}
        	Eigen::Matrix<double, 30, 6> K = PHT * HPHT.inverse();

            Matrix<scalar_type, n, 1> dx_ = K * z;

            P_ -= K * HP;
			x_.boxplus(dx_);
		}
		return;
	}

	void change_x(state &input_state)
	{
		x_ = input_state;

		if((!x_.vect_state.size())&&(!x_.SO3_state.size())&&(!x_.S2_state.size())&&(!x_.SEN_state.size()))
		{
			x_.build_S2_state();
			x_.build_SO3_state();
			x_.build_vect_state();
			x_.build_SEN_state();
		}
	}

	void change_P(cov &input_cov)
	{
		P_ = input_cov;
	}

	const state& get_x() const {
		return x_;
	}
	const cov& get_P() const {
		return P_;
	}
	state x_;
private:
	measurement m_;
	cov P_;
	spMt l_;
	spMt f_x_1;
	spMt f_x_2;
	cov F_x1 = cov::Identity();
	cov F_x2 = cov::Identity();
	cov L_ = cov::Identity();

	processModel *f;
	processMatrix1 *f_x;
	processMatrix2 *f_w;

	measurementMatrix1 *h_x;
	measurementMatrix2 *h_v;

	measurementMatrix1_dyn *h_x_dyn;
	measurementMatrix2_dyn *h_v_dyn;

	measurementModel_dyn_share_modified *h_dyn_share_modified_1;

	measurementModel_dyn_share_modified *h_dyn_share_modified_2;

	int maximum_iter = 0;
	scalar_type limit[n];

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace esekfom

#endif //  ESEKFOM_EKF_HPP
