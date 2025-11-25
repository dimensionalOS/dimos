# Copyright 2025 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

from dataclasses import dataclass, field
import time

import cv2
import numpy as np

from dimos.msgs.sensor_msgs.image_impls.AbstractImage import (
    HAS_CUDA,
    AbstractImage,
    ImageFormat,
    _ascontig,
    _is_cu,
    _to_cpu,
)

try:
    import cupy as cp  # type: ignore
    from cupyx.scipy import (  # type: ignore[import-not-found]
        ndimage as cndimage,
        signal as csignal,
    )
except Exception:  # pragma: no cover
    cp = None
    cndimage = None
    csignal = None


_CUDA_SRC = r"""
extern "C" {

__device__ __forceinline__ void rodrigues_R(const float r[3], float R[9]){
  float theta = sqrtf(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
  if(theta < 1e-8f){
    R[0]=1.f; R[1]=0.f; R[2]=0.f;
    R[3]=0.f; R[4]=1.f; R[5]=0.f;
    R[6]=0.f; R[7]=0.f; R[8]=1.f;
    return;
  }
  float kx=r[0]/theta, ky=r[1]/theta, kz=r[2]/theta;
  float c=cosf(theta), s=sinf(theta), v=1.f-c;
  R[0]=kx*kx*v + c;      R[1]=kx*ky*v - kz*s; R[2]=kx*kz*v + ky*s;
  R[3]=ky*kx*v + kz*s;   R[4]=ky*ky*v + c;    R[5]=ky*kz*v - kx*s;
  R[6]=kz*kx*v - ky*s;   R[7]=kz*ky*v + kx*s; R[8]=kz*kz*v + c;
}

__device__ __forceinline__ void mat3x3_vec3(const float R[9], const float x[3], float y[3]){
  y[0] = R[0]*x[0] + R[1]*x[1] + R[2]*x[2];
  y[1] = R[3]*x[0] + R[4]*x[1] + R[5]*x[2];
  y[2] = R[6]*x[0] + R[7]*x[1] + R[8]*x[2];
}

__device__ __forceinline__ void cross_mat(const float v[3], float S[9]){
  S[0]=0.f;     S[1]=-v[2]; S[2]= v[1];
  S[3]= v[2];   S[4]=0.f;   S[5]=-v[0];
  S[6]=-v[1];   S[7]= v[0]; S[8]=0.f;
}

// Solve a 6x6 system (JTJ * x = JTr) with Gauss-Jordan; JTJ is SPD after damping.
__device__ void solve6_gauss_jordan(float A[36], float b[6], float x[6]){
  float M[6][7];
  #pragma unroll
  for(int r=0;r<6;++r){
    #pragma unroll
    for(int c=0;c<6;++c) M[r][c] = A[r*6 + c];
    M[r][6] = b[r];
  }
  for(int piv=0;piv<6;++piv){
    float invd = 1.f / M[piv][piv];
    for(int c=piv;c<7;++c) M[piv][c] *= invd;
    for(int r=0;r<6;++r){
      if(r==piv) continue;
      float f = M[r][piv];
      if(fabsf(f) < 1e-20f) continue;
      for(int c=piv;c<7;++c) M[r][c] -= f * M[piv][c];
    }
  }
  #pragma unroll
  for(int r=0;r<6;++r) x[r] = M[r][6];
}

// One block solves one pose; dynamic shared memory holds per-thread accumulators.
__global__ void pnp_gn_batch(
    const float* __restrict__ obj,   // (B,N,3)
    const float* __restrict__ img,   // (B,N,2)
    const int N,
    const float* __restrict__ intr,  // (B,4) -> fx, fy, cx, cy
    const int max_iters,
    const float damping,
    float* __restrict__ rvec_out,    // (B,3)
    float* __restrict__ tvec_out     // (B,3)
){
  if(N <= 0) return;
  int b = blockIdx.x;
  const float* obj_b = obj + b * N * 3;
  const float* img_b = img + b * N * 2;
  float fx = intr[4*b + 0];
  float fy = intr[4*b + 1];
  float cx = intr[4*b + 2];
  float cy = intr[4*b + 3];

  __shared__ float s_R[9];
  __shared__ float s_rvec[3];
  __shared__ float s_tvec[3];
  __shared__ float s_JTJ[36];
  __shared__ float s_JTr[6];
  __shared__ int   s_done;

  extern __shared__ float scratch[];
  float* sh_JTJ = scratch;
  float* sh_JTr = scratch + 36 * blockDim.x;

  if(threadIdx.x==0){
    s_rvec[0]=0.f; s_rvec[1]=0.f; s_rvec[2]=0.f;
    s_tvec[0]=0.f; s_tvec[1]=0.f; s_tvec[2]=2.f;
  }
  __syncthreads();

  for(int it=0; it<max_iters; ++it){
    if(threadIdx.x==0){
      rodrigues_R(s_rvec, s_R);
      s_done = 0;
    }
    __syncthreads();

    float lJTJ[36];
    float lJTr[6];
    #pragma unroll
    for(int k=0;k<36;++k) lJTJ[k]=0.f;
    #pragma unroll
    for(int k=0;k<6;++k) lJTr[k]=0.f;

    for(int i=threadIdx.x; i<N; i+=blockDim.x){
      float Xi[3] = {obj_b[3*i+0], obj_b[3*i+1], obj_b[3*i+2]};
      float mi_u  = img_b[2*i+0];
      float mi_v  = img_b[2*i+1];

      float RX[3];
      mat3x3_vec3(s_R, Xi, RX);
      float Xc[3];
      Xc[0]=RX[0]+s_tvec[0];
      Xc[1]=RX[1]+s_tvec[1];
      Xc[2]=RX[2]+s_tvec[2];

      float invZ = 1.f / fmaxf(Xc[2], 1e-6f);
      float uhat = fx * Xc[0]*invZ + cx;
      float vhat = fy * Xc[1]*invZ + cy;

      float ru = mi_u - uhat;
      float rv = mi_v - vhat;

      float du_dX = fx * invZ;
      float du_dY = 0.f;
      float du_dZ = -fx * Xc[0] * invZ * invZ;

      float dv_dX = 0.f;
      float dv_dY = fy * invZ;
      float dv_dZ = -fy * Xc[1] * invZ * invZ;

      float S[9];
      cross_mat(Xi, S);
      float M[9];
      M[0]=-(s_R[0]*S[0] + s_R[1]*S[3] + s_R[2]*S[6]);
      M[1]=-(s_R[0]*S[1] + s_R[1]*S[4] + s_R[2]*S[7]);
      M[2]=-(s_R[0]*S[2] + s_R[1]*S[5] + s_R[2]*S[8]);
      M[3]=-(s_R[3]*S[0] + s_R[4]*S[3] + s_R[5]*S[6]);
      M[4]=-(s_R[3]*S[1] + s_R[4]*S[4] + s_R[5]*S[7]);
      M[5]=-(s_R[3]*S[2] + s_R[4]*S[5] + s_R[5]*S[8]);
      M[6]=-(s_R[6]*S[0] + s_R[7]*S[3] + s_R[8]*S[6]);
      M[7]=-(s_R[6]*S[1] + s_R[7]*S[4] + s_R[8]*S[7]);
      M[8]=-(s_R[6]*S[2] + s_R[7]*S[5] + s_R[8]*S[8]);

      float Ju_r = du_dX*M[0] + du_dY*M[3] + du_dZ*M[6];
      float Ju_p = du_dX*M[1] + du_dY*M[4] + du_dZ*M[7];
      float Ju_y = du_dX*M[2] + du_dY*M[5] + du_dZ*M[8];

      float Jv_r = dv_dX*M[0] + dv_dY*M[3] + dv_dZ*M[6];
      float Jv_p = dv_dX*M[1] + dv_dY*M[4] + dv_dZ*M[7];
      float Jv_y = dv_dX*M[2] + dv_dY*M[5] + dv_dZ*M[8];

      float Ju_tx = du_dX, Ju_ty = du_dY, Ju_tz = du_dZ;
      float Jv_tx = dv_dX, Jv_ty = dv_dY, Jv_tz = dv_dZ;

      float J0[6] = {Ju_r, Ju_p, Ju_y, Ju_tx, Ju_ty, Ju_tz};
      float J1[6] = {Jv_r, Jv_p, Jv_y, Jv_tx, Jv_ty, Jv_tz};

      int idx = 0;
      for(int a=0;a<6;++a){
        lJTr[a] += J0[a]*ru + J1[a]*rv;
        for(int b=0;b<6;++b){
          lJTJ[idx++] += J0[a]*J0[b] + J1[a]*J1[b];
        }
      }
    }

    for(int k=0;k<36;++k){
      sh_JTJ[k*blockDim.x + threadIdx.x] = lJTJ[k];
    }
    for(int k=0;k<6;++k){
      sh_JTr[k*blockDim.x + threadIdx.x] = lJTr[k];
    }
    __syncthreads();

    if(threadIdx.x==0){
      for(int k=0;k<36;++k){
        float sum = 0.f;
        for(int t=0;t<blockDim.x;++t){
          sum += sh_JTJ[k*blockDim.x + t];
        }
        s_JTJ[k] = sum;
      }
      for(int k=0;k<6;++k){
        float sum = 0.f;
        for(int t=0;t<blockDim.x;++t){
          sum += sh_JTr[k*blockDim.x + t];
        }
        s_JTr[k] = sum;
      }
      for(int d=0; d<6; ++d){
        s_JTJ[d*6 + d] += damping;
      }

      float delta[6];
      solve6_gauss_jordan(s_JTJ, s_JTr, delta);

      s_rvec[0] += delta[0]; s_rvec[1] += delta[1]; s_rvec[2] += delta[2];
      s_tvec[0] += delta[3]; s_tvec[1] += delta[4]; s_tvec[2] += delta[5];

      float nrm = fabsf(delta[0])+fabsf(delta[1])+fabsf(delta[2])
                + fabsf(delta[3])+fabsf(delta[4])+fabsf(delta[5]);
      s_done = (nrm < 1e-6f) ? 1 : 0;
    }
    __syncthreads();
    if(s_done) break;
  }

  if(threadIdx.x==0){
    rvec_out[3*b + 0]=s_rvec[0];
    rvec_out[3*b + 1]=s_rvec[1];
    rvec_out[3*b + 2]=s_rvec[2];
    tvec_out[3*b + 0]=s_tvec[0];
    tvec_out[3*b + 1]=s_tvec[1];
    tvec_out[3*b + 2]=s_tvec[2];
  }
}

} // extern "C"
"""

if cp is not None:
    _mod = cp.RawModule(code=_CUDA_SRC, options=("-std=c++14",), name_expressions=("pnp_gn_batch",))
    _pnp_kernel = _mod.get_function("pnp_gn_batch")


def _solve_pnp_cuda_kernel(obj, img, K, iterations: int = 15, damping: float = 1e-6):  # type: ignore[no-untyped-def]
    if cp is None:
        raise RuntimeError("CuPy/CUDA not available")

    obj_cu = cp.asarray(obj, dtype=cp.float32)
    if obj_cu.ndim == 2:
        obj_cu = obj_cu[None, ...]
    if obj_cu.ndim != 3 or obj_cu.shape[2] != 3:
        raise ValueError("object_points must have shape (..., 3)")
    B, N, _ = obj_cu.shape
    if N <= 0:
        raise ValueError("object_points must contain at least one correspondence")

    img_cu = cp.asarray(img, dtype=cp.float32)
    if img_cu.ndim == 2:
        img_cu = img_cu[None, ...]
    if img_cu.ndim != 3 or img_cu.shape[2] != 2:
        raise ValueError("image_points must have shape (..., 2)")
    if img_cu.shape[0] != B or img_cu.shape[1] != N:
        raise ValueError("object and image batches must align")

    obj_cu = cp.ascontiguousarray(obj_cu)
    img_cu = cp.ascontiguousarray(img_cu)

    K_np = np.asarray(_to_cpu(K), dtype=np.float32)  # type: ignore[no-untyped-call]
    np_intri = np.empty((B, 4), dtype=np.float32)
    if K_np.ndim == 2:
        if K_np.shape != (3, 3):
            raise ValueError("camera_matrix must be 3x3 or batched 3x3")
        fx, fy = K_np[0, 0], K_np[1, 1]
        cx, cy = K_np[0, 2], K_np[1, 2]
        np_intri[:] = (fx, fy, cx, cy)
    elif K_np.ndim == 3:
        if K_np.shape[0] != B or K_np.shape[1:] != (3, 3):
            raise ValueError("batched camera_matrix must match batch size and be 3x3")
        np_intri[:, 0] = K_np[:, 0, 0]
        np_intri[:, 1] = K_np[:, 1, 1]
        np_intri[:, 2] = K_np[:, 0, 2]
        np_intri[:, 3] = K_np[:, 1, 2]
    else:
        raise ValueError("camera_matrix must be 3x3 or batched 3x3")

    intr_cu = cp.asarray(np_intri, dtype=cp.float32)
    r_out = cp.empty((B, 3), dtype=cp.float32)
    t_out = cp.empty((B, 3), dtype=cp.float32)

    threads = 256
    if N < threads:
        threads = 32
        while threads < N and threads < 256:
            threads <<= 1
        threads = max(threads, 32)
    shared_mem = (36 + 6) * threads * np.dtype(np.float32).itemsize

    _pnp_kernel(
        (B,),
        (threads,),
        (
            obj_cu,
            img_cu,
            np.int32(int(N)),
            intr_cu,
            np.int32(int(iterations)),
            np.float32(damping),
            r_out,
            t_out,
        ),
        shared_mem=shared_mem,
    )

    r_host = cp.asnumpy(r_out).reshape(B, 3, 1).astype(np.float64, copy=False)
    t_host = cp.asnumpy(t_out).reshape(B, 3, 1).astype(np.float64, copy=False)
    if B == 1:
        return r_host[0], t_host[0]
    return r_host, t_host


def _bgr_to_rgb_cuda(img):  # type: ignore[no-untyped-def]
    return img[..., ::-1]


def _rgb_to_bgr_cuda(img):  # type: ignore[no-untyped-def]
    return img[..., ::-1]


def _bgra_to_rgba_cuda(img):  # type: ignore[no-untyped-def]
    out = img.copy()
    out[..., 0], out[..., 2] = img[..., 2], img[..., 0]
    return out


def _rgba_to_bgra_cuda(img):  # type: ignore[no-untyped-def]
    out = img.copy()
    out[..., 0], out[..., 2] = img[..., 2], img[..., 0]
    return out


def _gray_to_rgb_cuda(gray):  # type: ignore[no-untyped-def]
    return cp.stack([gray, gray, gray], axis=-1)


def _rgb_to_gray_cuda(rgb):  # type: ignore[no-untyped-def]
    r = rgb[..., 0].astype(cp.float32)
    g = rgb[..., 1].astype(cp.float32)
    b = rgb[..., 2].astype(cp.float32)
    # These come from the Rec.601 conversion for YUV. R = 0.299, G = 0.587, B = 0.114
    y = 0.299 * r + 0.587 * g + 0.114 * b
    if rgb.dtype == cp.uint8:
        y = cp.clip(y, 0, 255).astype(cp.uint8)
    return y


def _resize_bilinear_hwc_cuda(img, out_h: int, out_w: int):  # type: ignore[no-untyped-def]
    if cp is None or cndimage is None:
        raise RuntimeError("CuPy/CUDA not available")
    if img.ndim not in (2, 3):
        raise ValueError("Expected HxW or HxWxC array")

    work = img[..., None] if img.ndim == 2 else img
    squeezed = work is not img
    in_h, in_w = work.shape[:2]
    if (in_h, in_w) == (out_h, out_w):
        return img.copy()

    zoom = (out_h / in_h, out_w / in_w, 1.0)
    out = cndimage.zoom(
        work.astype(cp.float32, copy=False),
        zoom=zoom,
        order=1,
        mode="nearest",
        prefilter=False,
        grid_mode=True,
    )

    if squeezed:
        out = out[..., 0]
    if img.dtype == cp.uint8:
        out = cp.clip(out, 0, 255).astype(cp.uint8, copy=False)
    elif out.dtype != img.dtype:
        out = out.astype(img.dtype, copy=False)
    return out


def _rodrigues(x, inverse: bool = False):  # type: ignore[no-untyped-def]
    """Unified Rodrigues transform (vector<->matrix) for NumPy/CuPy arrays."""

    if cp is not None and (
        isinstance(x, cp.ndarray) or getattr(x, "__cuda_array_interface__", None) is not None
    ):
        xp = cp
    else:
        xp = np
    arr = xp.asarray(x, dtype=xp.float64)

    if not inverse and arr.ndim >= 2 and arr.shape[-2:] == (3, 3):
        inverse = True

    if not inverse:
        vec = arr
        if vec.ndim >= 2 and vec.shape[-1] == 1:
            vec = vec[..., 0]
        if vec.shape[-1] != 3:
            raise ValueError("Rodrigues expects vectors of shape (..., 3)")
        orig_shape = vec.shape[:-1]
        vec = vec.reshape(-1, 3)
        n = vec.shape[0]
        theta = xp.linalg.norm(vec, axis=1)
        small = theta < 1e-12

        def _skew(v):  # type: ignore[no-untyped-def]
            vx, vy, vz = v[:, 0], v[:, 1], v[:, 2]
            O = xp.zeros_like(vx)
            return xp.stack(
                [
                    xp.stack([O, -vz, vy], axis=-1),
                    xp.stack([vz, O, -vx], axis=-1),
                    xp.stack([-vy, vx, O], axis=-1),
                ],
                axis=-2,
            )

        K = _skew(vec)  # type: ignore[no-untyped-call]
        theta2 = theta * theta
        theta4 = theta2 * theta2
        theta_safe = xp.where(small, 1.0, theta)
        theta2_safe = xp.where(small, 1.0, theta2)
        A = xp.where(small, 1.0 - theta2 / 6.0 + theta4 / 120.0, xp.sin(theta) / theta_safe)[
            :, None, None
        ]
        B = xp.where(
            small,
            0.5 - theta2 / 24.0 + theta4 / 720.0,
            (1.0 - xp.cos(theta)) / theta2_safe,
        )[:, None, None]
        I = xp.eye(3, dtype=arr.dtype)
        I = I[None, :, :] if n == 1 else xp.broadcast_to(I, (n, 3, 3))
        KK = xp.matmul(K, K)
        out = I + A * K + B * KK
        return out.reshape((*orig_shape, 3, 3)) if orig_shape else out[0]

    mat = arr
    if mat.shape[-2:] != (3, 3):
        raise ValueError("Rodrigues expects rotation matrices of shape (..., 3, 3)")
    orig_shape = mat.shape[:-2]
    mat = mat.reshape(-1, 3, 3)
    trace = xp.trace(mat, axis1=1, axis2=2)
    trace = xp.clip((trace - 1.0) / 2.0, -1.0, 1.0)
    theta = xp.arccos(trace)
    v = xp.stack(
        [
            mat[:, 2, 1] - mat[:, 1, 2],
            mat[:, 0, 2] - mat[:, 2, 0],
            mat[:, 1, 0] - mat[:, 0, 1],
        ],
        axis=1,
    )
    norm_v = xp.linalg.norm(v, axis=1)
    small = theta < 1e-7
    eps = 1e-8
    norm_safe = xp.where(norm_v < eps, 1.0, norm_v)
    r_general = theta[:, None] * v / norm_safe[:, None]
    r_small = 0.5 * v
    r = xp.where(small[:, None], r_small, r_general)
    pi_mask = xp.abs(theta - xp.pi) < 1e-4
    if np.any(pi_mask) if xp is np else bool(cp.asnumpy(pi_mask).any()):
        diag = xp.diagonal(mat, axis1=1, axis2=2)
        axis_candidates = xp.clip((diag + 1.0) / 2.0, 0.0, None)
        axis = xp.sqrt(axis_candidates)
        signs = xp.sign(v)
        axis = xp.where(signs == 0, axis, xp.copysign(axis, signs))
        axis_norm = xp.linalg.norm(axis, axis=1)
        axis_norm = xp.where(axis_norm < eps, 1.0, axis_norm)
        axis = axis / axis_norm[:, None]
        r_pi = theta[:, None] * axis
        r = xp.where(pi_mask[:, None], r_pi, r)
    out = r.reshape((*orig_shape, 3)) if orig_shape else r[0]
    return out


def _undistort_points_cuda(
    img_px: cp.ndarray, K: cp.ndarray, dist: cp.ndarray, iterations: int = 8
) -> cp.ndarray:
    """Iteratively undistort pixel coordinates on device (Brown–Conrady).

    Returns pixel coordinates after undistortion (fx*xu+cx, fy*yu+cy).
    """
    N = img_px.shape[0]
    ones = cp.ones((N, 1), dtype=cp.float64)
    uv1 = cp.concatenate([img_px.astype(cp.float64), ones], axis=1)
    Kinv = cp.linalg.inv(K)
    xdyd1 = uv1 @ Kinv.T
    xd = xdyd1[:, 0]
    yd = xdyd1[:, 1]
    xu = xd.copy()
    yu = yd.copy()
    k1 = dist[0]
    k2 = dist[1] if dist.size > 1 else 0.0
    p1 = dist[2] if dist.size > 2 else 0.0
    p2 = dist[3] if dist.size > 3 else 0.0
    k3 = dist[4] if dist.size > 4 else 0.0
    for _ in range(iterations):
        r2 = xu * xu + yu * yu
        r4 = r2 * r2
        r6 = r4 * r2
        radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6
        delta_x = 2.0 * p1 * xu * yu + p2 * (r2 + 2.0 * xu * xu)
        delta_y = p1 * (r2 + 2.0 * yu * yu) + 2.0 * p2 * xu * yu
        xu = (xd - delta_x) / radial
        yu = (yd - delta_y) / radial
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    return cp.stack([fx * xu + cx, fy * yu + cy], axis=1)


@dataclass
class CudaImage(AbstractImage):
    data: any  # type: ignore[valid-type]  # cupy.ndarray
    format: ImageFormat = field(default=ImageFormat.BGR)
    frame_id: str = field(default="")
    ts: float = field(default_factory=time.time)

    def __post_init__(self):  # type: ignore[no-untyped-def]
        if not HAS_CUDA or cp is None:
            raise RuntimeError("CuPy/CUDA not available")
        if not _is_cu(self.data):
            # Accept NumPy arrays and move to device automatically
            try:
                self.data = cp.asarray(self.data)
            except Exception as e:
                raise ValueError("CudaImage requires a CuPy array") from e
        if self.data.ndim < 2:  # type: ignore[attr-defined]
            raise ValueError("Image data must be at least 2D")
        self.data = _ascontig(self.data)  # type: ignore[no-untyped-call]

    @property
    def is_cuda(self) -> bool:
        return True

    def to_opencv(self) -> np.ndarray:  # type: ignore[type-arg]
        if self.format in (ImageFormat.BGR, ImageFormat.RGB, ImageFormat.RGBA, ImageFormat.BGRA):
            return _to_cpu(self.to_bgr().data)  # type: ignore[no-any-return, no-untyped-call]
        return _to_cpu(self.data)  # type: ignore[no-any-return, no-untyped-call]

    def to_rgb(self) -> CudaImage:
        if self.format == ImageFormat.RGB:
            return self.copy()  # type: ignore
        if self.format == ImageFormat.BGR:
            return CudaImage(_bgr_to_rgb_cuda(self.data), ImageFormat.RGB, self.frame_id, self.ts)  # type: ignore[no-untyped-call]
        if self.format == ImageFormat.RGBA:
            return self.copy()  # type: ignore
        if self.format == ImageFormat.BGRA:
            return CudaImage(
                _bgra_to_rgba_cuda(self.data),  # type: ignore[no-untyped-call]
                ImageFormat.RGBA,
                self.frame_id,
                self.ts,
            )
        if self.format == ImageFormat.GRAY:
            return CudaImage(_gray_to_rgb_cuda(self.data), ImageFormat.RGB, self.frame_id, self.ts)  # type: ignore[no-untyped-call]
        if self.format in (ImageFormat.GRAY16, ImageFormat.DEPTH16):
            gray8 = (self.data.astype(cp.float32) / 256.0).clip(0, 255).astype(cp.uint8)  # type: ignore
            return CudaImage(_gray_to_rgb_cuda(gray8), ImageFormat.RGB, self.frame_id, self.ts)  # type: ignore[no-untyped-call]
        return self.copy()  # type: ignore

    def to_bgr(self) -> CudaImage:
        if self.format == ImageFormat.BGR:
            return self.copy()  # type: ignore
        if self.format == ImageFormat.RGB:
            return CudaImage(_rgb_to_bgr_cuda(self.data), ImageFormat.BGR, self.frame_id, self.ts)  # type: ignore[no-untyped-call]
        if self.format == ImageFormat.RGBA:
            return CudaImage(
                _rgba_to_bgra_cuda(self.data)[..., :3],  # type: ignore[no-untyped-call]
                ImageFormat.BGR,
                self.frame_id,
                self.ts,
            )
        if self.format == ImageFormat.BGRA:
            return CudaImage(self.data[..., :3], ImageFormat.BGR, self.frame_id, self.ts)  # type: ignore[index]
        if self.format in (ImageFormat.GRAY, ImageFormat.DEPTH):
            return CudaImage(
                _rgb_to_bgr_cuda(_gray_to_rgb_cuda(self.data)),  # type: ignore[no-untyped-call]
                ImageFormat.BGR,
                self.frame_id,
                self.ts,
            )
        if self.format in (ImageFormat.GRAY16, ImageFormat.DEPTH16):
            gray8 = (self.data.astype(cp.float32) / 256.0).clip(0, 255).astype(cp.uint8)  # type: ignore
            return CudaImage(
                _rgb_to_bgr_cuda(_gray_to_rgb_cuda(gray8)),  # type: ignore[no-untyped-call]
                ImageFormat.BGR,
                self.frame_id,
                self.ts,
            )
        return self.copy()  # type: ignore

    def to_grayscale(self) -> CudaImage:
        if self.format in (ImageFormat.GRAY, ImageFormat.GRAY16, ImageFormat.DEPTH):
            return self.copy()  # type: ignore
        if self.format == ImageFormat.BGR:
            return CudaImage(
                _rgb_to_gray_cuda(_bgr_to_rgb_cuda(self.data)),  # type: ignore[no-untyped-call]
                ImageFormat.GRAY,
                self.frame_id,
                self.ts,
            )
        if self.format == ImageFormat.RGB:
            return CudaImage(_rgb_to_gray_cuda(self.data), ImageFormat.GRAY, self.frame_id, self.ts)  # type: ignore[no-untyped-call]
        if self.format in (ImageFormat.RGBA, ImageFormat.BGRA):
            rgb = (
                self.data[..., :3]  # type: ignore[index]
                if self.format == ImageFormat.RGBA
                else _bgra_to_rgba_cuda(self.data)[..., :3]  # type: ignore[no-untyped-call]
            )
            return CudaImage(_rgb_to_gray_cuda(rgb), ImageFormat.GRAY, self.frame_id, self.ts)  # type: ignore[no-untyped-call]
        raise ValueError(f"Unsupported format: {self.format}")

    def resize(self, width: int, height: int, interpolation: int = cv2.INTER_LINEAR) -> CudaImage:
        return CudaImage(
            _resize_bilinear_hwc_cuda(self.data, height, width), self.format, self.frame_id, self.ts
        )

    def crop(self, x: int, y: int, width: int, height: int) -> CudaImage:
        """Crop the image to the specified region.

        Args:
            x: Starting x coordinate (left edge)
            y: Starting y coordinate (top edge)
            width: Width of the cropped region
            height: Height of the cropped region

        Returns:
            A new CudaImage containing the cropped region
        """
        # Get current image dimensions
        img_height, img_width = self.data.shape[:2]  # type: ignore[attr-defined]

        # Clamp the crop region to image bounds
        x = max(0, min(x, img_width))
        y = max(0, min(y, img_height))
        x_end = min(x + width, img_width)
        y_end = min(y + height, img_height)

        # Perform the crop using array slicing
        if self.data.ndim == 2:  # type: ignore[attr-defined]
            # Grayscale image
            cropped_data = self.data[y:y_end, x:x_end]  # type: ignore[index]
        else:
            # Color image (HxWxC)
            cropped_data = self.data[y:y_end, x:x_end, :]  # type: ignore[index]

        # Return a new CudaImage with the cropped data
        return CudaImage(cropped_data, self.format, self.frame_id, self.ts)

    def sharpness(self) -> float:
        if cp is None:
            return 0.0
        try:
            from cupyx.scipy import ndimage as cndimage

            gray = self.to_grayscale().data.astype(cp.float32)  # type: ignore[attr-defined]
            deriv5 = cp.asarray([1, 2, 0, -2, -1], dtype=cp.float32)
            smooth5 = cp.asarray([1, 4, 6, 4, 1], dtype=cp.float32)
            gx = cndimage.convolve1d(gray, deriv5, axis=1, mode="reflect")
            gx = cndimage.convolve1d(gx, smooth5, axis=0, mode="reflect")
            gy = cndimage.convolve1d(gray, deriv5, axis=0, mode="reflect")
            gy = cndimage.convolve1d(gy, smooth5, axis=1, mode="reflect")
            magnitude = cp.hypot(gx, gy)
            mean_mag = float(cp.asnumpy(magnitude.mean()))
        except Exception:
            return 0.0
        if mean_mag <= 0:
            return 0.0
        return float(np.clip((np.log10(mean_mag + 1) - 1.7) / 2.0, 0.0, 1.0))

    # CUDA tracker (template NCC with small scale pyramid)
    @dataclass
    class BBox:
        x: int
        y: int
        w: int
        h: int

    def create_csrt_tracker(self, bbox: BBox):  # type: ignore[no-untyped-def]
        if csignal is None:
            raise RuntimeError("cupyx.scipy.signal not available for CUDA tracker")
        x, y, w, h = map(int, bbox)  # type: ignore[call-overload]
        gray = self.to_grayscale().data.astype(cp.float32)  # type: ignore[attr-defined]
        tmpl = gray[y : y + h, x : x + w]
        if tmpl.size == 0:
            raise ValueError("Invalid bbox for CUDA tracker")
        return _CudaTemplateTracker(tmpl, x0=x, y0=y)

    def csrt_update(self, tracker) -> tuple[bool, tuple[int, int, int, int]]:  # type: ignore[no-untyped-def]
        if not isinstance(tracker, _CudaTemplateTracker):
            raise TypeError("Expected CUDA tracker instance")
        gray = self.to_grayscale().data.astype(cp.float32)  # type: ignore[attr-defined]
        x, y, w, h = tracker.update(gray)
        return True, (int(x), int(y), int(w), int(h))

    # PnP – Gauss–Newton (no distortion in batch), iterative per-instance
    def solve_pnp(
        self,
        object_points: np.ndarray,  # type: ignore[type-arg]
        image_points: np.ndarray,  # type: ignore[type-arg]
        camera_matrix: np.ndarray,  # type: ignore[type-arg]
        dist_coeffs: np.ndarray | None = None,  # type: ignore[type-arg]
        flags: int = cv2.SOLVEPNP_ITERATIVE,
    ) -> tuple[bool, np.ndarray, np.ndarray]:  # type: ignore[type-arg]
        if not HAS_CUDA or cp is None or (dist_coeffs is not None and np.any(dist_coeffs)):
            obj = np.asarray(object_points, dtype=np.float32).reshape(-1, 3)
            img = np.asarray(image_points, dtype=np.float32).reshape(-1, 2)
            K = np.asarray(camera_matrix, dtype=np.float64)
            dist = None if dist_coeffs is None else np.asarray(dist_coeffs, dtype=np.float64)
            ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=flags)  # type: ignore[arg-type]
            return bool(ok), rvec.astype(np.float64), tvec.astype(np.float64)

        rvec, tvec = _solve_pnp_cuda_kernel(object_points, image_points, camera_matrix)
        ok = np.isfinite(rvec).all() and np.isfinite(tvec).all()
        return ok, rvec, tvec

    def solve_pnp_batch(
        self,
        object_points_batch: np.ndarray,  # type: ignore[type-arg]
        image_points_batch: np.ndarray,  # type: ignore[type-arg]
        camera_matrix: np.ndarray,  # type: ignore[type-arg]
        dist_coeffs: np.ndarray | None = None,  # type: ignore[type-arg]
        iterations: int = 15,
        damping: float = 1e-6,
    ) -> tuple[np.ndarray, np.ndarray]:  # type: ignore[type-arg]
        """Batched PnP (each block = one instance)."""
        if not HAS_CUDA or cp is None or (dist_coeffs is not None and np.any(dist_coeffs)):
            obj = np.asarray(object_points_batch, dtype=np.float32)
            img = np.asarray(image_points_batch, dtype=np.float32)
            if obj.ndim != 3 or img.ndim != 3 or obj.shape[:2] != img.shape[:2]:
                raise ValueError(
                    "Batched object/image arrays must be shaped (B,N,...) with matching sizes"
                )
            K = np.asarray(camera_matrix, dtype=np.float64)
            dist = None if dist_coeffs is None else np.asarray(dist_coeffs, dtype=np.float64)
            B = obj.shape[0]
            r_list = np.empty((B, 3, 1), dtype=np.float64)
            t_list = np.empty((B, 3, 1), dtype=np.float64)
            for b in range(B):
                K_b = K if K.ndim == 2 else K[b]
                dist_b = None
                if dist is not None:
                    if dist.ndim == 1:
                        dist_b = dist
                    elif dist.ndim == 2:
                        dist_b = dist[b]
                    else:
                        raise ValueError("dist_coeffs must be 1D or batched 2D")
                ok, rvec, tvec = cv2.solvePnP(
                    obj[b],
                    img[b],
                    K_b,
                    dist_b,  # type: ignore[arg-type]
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )
                if not ok:
                    raise RuntimeError(f"cv2.solvePnP failed for batch index {b}")
                r_list[b] = rvec.astype(np.float64)
                t_list[b] = tvec.astype(np.float64)
            return r_list, t_list

        return _solve_pnp_cuda_kernel(  # type: ignore[no-any-return]
            object_points_batch,
            image_points_batch,
            camera_matrix,
            iterations=iterations,
            damping=damping,
        )

    def solve_pnp_ransac(
        self,
        object_points: np.ndarray,  # type: ignore[type-arg]
        image_points: np.ndarray,  # type: ignore[type-arg]
        camera_matrix: np.ndarray,  # type: ignore[type-arg]
        dist_coeffs: np.ndarray | None = None,  # type: ignore[type-arg]
        iterations_count: int = 100,
        reprojection_error: float = 3.0,
        confidence: float = 0.99,
        min_sample: int = 6,
    ) -> tuple[bool, np.ndarray, np.ndarray, np.ndarray]:  # type: ignore[type-arg]
        """RANSAC with CUDA PnP solver."""
        if not HAS_CUDA or cp is None or (dist_coeffs is not None and np.any(dist_coeffs)):
            obj = np.asarray(object_points, dtype=np.float32)
            img = np.asarray(image_points, dtype=np.float32)
            K = np.asarray(camera_matrix, dtype=np.float64)
            dist = None if dist_coeffs is None else np.asarray(dist_coeffs, dtype=np.float64)
            ok, rvec, tvec, mask = cv2.solvePnPRansac(
                obj,
                img,
                K,
                dist,  # type: ignore[arg-type]
                iterationsCount=int(iterations_count),
                reprojectionError=float(reprojection_error),
                confidence=float(confidence),
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            mask_flat = np.zeros((obj.shape[0],), dtype=np.uint8)
            if mask is not None and len(mask) > 0:
                mask_flat[mask.flatten()] = 1
            return bool(ok), rvec.astype(np.float64), tvec.astype(np.float64), mask_flat

        obj = cp.asarray(object_points, dtype=cp.float32)
        img = cp.asarray(image_points, dtype=cp.float32)
        camera_matrix_np = np.asarray(_to_cpu(camera_matrix), dtype=np.float32)  # type: ignore[no-untyped-call]
        fx = float(camera_matrix_np[0, 0])
        fy = float(camera_matrix_np[1, 1])
        cx = float(camera_matrix_np[0, 2])
        cy = float(camera_matrix_np[1, 2])
        N = obj.shape[0]
        rng = cp.random.RandomState(1234)
        best_inliers = -1
        _best_r, _best_t, best_mask = None, None, None

        for _ in range(iterations_count):
            idx = rng.choice(N, size=min_sample, replace=False)
            rvec, tvec = _solve_pnp_cuda_kernel(obj[idx], img[idx], camera_matrix_np)
            R = _rodrigues(cp.asarray(rvec.flatten()))
            Xc = obj @ R.T + cp.asarray(tvec.flatten())
            invZ = 1.0 / cp.clip(Xc[:, 2], 1e-6, None)
            u_hat = fx * Xc[:, 0] * invZ + cx
            v_hat = fy * Xc[:, 1] * invZ + cy
            err = cp.sqrt((img[:, 0] - u_hat) ** 2 + (img[:, 1] - v_hat) ** 2)
            mask = (err < reprojection_error).astype(cp.uint8)
            inliers = int(mask.sum())
            if inliers > best_inliers:
                best_inliers, _best_r, _best_t, best_mask = inliers, rvec, tvec, mask
                if inliers >= int(confidence * N):
                    break

        if best_inliers <= 0:
            return False, np.zeros((3, 1)), np.zeros((3, 1)), np.zeros((N,), dtype=np.uint8)
        in_idx = cp.nonzero(best_mask)[0]
        rvec, tvec = _solve_pnp_cuda_kernel(obj[in_idx], img[in_idx], camera_matrix_np)
        return True, rvec, tvec, cp.asnumpy(best_mask)


class _CudaTemplateTracker:
    def __init__(
        self,
        tmpl: cp.ndarray,
        scale_step: float = 1.05,
        lr: float = 0.1,
        search_radius: int = 16,
        x0: int = 0,
        y0: int = 0,
    ) -> None:
        self.tmpl = tmpl.astype(cp.float32)
        self.h, self.w = int(tmpl.shape[0]), int(tmpl.shape[1])
        self.scale_step = float(scale_step)
        self.lr = float(lr)
        self.search_radius = int(search_radius)
        # Cosine window
        wy = cp.hanning(self.h).astype(cp.float32)
        wx = cp.hanning(self.w).astype(cp.float32)
        self.window = wy[:, None] * wx[None, :]
        self.tmpl = self.tmpl * self.window
        self.y = int(y0)
        self.x = int(x0)

    def update(self, gray: cp.ndarray):  # type: ignore[no-untyped-def]
        H, W = int(gray.shape[0]), int(gray.shape[1])
        r = self.search_radius
        x0 = max(0, self.x - r)
        y0 = max(0, self.y - r)
        x1 = min(W, self.x + self.w + r)
        y1 = min(H, self.y + self.h + r)
        search = gray[y0:y1, x0:x1]
        if search.shape[0] < self.h or search.shape[1] < self.w:
            search = gray
            x0 = y0 = 0
        best = (self.x, self.y, self.w, self.h)
        best_score = -1e9
        for s in (1.0 / self.scale_step, 1.0, self.scale_step):
            th = max(1, round(self.h * s))
            tw = max(1, round(self.w * s))
            tmpl_s = _resize_bilinear_hwc_cuda(self.tmpl, th, tw)
            if tmpl_s.ndim == 3:
                tmpl_s = tmpl_s[..., 0]
            tmpl_s = tmpl_s.astype(cp.float32)
            tmpl_zm = tmpl_s - tmpl_s.mean()
            tmpl_energy = cp.sqrt(cp.sum(tmpl_zm * tmpl_zm)) + 1e-6
            # NCC via correlate2d and local std
            ones = cp.ones((th, tw), dtype=cp.float32)
            num = csignal.correlate2d(search, tmpl_zm, mode="valid")
            sumS = csignal.correlate2d(search, ones, mode="valid")
            sumS2 = csignal.correlate2d(search * search, ones, mode="valid")
            n = float(th * tw)
            meanS = sumS / n
            varS = cp.clip(sumS2 - n * meanS * meanS, 0.0, None)
            stdS = cp.sqrt(varS) + 1e-6
            res = num / (stdS * tmpl_energy)
            ij = cp.unravel_index(cp.argmax(res), res.shape)
            dy, dx = int(ij[0].get()), int(ij[1].get())
            score = float(res[ij].get())
            if score > best_score:
                best_score = score
                best = (x0 + dx, y0 + dy, tw, th)
        x, y, w, h = best
        patch = gray[y : y + h, x : x + w]
        if patch.shape[0] != self.h or patch.shape[1] != self.w:
            patch = _resize_bilinear_hwc_cuda(patch, self.h, self.w)
            if patch.ndim == 3:
                patch = patch[..., 0]
        patch = patch.astype(cp.float32) * self.window
        self.tmpl = (1.0 - self.lr) * self.tmpl + self.lr * patch
        self.x, self.y, self.w, self.h = x, y, w, h
        return x, y, w, h
