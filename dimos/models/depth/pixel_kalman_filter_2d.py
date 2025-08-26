import cupy as cp
import cupyx.scipy.ndimage as cnd

class PixelKalmanFilter2D:
    def __init__(self, shape, dt=1.0, process_var=1e-3, meas_var=1e-2, init_var=1.0):
        """
        Pixel-wise Kalman filter with depth + velocity states.
        
        Args:
            shape: (H, W) of the depth map
            dt: time delta between frames
            process_var: Q, process noise variance
            meas_var: R, measurement noise variance
            init_var: initial error covariance
        """
        self.shape = shape
        self.H, self.W = shape
        self.dt = cp.float32(dt)
        
        # State: [depth, velocity] per pixel
        self.x = cp.zeros((self.H, self.W, 2), dtype=cp.float32)
        
        # Covariance: 2x2 per pixel
        self.P = cp.zeros((self.H, self.W, 2, 2), dtype=cp.float32)
        self.P[..., 0, 0] = init_var
        self.P[..., 1, 1] = init_var
        
        # Noise
        self.Q = cp.array([[process_var, 0],
                           [0, process_var]], dtype=cp.float32)
        self.R = cp.float32(meas_var)
        
        # Constant matrices
        self.F = cp.array([[1, self.dt],
                           [0, 1]], dtype=cp.float32)
        self.Hm = cp.array([[1, 0]], dtype=cp.float32)

    def update(self, z):
        """
        Update filter with new depth measurement z (CuPy array, shape = HxW).
        Returns filtered depth map (CuPy array).
        """
        # Prediction
        x_pred = self.x @ self.F.T  # (H, W, 2)
        P_pred = self.F @ self.P @ self.F.T + self.Q  # (H, W, 2, 2)
        
        # Innovation
        z = z[..., None]  # (H, W, 1)
        y = z - (x_pred @ self.Hm.T)  # (H, W, 1)
        S = (self.Hm @ P_pred @ self.Hm.T) + self.R  # (H, W, 1, 1)
        
        # Kalman gain
        K = (P_pred @ self.Hm.T) / S  # (H, W, 2, 1)
        
        # Update state
        self.x = x_pred + (K * y)[:, :, :, 0]  # broadcasting (H, W, 2)
        
        # Update covariance
        I = cp.eye(2, dtype=cp.float32)
        self.P = (I - K @ self.Hm) @ P_pred

        # Return filtered depth (state[0])
        return self.x[..., 0]

def bilateral_filter(depth, sigma_spatial=3, sigma_intensity=0.1):
    """
    Approximate bilateral filter using CuPy.
    Convenience function. Use if smoothing post-Kalman necessary
    """
    smoothed = cnd.gaussian_filter(depth, sigma=sigma_spatial)
    return (depth * (1 - sigma_intensity)) + (smoothed * sigma_intensity)


