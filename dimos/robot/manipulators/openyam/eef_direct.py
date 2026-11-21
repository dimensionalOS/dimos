# Copyright 2026 Dimensional Inc.
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

"""Frozen 50 Hz SuperReach EEF policy contract (noise-dr 11,500)."""
from dataclasses import dataclass
from hashlib import sha256
from multiprocessing import Pipe
from multiprocessing.connection import Connection
from pathlib import Path
import subprocess
import sys

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.robot.manipulators.openyam.policy.contract import PolicyContractError
from dimos.robot.manipulators.openyam.policy.native_actor import BoundedNativeActor
from dimos.robot.manipulators.openyam.policy.quest_reference import controller_matrix

ASSETS = Path(__file__).with_name('eef_direct_assets')
POLICY = ASSETS / 'policy_eef_direct_noise_dr_10ms_11.5k.onnx'
POLICY_HASH = '5de4bef7c67e85e3a3d175327dee75a7a0ee5cdb723457410c7cdd5c099ac1b0'
PLANT = ASSETS / 'tcp_fk.xml'
PLANT_HASH = '99a4f14707937dcd7fd49c84e27e7e6ac96c55fa051651eb6b6a1dfcd7648863'
DT = 0.02
JOINTS = tuple(f'yam_joint{i}' for i in range(1, 7))
KP = (50, 60, 50, 25, 15, 10)
KD = (3, 4, 3, 2, 1, 0.8)


def pinned_bytes(path, digest):
    data = path.read_bytes()
    if sha256(data).hexdigest() != digest:
        raise ValueError(f'Unexpected SHA-256 for {path.name}')
    return data


@dataclass(frozen=True)
class TcpGoal:
    index: int
    goal_position: np.ndarray
    goal_quaternion_wxyz: np.ndarray


class DirectTcpReference:
    """Map relative right-hand motion into the trained TCP frame, without IK."""

    def __init__(self):
        import mujoco

        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_string(pinned_bytes(PLANT, PLANT_HASH).decode())
        self.data = mujoco.MjData(self.model)
        self.site = self.model.site('arm_tcp_site').id
        self.joints = [self.model.joint(f'arm_joint_{i}') for i in range(1, 7)]
        self.indices = [int(j.qposadr[0]) for j in self.joints]
        self.bounds = np.stack([j.range for j in self.joints], axis=1)
        self.release()

    @property
    def anchored(self):
        return self._tcp_anchor is not None

    def release(self):
        self._controller_inverse = self._tcp_anchor = None

    def _controller(self, pose):
        try:
            result = controller_matrix(pose)
        except ValueError as error:
            raise PolicyContractError('Invalid Quest controller pose') from error
        if not np.isfinite(result).all():
            raise PolicyContractError('Nonfinite Quest controller pose')
        return result

    def anchor(self, q, pose, t_now):
        controller = self._controller(pose)
        self.data.qpos[self.indices] = q
        self.mj.mj_kinematics(self.model, self.data)
        tcp = np.eye(4)
        tcp[:3, 3] = self.data.site_xpos[self.site]
        tcp[:3, :3] = self.data.site_xmat[self.site].reshape(3, 3)
        self._controller_inverse = np.linalg.inv(controller)
        self._tcp_anchor = tcp
        return self._goal(0, tcp)

    def update(self, pose, t_now, *, tick_index):
        if not self.anchored:
            raise PolicyContractError('Missing Quest TCP anchor')
        return self._goal(tick_index, self._controller(pose) @ self._controller_inverse @ self._tcp_anchor)

    @staticmethod
    def _goal(index, tcp):
        xyzw = Rotation.from_matrix(tcp[:3, :3]).as_quat()
        quat = xyzw[[3, 0, 1, 2]]
        return TcpGoal(index, tcp[:3, 3].copy(), quat if quat[0] >= 0 else -quat)


class DirectHistory:
    def __init__(self, infer):
        self.infer = infer
        self.frames = np.empty((16, 19), dtype=np.float32)

    def reset(self, q, dq):
        self.tick = 0
        self.frames[:] = np.r_[q, dq, q, 0.0]
        self.frames[-1, 18] = 1.0

    def commit(self, q_des, q, dq):
        self.frames[:-1] = self.frames[1:].copy()
        self.frames[-1] = np.r_[q, dq, q_des, 1.0]
        self.tick += 1

    def command(self, goal):
        obs = np.empty((1, 311), dtype=np.float32)
        obs[0, :304] = self.frames.ravel()
        obs[0, 304:307] = goal.goal_position
        obs[0, 307:311] = goal.goal_quaternion_wxyz
        if not np.isfinite(obs).all():
            raise ValueError('Nonfinite EEF observation')
        result = self.infer(obs)
        if result.shape != (1, 6) or not np.isfinite(result).all():
            raise ValueError('Invalid EEF q_des')
        return result[0].copy()


class DirectActorProcess(BoundedNativeActor):
    """Established bounded subprocess transport, new ONNX endpoint and ABI."""

    def __init__(self):
        self.timeout_s = 0.020
        self.failed = self.closed = False
        self._pipe, child = Pipe()
        try:
            self._process = subprocess.Popen(
                [sys.executable, '-B', '-m', __name__, str(child.fileno())],
                pass_fds=(child.fileno(),), stdin=subprocess.DEVNULL,
            )
        except BaseException:
            self._pipe.close()
            raise
        finally:
            child.close()
        try:
            if not self._pipe.poll(30):
                raise TimeoutError('EEF actor startup deadline')
            ready = self._pipe.recv()
            if ready.get('ready') is not True:
                raise RuntimeError(f'EEF actor startup failed: {ready}')
        except BaseException:
            self.close()
            raise

    def close(self):
        if not self.closed:
            self.closed = True
            super().close()


def _serve(fd):
    import onnxruntime as ort

    with Connection(fd) as pipe:
        try:
            options = ort.SessionOptions()
            options.intra_op_num_threads = 1
            session = ort.InferenceSession(pinned_bytes(POLICY, POLICY_HASH), options,
                                           providers=['CPUExecutionProvider'])
            inputs = [(v.name, v.type, v.shape) for v in session.get_inputs()]
            outputs = [(v.name, v.type, v.shape) for v in session.get_outputs()]
            if inputs != [('obs', 'tensor(float)', [1, 311])] or outputs != [('q_des', 'tensor(float)', [1, 6])]:
                raise ValueError(f'Unexpected EEF ONNX ABI: {inputs=} {outputs=}')
            warm = DirectHistory(lambda x: session.run(['q_des'], {'obs': x})[0])
            warm.reset(np.array([0, 1.047, 1.047, 0, 0, 0]), np.zeros(6))
            for _ in range(3):
                warm.command(TcpGoal(0, np.array([0.3, 0, 0.3]), np.array([1, 0, 0, 0])))
            pipe.send({'ready': True})
            while True:
                obs = pipe.recv()
                pipe.send(session.run(['q_des'], {'obs': obs})[0])
        except EOFError:
            pass
        except Exception as error:
            pipe.send({'error': repr(error)})


if __name__ == '__main__':
    _serve(int(sys.argv[1]))
