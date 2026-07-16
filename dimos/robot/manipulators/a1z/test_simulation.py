import itertools
import math

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")
from mujoco import MjData, MjModel

pytestmark = pytest.mark.mujoco

from dimos.robot.manipulators.a1z.simulation import A1Z_SCENE_PATH
from dimos.robot.manipulators.a1z.simulation.generate import PAD_CENTER_OFFSET, PAD_HALF_THICKNESS


def test_a1z_scene_compiles_and_has_stable_contract() -> None:
    model = mujoco.MjModel.from_xml_path(str(A1Z_SCENE_PATH))
    a1z_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "a1z")
    assert model.body_quat[a1z_id].tolist() == pytest.approx([1.0, 0.0, 0.0, 0.0])
    table_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "table")
    robot_base = np.array([0.42, 0.0])
    native_table_xy = np.array([0.0, 0.0])
    expected_table_xy = robot_base - (native_table_xy - robot_base)
    assert model.body_pos[table_id, :2].tolist() == pytest.approx(expected_table_xy.tolist())
    assert model.body_pos[table_id, 2] == pytest.approx(0.38462332)
    assert model.body_quat[table_id].tolist() == pytest.approx([0.0, 0.0, 0.0, 1.0])
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]
    assert names[:6] == [f"arm_joint{i}" for i in range(1, 7)]
    assert names[6:8] == ["gripper_finger_left_joint", "gripper_finger_rIght_joint"]
    assert model.jnt_range[6].tolist() == pytest.approx([0.0, 0.015])
    assert model.jnt_axis[6].tolist() == pytest.approx([0.0, 1.0, 0.0])
    assert model.jnt_axis[7].tolist() == pytest.approx([0.0, -1.0, 0.0])
    actuator_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)
    ]
    assert actuator_names == [f"arm_joint{i}_motor" for i in range(1, 7)] + ["gripper_motor"]
    assert model.nu == 7
    assert model.actuator_gainprm[:6, 0].tolist() == pytest.approx([500.0] * 6)
    assert model.actuator_biasprm[:6, 2].tolist() == pytest.approx([-45.0] * 6)
    for actual, expected in zip(
        model.actuator_ctrlrange[:6],
        [
            [-2.094, 2.094],
            [0.0, 3.142],
            [-3.142, 0.0],
            [-1.309, 1.309],
            [-1.484, 1.484],
            [-2.007, 2.007],
        ],
        strict=True,
    ):
        assert actual.tolist() == pytest.approx(expected)
    for actual, expected in zip(
        model.actuator_forcerange[:6],
        [[-20.0, 20.0], [-25.0, 25.0], [-20.0, 20.0], [-9.0, 9.0], [-3.5, 3.5], [-3.5, 3.5]],
        strict=True,
    ):
        assert actual.tolist() == pytest.approx(expected)
    assert model.actuator_gainprm[6, 0] == pytest.approx(200.0)
    assert model.actuator_biasprm[6, 2] == pytest.approx(-8.0)
    assert model.actuator_forcerange[6].tolist() == pytest.approx([-20.0, 20.0])
    assert model.actuator_forcelimited[6]
    tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, "finger_split")
    assert tendon_id >= 0
    assert model.actuator_trntype[6] == mujoco.mjtTrn.mjTRN_TENDON
    assert model.actuator_trnid[6, 0] == tendon_id
    assert model.tendon_length0[tendon_id] == pytest.approx(0.0)
    assert model.eq_obj1id[0] == 7
    assert model.eq_obj2id[0] == 6
    assert model.eq_data[0, :5].tolist() == pytest.approx([0.0, 1.0, 0.0, 0.0, 0.0])
    assert model.actuator_ctrlrange[6].tolist() == pytest.approx([0.0, 0.015])
    assert model.jnt_margin[6] == pytest.approx(0.0001)
    assert model.jnt_margin[7] == pytest.approx(0.0001)
    assert model.dof_damping[model.jnt_dofadr[6]] == pytest.approx(20.0)
    assert model.dof_damping[model.jnt_dofadr[7]] == pytest.approx(20.0)
    assert model.dof_armature[model.jnt_dofadr[6]] == pytest.approx(0.05)
    assert model.eq_solref[0].tolist() == pytest.approx([0.005, 1.0])
    pillar_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pillar")
    native_pillar_xy = np.array([0.139000448, -0.001852509])
    expected_pillar_xy = robot_base - (native_pillar_xy - robot_base)
    assert model.body_pos[pillar_body, :2].tolist() == pytest.approx(expected_pillar_xy.tolist())
    assert model.body_pos[pillar_body].tolist() == pytest.approx(
        [0.700999552, 0.001852509, 0.44462332]
    )
    assert model.body_mass[pillar_body] == pytest.approx(0.05)
    assert model.opt.noslip_iterations == 20
    pillar_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "pillar_geom")
    assert model.geom_type[pillar_geom] == mujoco.mjtGeom.mjGEOM_CYLINDER
    assert model.geom_size[pillar_geom].tolist() == pytest.approx([0.01, 0.03, 0.0])
    assert model.geom_contype[pillar_geom] == 4
    assert model.geom_conaffinity[pillar_geom] == 7
    table_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "tabletop")
    assert model.geom_contype[table_geom] == 1
    assert model.geom_conaffinity[table_geom] == 7
    pair_id = next(
        pair_id
        for pair_id in range(model.npair)
        if {model.pair_geom1[pair_id], model.pair_geom2[pair_id]} == {pillar_geom, table_geom}
    )
    assert model.pair_dim[pair_id] == 6
    assert model.pair_friction[pair_id].tolist() == pytest.approx([0.8, 0.8, 0.005, 0.005, 0.005])
    assert model.pair_margin[pair_id] == pytest.approx(0.0001)
    assert model.pair_solref[pair_id].tolist() == pytest.approx([0.005, 1.0])
    assert model.pair_solimp[pair_id, :3].tolist() == pytest.approx([0.99, 0.999, 0.0001])
    pad_mesh_ids = set()
    for pad_name in ("left_pad", "right_pad"):
        pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, pad_name)
        pad_mesh_ids.add(int(model.geom_dataid[pad_id]))
        assert model.geom_type[pad_id] == mujoco.mjtGeom.mjGEOM_MESH
        assert model.geom_contype[pad_id] == 2
        assert model.geom_conaffinity[pad_id] == 5
        assert model.geom_condim[pad_id] == 4
        assert model.geom_margin[pad_id] == pytest.approx(0.001)
        assert model.geom_solref[pad_id].tolist() == pytest.approx([0.02, 1.0])
        assert model.geom_matid[pad_id] >= 0
    assert len(pad_mesh_ids) == 1
    pad_mesh_id = next(iter(pad_mesh_ids))
    pad_vertices = model.mesh_vert[
        model.mesh_vertadr[pad_mesh_id] : model.mesh_vertadr[pad_mesh_id]
        + model.mesh_vertnum[pad_mesh_id]
    ]
    assert sorted(np.ptp(pad_vertices, axis=0).tolist()) == pytest.approx(
        sorted([0.0015, 0.01, 0.014]), abs=1e-6
    )
    for geom_id in range(model.ngeom):
        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
        if geom_name is not None and geom_name.endswith("_visual"):
            assert model.geom_contype[geom_id] == 0
            assert model.geom_conaffinity[geom_id] == 0
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist_camera") >= 0
    assert model.vis.global_.offwidth == 640
    assert model.vis.global_.offheight == 480
    fps_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_NUMERIC, "wrist_camera_fps")
    assert model.numeric_data[model.numeric_adr[fps_id]] == pytest.approx(30.0)
    data = mujoco.MjData(model)
    for value in (0.0, 0.0075, 0.015):
        data.ctrl[6] = value
        for _ in range(20):
            mujoco.mj_step(model, data)
        assert data.qpos[6] == pytest.approx(data.qpos[7], abs=2e-3)


def test_a1z_gripper_inward_pad_aperture_increases_to_safe_width() -> None:
    model, data = _home_data()
    data.qpos[:6] = GRASP_ARM_TARGET
    pad_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        for name in ("left_pad", "right_pad")
    ]
    gaps = []
    for value in (0.0, 0.015):
        data.qpos[6:8] = value
        mujoco.mj_forward(model, data)
        centers = np.array([data.geom_xpos[pad_id] for pad_id in pad_ids])
        direction = centers[1] - centers[0]
        direction /= np.linalg.norm(direction)
        half_widths = []
        for pad_id in pad_ids:
            local_direction = data.geom_xmat[pad_id].reshape(3, 3).T @ direction
            half_widths.append(float(np.sum(model.geom_size[pad_id] * np.abs(local_direction))))
        gaps.append(
            float(np.linalg.norm(direction * np.linalg.norm(centers[1] - centers[0])))
            - sum(half_widths)
        )

    assert gaps[1] > gaps[0]
    assert gaps[1] <= 0.050001


def test_a1z_compiled_pad_frames_are_opposing_and_symmetric() -> None:
    model, data = _home_data()
    data.qpos[:6] = GRASP_ARM_TARGET
    data.qpos[6:8] = 0.015
    mujoco.mj_forward(model, data)
    left_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_pad")
    right_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_pad")
    pillar_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "pillar_geom")
    left_frame = data.geom_xmat[left_id].reshape(3, 3)
    right_frame = data.geom_xmat[right_id].reshape(3, 3)
    left_y, right_y = left_frame[:, 1], right_frame[:, 1]
    assert np.dot(left_y, -right_y) >= 0.995
    assert np.dot(left_y, right_y) <= -0.995
    assert (
        max(
            np.degrees(np.arccos(np.clip(abs(frame[2, 2]), 0.0, 1.0)))
            for frame in (left_frame, right_frame)
        )
        <= 5.0
    )
    assert abs(data.geom_xpos[right_id, 2] - data.geom_xpos[left_id, 2]) <= 0.0001
    assert (
        abs(
            np.linalg.norm(data.geom_xpos[left_id] - data.geom_xpos[pillar_id])
            - np.linalg.norm(data.geom_xpos[right_id] - data.geom_xpos[pillar_id])
        )
        <= 0.00025
    )
    for pad_id in (left_id, right_id):
        assert model.geom_matid[pad_id] >= 0


def test_a1z_pad_embedding_and_open_clearance_are_physical() -> None:
    model, data = _home_data()
    data.qpos[:6] = GRASP_ARM_TARGET
    data.qpos[6:8] = 0.015
    mujoco.mj_forward(model, data)
    pillar_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "pillar_geom")
    pillar_center = data.geom_xpos[pillar_id]
    pad_margin = max(
        model.geom_margin[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, pad_name)]
        for pad_name in ("left_pad", "right_pad")
    )
    assert PAD_HALF_THICKNESS - PAD_CENTER_OFFSET == pytest.approx(0.0005, abs=0.0001)
    assert PAD_HALF_THICKNESS + PAD_CENTER_OFFSET == pytest.approx(0.001, abs=0.0001)
    data.qpos[6:8] = 0.015
    mujoco.mj_forward(model, data)
    pillar_center = data.geom_xpos[pillar_id]
    for pad_name in ("left_pad", "right_pad"):
        pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, pad_name)
        radial_gap = min(
            np.linalg.norm((vertex - pillar_center)[:2]) - model.geom_size[pillar_id, 0]
            for vertex in (
                model.mesh_vert[
                    model.mesh_vertadr[model.geom_dataid[pad_id]] : model.mesh_vertadr[
                        model.geom_dataid[pad_id]
                    ]
                    + model.mesh_vertnum[model.geom_dataid[pad_id]]
                ]
                @ data.geom_xmat[pad_id].reshape(3, 3).T
                + data.geom_xpos[pad_id]
            )
        )
        assert radial_gap > pad_margin + 0.0001
    assert not any(
        frozenset(
            (
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(data.contact[i].geom1)),
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(data.contact[i].geom2)),
            )
        )
        == frozenset(("left_pad", "pillar_geom"))
        or frozenset(
            (
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(data.contact[i].geom1)),
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(data.contact[i].geom2)),
            )
        )
        == frozenset(("right_pad", "pillar_geom"))
        for i in range(data.ncon)
    )


def test_a1z_stepped_actuator_opens_laterally_without_vertical_drift() -> None:
    model, data = _home_data()
    mujoco.mj_forward(model, data)
    data.ctrl[:6] = data.qpos[:6]
    data.ctrl[6] = 0.015
    left_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_pad")
    right_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_pad")
    pillar_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pillar")
    table_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "table")
    table_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "tabletop")
    pad_midpoint = (data.geom_xpos[left_id] + data.geom_xpos[right_id]) / 2.0
    assert np.linalg.norm(data.xpos[pillar_id][:2] - pad_midpoint[:2]) < 0.02
    assert data.xpos[pillar_id][2] == pytest.approx(0.44462332)
    assert data.xpos[pillar_id][2] - model.geom_size[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "pillar_geom")
    ][1] == pytest.approx(data.xpos[table_id][2] + model.geom_size[table_geom][2])
    separations = []
    vertical_deltas = []
    for _ in range(400):
        mujoco.mj_step(model, data)
        left = data.geom_xpos[left_id]
        right = data.geom_xpos[right_id]
        separations.append(float(right[1] - left[1]))
        vertical_deltas.append(abs(float(right[2] - left[2])))

    assert all(next_sep <= sep + 1e-8 for sep, next_sep in itertools.pairwise(separations))
    assert separations[-1] < separations[0] - 0.015
    assert max(vertical_deltas) < 0.03


def _contact_pairs(model: MjModel, data: MjData) -> set[frozenset[str]]:
    pairs = set()
    for index in range(data.ncon):
        names = frozenset(
            (
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(data.contact[index].geom1)),
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(data.contact[index].geom2)),
            )
        )
        pairs.add(names)
    return pairs


def test_a1z_contact_masks_keep_pads_off_table_and_on_pillar() -> None:
    model, data = _home_data()
    mujoco.mj_forward(model, data)
    table_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "tabletop")
    pillar_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "pillar_geom")
    for pad_name in ("left_pad", "right_pad"):
        pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, pad_name)
        assert model.geom_contype[pad_id] & model.geom_conaffinity[table_id]
        assert model.geom_contype[pad_id] & model.geom_conaffinity[pillar_id]


GRASP_ARM_TARGET = [
    0.08004403,
    0.98357235,
    -0.27377253,
    0.08290665,
    -0.10518632,
    0.06622248,
]
SETTLED_ARM_STATE = [0.08004403, 0.98690963, -0.26047159, 0.08581191, -0.10508347, 0.06622232]
SETTLE_COMMAND = [0.08004403, 0.98023507, -0.28707347, 0.08000139, -0.10518632, 0.06622248]
GRASP_PILLAR_POS = [0.700999552, 0.001852509, 0.44462332]
ACQUISITION_OFFSETS = (
    pytest.param(0.0, 0.0, id="nominal"),
    pytest.param(0.25e-3, 0.0, id="jaw_plus"),
    pytest.param(-0.25e-3, 0.0, id="jaw_minus"),
    pytest.param(0.0, 0.25e-3, id="tangent_plus"),
    pytest.param(0.0, -0.25e-3, id="tangent_minus"),
)


@pytest.mark.parametrize("jaw_offset,tangent_offset", ACQUISITION_OFFSETS)
def test_a1z_settled_approach_acquires_free_pillar(
    jaw_offset: float, tangent_offset: float
) -> None:
    model, data = _home_data()
    left_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_pad")
    right_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_pad")
    data.qpos[:6] = [0.0, 0.7, -1.2, 0.5, 0.0, 0.0]
    data.qpos[6:8] = 0.015
    nominal_pillar = data.qpos[8:11].copy()
    data.qpos[:6] = GRASP_ARM_TARGET
    mujoco.mj_forward(model, data)
    jaw = data.geom_xpos[right_id] - data.geom_xpos[left_id]
    jaw[2] = 0.0
    jaw /= np.linalg.norm(jaw)
    tangent = np.array([-jaw[1], jaw[0], 0.0])
    data.qpos[:6] = [0.0, 0.7, -1.2, 0.5, 0.0, 0.0]
    data.qpos[8:11] = nominal_pillar + jaw_offset * jaw + tangent_offset * tangent
    data.qvel[8:15] = 0.0
    mujoco.mj_forward(model, data)
    data.ctrl[:6] = SETTLE_COMMAND
    data.ctrl[6] = 0.015
    for _ in range(1000):
        mujoco.mj_step(model, data)
    midpoint = (data.geom_xpos[left_id] + data.geom_xpos[right_id]) / 2.0
    midpoint_velocity = []
    for _ in range(20):
        previous = midpoint.copy()
        mujoco.mj_step(model, data)
        midpoint = (data.geom_xpos[left_id] + data.geom_xpos[right_id]) / 2.0
        midpoint_velocity.append(float(np.linalg.norm(midpoint - previous) / model.opt.timestep))
    assert max(midpoint_velocity) < 0.01
    initial = data.qpos[8:11].copy()
    initial_quat = data.qpos[11:15].copy()
    first_contact: dict[str, int] = {}
    bilateral_samples = 0
    force_samples: list[tuple[float, float]] = []
    contact_height_deltas: list[float] = []
    for step in range(2000):
        data.ctrl[6] = 0.0
        mujoco.mj_step(model, data)
        pad_forces: dict[str, float] = {}
        pad_contacts: dict[str, np.ndarray] = {}
        for index in range(data.ncon):
            geom1 = mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_GEOM, int(data.contact[index].geom1)
            )
            geom2 = mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_GEOM, int(data.contact[index].geom2)
            )
            pad_name = geom1 if geom1 in {"left_pad", "right_pad"} else geom2
            if pad_name in {"left_pad", "right_pad"} and "pillar_geom" in {geom1, geom2}:
                first_contact.setdefault(pad_name, step)
                contact_force = np.zeros(6)
                mujoco.mj_contactForce(model, data, index, contact_force)
                pad_forces[pad_name] = pad_forces.get(pad_name, 0.0) + abs(float(contact_force[0]))
                pad_contacts[pad_name] = data.contact[index].pos.copy()
        if step >= 100 and {"left_pad", "right_pad"}.issubset(pad_forces):
            bilateral_samples += 1
            force_samples.append((pad_forces["left_pad"], pad_forces["right_pad"]))
            contact_height_deltas.append(
                abs(data.geom_xpos[left_id, 2] - data.geom_xpos[right_id, 2])
            )
    assert abs(first_contact["left_pad"] - first_contact["right_pad"]) * model.opt.timestep <= 0.05
    assert bilateral_samples / 1900.0 > 0.95
    assert np.linalg.norm(data.qpos[8:11] - initial) < 0.002
    total_forces = [left + right for left, right in force_samples]
    imbalances = [abs(left - right) / ((left + right) / 2.0) for left, right in force_samples]
    assert min(total_forces) >= 1.2
    assert max(total_forces) <= 5.0
    assert max(imbalances) <= 0.2
    assert max(contact_height_deltas) <= 0.00025
    tilt_deg = 2.0 * math.degrees(
        math.acos(min(1.0, abs(float(np.dot(initial_quat, data.qpos[11:15])))))
    )
    print(
        f"free-pillar acquisition: occupancy={bilateral_samples / 1900.0:.3f} "
        f"max_imbalance={max(imbalances):.3f} "
        f"p95={np.percentile(imbalances, 95):.3f} mean={np.mean(imbalances):.3f} "
        f"final={imbalances[-1]:.3f} "
        f"force_range=({min(total_forces):.3f}, {max(total_forces):.3f}) "
        f"translation={np.linalg.norm(data.qpos[8:11] - initial):.6f} "
        f"tilt_deg={tilt_deg:.3f} height_skew={max(contact_height_deltas):.6f}"
    )


def test_a1z_open_pad_frames_are_symmetric_and_nonpenetrating() -> None:
    model, data = _home_data()
    data.qpos[:6] = GRASP_ARM_TARGET
    data.qpos[6:8] = 0.015
    mujoco.mj_forward(model, data)
    left_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_pad")
    right_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_pad")
    pillar_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "pillar_geom")
    midpoint = (data.geom_xpos[left_id] + data.geom_xpos[right_id]) / 2.0
    assert midpoint.tolist() == pytest.approx(data.geom_xpos[pillar_id].tolist(), abs=2e-3)
    for pad_name, visual_name in (
        ("left_pad", "gripper_finger_left_link_visual"),
        ("right_pad", "gripper_finger_rIght_link_visual"),
    ):
        pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, pad_name)
        visual_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, visual_name)
        mesh_id = model.geom_dataid[visual_id]
        vertices = model.mesh_vert[
            model.mesh_vertadr[mesh_id] : model.mesh_vertadr[mesh_id] + model.mesh_vertnum[mesh_id]
        ]
        world_vertices = (
            vertices @ data.geom_xmat[visual_id].reshape(3, 3).T + data.geom_xpos[visual_id]
        )
        nearest_visual_offset = np.min(
            np.linalg.norm(world_vertices - data.geom_xpos[pad_id], axis=1)
        )
        assert nearest_visual_offset <= 0.012
    pad_pillar_pairs = {
        frozenset(
            (
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(data.contact[i].geom1)),
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(data.contact[i].geom2)),
            )
        )
        for i in range(data.ncon)
    }
    assert frozenset(("left_pad", "pillar_geom")) not in pad_pillar_pairs
    assert frozenset(("right_pad", "pillar_geom")) not in pad_pillar_pairs
    center_gap = float(np.linalg.norm(data.geom_xpos[left_id] - data.geom_xpos[pillar_id]))
    center_gap = min(
        center_gap, float(np.linalg.norm(data.geom_xpos[right_id] - data.geom_xpos[pillar_id]))
    )
    clearance = center_gap - model.geom_size[left_id, 0] - model.geom_size[pillar_id, 0]
    assert clearance > 0.0


@pytest.mark.parametrize(
    "lateral_offset,tilt", ((0.0, 0.0), (0.0005, math.radians(1.0))), ids=("nominal", "tilted")
)
def test_a1z_table_drop_without_arm_contact_settles_on_table(
    lateral_offset: float, tilt: float
) -> None:
    model, data = _home_data()
    pillar_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pillar")
    pillar_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "pillar_geom")
    table_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "tabletop")
    for geom_id in range(model.ngeom):
        if model.geom_bodyid[geom_id] != pillar_body and geom_id not in {table_geom}:
            model.geom_contype[geom_id] = 0
            model.geom_conaffinity[geom_id] = 0
    table_top = model.body_pos[model.geom_bodyid[table_geom], 2] + model.geom_size[table_geom, 2]
    settled_z = model.body_pos[pillar_body, 2]
    data.qpos[8:11] = [data.qpos[8] + lateral_offset, data.qpos[9], settled_z + 0.0015]
    data.qpos[11:15] = [math.cos(tilt / 2.0), math.sin(tilt / 2.0), 0.0, 0.0]
    data.qvel[8:15] = 0.0
    mujoco.mj_forward(model, data)
    bottom_history: list[float] = []
    velocity_history: list[float] = []
    angular_velocity_history: list[float] = []
    table_forces: list[float] = []
    impact_seen = False
    rebound_heights: list[float] = []
    for _ in range(5000):
        mujoco.mj_step(model, data)
        bottom = float(data.geom_xpos[pillar_geom, 2] - model.geom_size[pillar_geom, 1])
        bottom_history.append(bottom)
        velocity_history.append(abs(float(data.qvel[10])))
        angular_velocity_history.append(float(np.linalg.norm(data.qvel[11:14])))
        normal_force = 0.0
        for index in range(data.ncon):
            pair = {int(data.contact[index].geom1), int(data.contact[index].geom2)}
            if pair == {pillar_geom, table_geom}:
                force = np.zeros(6)
                mujoco.mj_contactForce(model, data, index, force)
                normal_force += abs(float(force[0]))
        table_forces.append(normal_force)
        if normal_force > 0.01:
            impact_seen = True
        if impact_seen:
            rebound_heights.append(float(data.geom_xpos[pillar_geom, 2]))
    settled_bottom = float(np.mean(bottom_history[-100:]))
    settled_z_velocity = max(velocity_history[-100:])
    settled_angular_velocity = max(angular_velocity_history[-100:])
    settled_normal_force = float(np.mean(table_forces[-100:]))
    rebound = max(rebound_heights) - settled_z
    penetration = max(0.0, table_top - min(bottom_history))
    print(
        f"table drop: penetration={penetration:.6f} rebound={rebound:.6f} "
        f"settled_bottom={settled_bottom:.6f} settled_z_velocity={settled_z_velocity:.6f} "
        f"settled_angular_velocity={settled_angular_velocity:.6f} "
        f"table_normal_force={settled_normal_force:.6f}"
    )
    assert penetration <= 0.0002
    assert rebound <= 0.0002
    assert settled_z_velocity <= 0.001
    assert settled_angular_velocity <= 0.001
    assert settled_normal_force >= 0.1


def test_a1z_screenshot_arm_target_keeps_fingers_bounded_without_projection() -> None:
    model, data = _home_data()
    data.ctrl[:6] = [
        0.523899814140345,
        2.819045762646406,
        -0.7047955612496017,
        -0.7194075766046305,
        -0.5931064663834829,
        1.4994435298205953,
    ]
    data.ctrl[6] = 0.0075
    for _ in range(20):
        mujoco.mj_step(model, data)
        assert all(math.isfinite(float(value)) for value in data.qpos)
        assert all(math.isfinite(float(value)) for value in data.qvel)
        assert -0.01 <= data.qpos[6] <= 0.02
        assert -0.01 <= data.qpos[7] <= 0.02
        assert abs(float(data.qpos[6] - data.qpos[7])) < 0.02


def _home_data() -> tuple[MjModel, MjData]:
    model = mujoco.MjModel.from_xml_path(str(A1Z_SCENE_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.ctrl[:6] = data.qpos[:6]
    return model, data


def _assert_safe_gripper_sample(data: MjData, target: float) -> None:
    assert all(math.isfinite(float(value)) for value in data.qpos)
    assert all(math.isfinite(float(value)) for value in data.qvel)
    assert -0.005 <= data.qpos[6] <= 0.02
    assert -0.005 <= data.qpos[7] <= 0.02
    assert abs(float(data.qpos[6] - target)) <= 0.0152
    assert abs(float(data.qpos[7] - target)) <= 0.0152
    assert abs(float(data.qpos[6] - data.qpos[7])) <= 0.005
    assert abs(float(data.actuator_force[6])) <= 20.001


def test_a1z_gripper_low_high_endpoint_transitions_are_safe() -> None:
    model = mujoco.MjModel.from_xml_path(str(A1Z_SCENE_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.ctrl[:6] = data.qpos[:6]
    transition_times = []

    for target in (0.015, 0.0):
        data.ctrl[:6] = data.qpos[:6]
        data.ctrl[6] = target
        reached = None
        for step in range(1500):
            mujoco.mj_step(model, data)
            _assert_safe_gripper_sample(data, target)
            if reached is None and (
                (target > 0 and data.qpos[6] >= 0.0135) or (target == 0 and data.qpos[6] <= 0.0015)
            ):
                reached = step
        assert reached is not None
        assert reached < 500
        transition_times.append(reached * model.opt.timestep)

    assert transition_times[0] <= 1.0
    assert transition_times[1] <= 1.0


def test_a1z_gripper_holds_high_endpoint_during_arm_motion() -> None:
    model, data = _home_data()
    home = [float(value) for value in data.qpos[:6]]
    data.ctrl[6] = 0.015
    for _ in range(500):
        data.ctrl[:6] = home
        mujoco.mj_step(model, data)
        _assert_safe_gripper_sample(data, 0.015)
    for step in range(1500):
        time = step * model.opt.timestep
        ramp = min(1.0, max(0.0, (step - 500) / 500))
        scale = 1.0 + 0.5 * ramp * ramp * (3.0 - 2.0 * ramp)
        data.ctrl[:6] = [
            home[0] + scale * 0.12 * math.sin(0.8 * time),
            home[1] + scale * 0.08 * math.sin(0.6 * time),
            home[2] + scale * 0.06 * math.sin(0.7 * time),
            home[3] + scale * 0.10 * math.sin(0.5 * time),
            home[4] + scale * 0.12 * math.sin(0.9 * time),
            home[5] + scale * 0.08 * math.sin(0.4 * time),
        ]
        mujoco.mj_step(model, data)
        _assert_safe_gripper_sample(data, 0.015)


def test_a1z_arm_position_hold_and_small_command() -> None:
    model = mujoco.MjModel.from_xml_path(str(A1Z_SCENE_PATH))
    data = mujoco.MjData(model)
    home = [0.0, 0.7, -1.2, 0.5, 0.0, 0.0]
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.ctrl[:6] = home

    for _ in range(500):
        mujoco.mj_step(model, data)

    home_drift = max(abs(float(data.qpos[i]) - home[i]) for i in range(6))
    assert home_drift < 0.02

    commanded = home[1] + 0.1
    data.ctrl[1] = commanded
    for _ in range(500):
        mujoco.mj_step(model, data)

    assert abs(float(data.qpos[1]) - commanded) < 0.01


def test_a1z_compiled_directional_support_map_diagnostic() -> None:
    """Measure compiled visual support without creating or moving collision geometry."""
    model, data = _home_data()
    data.qpos[:6] = GRASP_ARM_TARGET
    data.qpos[6:8] = 0.015
    mujoco.mj_forward(model, data)
    pillar_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "pillar_geom")
    pillar = data.geom_xpos[pillar_id]
    results: list[tuple[str, int, float, float, float]] = []
    for finger in ("gripper_finger_left_link", "gripper_finger_rIght_link"):
        visual_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{finger}_visual")
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, finger)
        mesh_id = int(model.geom_dataid[visual_id])
        vertices = model.mesh_vert[
            model.mesh_vertadr[mesh_id] : model.mesh_vertadr[mesh_id] + model.mesh_vertnum[mesh_id]
        ]
        geom_rotation = data.geom_xmat[visual_id].reshape(3, 3)
        geom_position = data.geom_xpos[visual_id]
        body_rotation = data.xmat[body_id].reshape(3, 3)
        inward = body_rotation @ np.array([0.0, 1.0 if "left" in finger else -1.0, 0.0])
        vertical = np.array([0.0, 0.0, 1.0])
        vertical -= inward * np.dot(vertical, inward)
        vertical /= np.linalg.norm(vertical)
        tangent = np.cross(inward, vertical)
        cells: dict[tuple[int, int], tuple[float, np.ndarray]] = {}
        face_start = int(model.mesh_faceadr[mesh_id])
        face_stop = face_start + int(model.mesh_facenum[mesh_id])
        for face in model.mesh_face[face_start:face_stop]:
            triangle = vertices[face.astype(int)]
            subdivisions = max(
                1,
                math.ceil(
                    max(
                        np.linalg.norm(triangle[1] - triangle[0]),
                        np.linalg.norm(triangle[2] - triangle[1]),
                        np.linalg.norm(triangle[0] - triangle[2]),
                    )
                    / 0.001
                ),
            )
            for i in range(subdivisions + 1):
                for j in range(subdivisions + 1 - i):
                    local = (
                        triangle[0] * i + triangle[1] * j + triangle[2] * (subdivisions - i - j)
                    ) / subdivisions
                    world = geom_position + geom_rotation @ local
                    tangent_offset = float(np.dot(world - pillar, tangent))
                    vertical_offset = float(np.dot(world - pillar, vertical))
                    if abs(tangent_offset) > 0.012 or abs(vertical_offset) > 0.008:
                        continue
                    key = (round(tangent_offset / 0.001), round(vertical_offset / 0.001))
                    depth = float(np.dot(world, inward))
                    previous = cells.get(key)
                    if previous is None or (
                        depth < previous[0] if "left" in finger else depth > previous[0]
                    ):
                        cells[key] = (depth, world)
        assert cells
        tangent_span = (max(key[0] for key in cells) - min(key[0] for key in cells)) * 0.001
        vertical_span = (max(key[1] for key in cells) - min(key[1] for key in cells)) * 0.001
        depth_span = max(value[0] for value in cells.values()) - min(
            value[0] for value in cells.values()
        )
        row_gaps: list[int] = []
        for row in range(min(key[1] for key in cells), max(key[1] for key in cells) + 1):
            run = 0
            for column in range(min(key[0] for key in cells), max(key[0] for key in cells) + 1):
                run = run + 1 if (column, row) not in cells else 0
                row_gaps.append(run)
        max_hole = max(row_gaps or [0]) * 0.001
        results.append((finger, len(cells), tangent_span, vertical_span, max_hole))
        print(
            f"{finger}: cells={len(cells)} tangent={tangent_span * 1000:.1f}mm "
            f"vertical={vertical_span * 1000:.1f}mm depth={depth_span * 1000:.2f}mm "
            f"max_hole={max_hole * 1000:.1f}mm"
        )
    assert results[0][1] > 0 and results[1][1] > 0
