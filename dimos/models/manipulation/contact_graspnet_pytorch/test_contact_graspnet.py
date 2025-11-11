import glob
import os

import numpy as np
import pytest


def is_manipulation_installed() -> bool:
    """Check if the manipulation extras are installed."""
    try:
        import contact_graspnet_pytorch
        return True
    except ImportError:
        return False

@pytest.mark.skipif(not is_manipulation_installed(),
                   reason="This test requires 'pip install .[manipulation]' to be run")
def test_contact_graspnet_inference() -> None:
    """Test contact graspnet inference with local regions and filter grasps."""
    # Skip test if manipulation dependencies not installed
    if not is_manipulation_installed():
        pytest.skip("contact_graspnet_pytorch not installed. Run 'pip install .[manipulation]' first.")
        return

    try:
        from contact_graspnet_pytorch import config_utils

        from dimos.models.manipulation.contact_graspnet_pytorch.inference import inference
        from dimos.utils.data import get_data
    except ImportError:
        pytest.skip("Required modules could not be imported. Make sure you have run 'pip install .[manipulation]'.")
        return

    # Test data path - use the default test data path
    test_data_path = os.path.join(get_data("models_contact_graspnet"), "test_data/0.npy")

    # Check if test data exists
    test_files = glob.glob(test_data_path)
    if not test_files:
        pytest.fail(f"No test data found at {test_data_path}")

    # Load config with default values
    ckpt_dir = 'models_contact_graspnet'
    global_config = config_utils.load_config(ckpt_dir, batch_size=1)

    # Run inference function with the same params as the command line
    result_files_before = glob.glob('results/predictions_*.npz')

    inference(
        global_config=global_config,
        ckpt_dir=ckpt_dir,
        input_paths=test_data_path,
        local_regions=True,
        filter_grasps=True,
        skip_border_objects=False,
        z_range=[0.2, 1.8],
        forward_passes=1,
        K=None
    )

    # Verify results were created
    result_files_after = glob.glob('results/predictions_*.npz')
    assert len(result_files_after) >= len(result_files_before), "No result files were generated"

    # Load at least one result file and verify it contains expected data
    if result_files_after:
        latest_result = sorted(result_files_after)[-1]
        result_data = np.load(latest_result, allow_pickle=True)
        expected_keys = ['pc_full', 'pred_grasps_cam', 'scores', 'contact_pts', 'pc_colors']
        for key in expected_keys:
            assert key in result_data.files, f"Expected key '{key}' not found in results"
