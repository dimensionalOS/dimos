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

"""Pinned upstream SONIC assets; policy pins shared with CC's integration."""

import os
from pathlib import Path

from dimos.constants import CACHE_DIR


def sonic_model_directory() -> Path:
    """Use the shared model cache, or the operator's explicit model directory."""
    return Path(os.environ.get("SONIC_MODEL_DIR", CACHE_DIR / "sonic"))


MODEL_REPOSITORY = "nvidia/GEAR-SONIC"
MODEL_REVISION = "6733128a3d8a523b1418b06bca3cdf61c8b0987f"
MODEL_FILES: dict[str, str] = {
    "planner_sonic.onnx": "39b553e197f62f077975ba38512bc04781a3fc37c2af7c6756e04629f760edea",
    "sonic_v1_1/model_encoder.onnx": "fb97de22819b2057b41459802128d91723d91a25f0ad73e7bfc41a9cf8365bae",
    "sonic_v1_1/model_decoder.onnx": "34bae8570d4a4421a5391a5c2befd745d4a02d182ec539e5f9da44c091c67509",
    "low_latency/model_encoder.onnx": "60be43157f57d812f38bdbb740a5de5d5d070e8840d9edc16f02a91a6d06255b",
    "low_latency/model_decoder.onnx": "c4ac2e74045e7cbfb568f15e6bf47ea7ce023df7a94322af50be223e0a628bab",
    "sonic_v1_1/observation_config.yaml": "4a67713b310932e50aca81f19188c8d76013148e98b15c8b5bbea995f12e59f0",
    "low_latency/observation_config.yaml": "582b9a273a3d69fbf49ae59b39295a3be2b4a295e195ef4cf674b5e2571c90ab",
}

MOTION_REVISION = "087f9ac01d46f6d8e4d0b73c01ae64799f292a38"
MOTION_FILES: dict[str, str] = {
    "dance_in_da_party_001__A464/body_quat.csv": "ee91908150c8f4391ba35a25995e8a0b6c6f00212ce425a33cb1ff9e05d5b2da",
    "dance_in_da_party_001__A464/joint_pos.csv": "17027b288d550a06701c4e1ca3759f76accdfdc78d7aceb5325579a4cb2567b6",
    "dance_in_da_party_001__A464/joint_vel.csv": "9fcb9531a144c5a39573fd765d251e733ec372c5499ea03cc9c450d203f5b451",
    "dance_in_da_party_001__A464_M/body_quat.csv": "d59594991ba416def87330c64b85adc3edff815fadb4d4bccbf9df2a2a0d56b7",
    "dance_in_da_party_001__A464_M/joint_pos.csv": "05f62f9bf79fa78ef777a528b07e376f8bf32a35a45135617fe1000f4076cdc4",
    "dance_in_da_party_001__A464_M/joint_vel.csv": "78c7af2ecc94998d3e2077384673cf4c232af8fb4d9327b9ee826f1748f0487b",
    "forward_lunge_R_001__A359_M/body_quat.csv": "65e322f25dc5b493d184dfc8dc420907385c255ee79ba839cc5544f9bb93ab4a",
    "forward_lunge_R_001__A359_M/joint_pos.csv": "ba1d5f70ef05ed9cd8abf5bb3dc94f3fab404069c6613eb6164926cf792b7929",
    "forward_lunge_R_001__A359_M/joint_vel.csv": "5a279b94233fc70c7cc14294db5549b94401a63d1dfec23b96bd42298207eff2",
    "macarena_001__A545/body_quat.csv": "b8b12818a64405764dee8035f84948a5d9db55332576065f79cd79962a99375d",
    "macarena_001__A545/joint_pos.csv": "c5d3346b84d18f654d41119f77b3c22158222a0ed8e0cfdd5d1e365cd2f928bc",
    "macarena_001__A545/joint_vel.csv": "a253beca302630400e4f94d92e2af7fb0d8492b13dc202f9f23941b35dfbde66",
    "macarena_001__A545_M/body_quat.csv": "1075cccb999d7257d60b65d35085793d6f2bb642f2b91cf1564a5e5b89b1373b",
    "macarena_001__A545_M/joint_pos.csv": "900c8ab465f6657d6eb126dd415f7573577fa9ae5457aebe5a842eeccb4600a0",
    "macarena_001__A545_M/joint_vel.csv": "7e6bc7a2f5c1ae24e6899973e01dd22329a4feb70fb0ff05e5c71c62001e6aa8",
    "neutral_kick_R_001__A543/body_quat.csv": "90803a0726495abea7af897e52f3324b8083e0b7c81cad1d1289dc742b3ece61",
    "neutral_kick_R_001__A543/joint_pos.csv": "8b4f673ad056dc7b2e3f2623d39d345b951bd7b9c0c50abc6eb50eb70d952654",
    "neutral_kick_R_001__A543/joint_vel.csv": "0e8dbc5c76804001bd6f0bae1e3614f4918cf9fff79c5c3f11202a98ed1a1e4b",
    "neutral_kick_R_001__A543_M/body_quat.csv": "d11a5a0519614c5bb9e0040c80f0ad3c019b7a568ec3eeaa66e2c3bc7eebb470",
    "neutral_kick_R_001__A543_M/joint_pos.csv": "b879921ad2f2701572deefee2b581f68879b7903e41e280e4d1a54acfb04dacf",
    "neutral_kick_R_001__A543_M/joint_vel.csv": "d8fc7f97da478c7c41d54c15364ab4929169833ec813cca756d61cbde0b60e90",
    "squat_001__A359/body_quat.csv": "b1e6ab7b81de682fed07720ee6d21ba99f02812631f262bfcf77f6c39e0c164b",
    "squat_001__A359/joint_pos.csv": "fd6b07ac5d66a3ffb78999eefc12f6934402b3103853e71d4667858dab088e8d",
    "squat_001__A359/joint_vel.csv": "246e8225dcdf410339bbaa6700375e92b217166de6e497c5168c4c4eb34dd38c",
    "tired_forward_lunge_R_001__A359_M/body_quat.csv": "a6432b76702d3c3e68582620a75119ee2b34dd420c611a406e804fd059e92e3f",
    "tired_forward_lunge_R_001__A359_M/joint_pos.csv": "0fdeda9a95cac3ed98b5d3593fe4b5df8f1f806e8c8dd25aab956cd63eb4569e",
    "tired_forward_lunge_R_001__A359_M/joint_vel.csv": "f4c0c7294edddb3f4c4a5fd23077a32c0a38e657c93cc6c902b1af1e3a671b89",
    "tired_one_leg_jumping_R_001__A359/body_quat.csv": "461928b2fc6fa9f5a87253614421a2fe80bfb0811f30090b67247ffe81ec8a49",
    "tired_one_leg_jumping_R_001__A359/joint_pos.csv": "3d4aa3b8fd5af3d42d9003706f24707453ce564191da355b9835a57527153b00",
    "tired_one_leg_jumping_R_001__A359/joint_vel.csv": "636f2a952c138f7af42a332530a334fdb0aed05a0ec9c481fc1803aa31b44cd5",
    "tired_one_leg_jumping_R_001__A359_M/body_quat.csv": "170208770667c8e2a8909e0bc2363a76e191eb29f470b33aaab420b61addc7a1",
    "tired_one_leg_jumping_R_001__A359_M/joint_pos.csv": "f94627ec643420221518fe7a3021e5d8bd159a3f6b535d8a72a2dfeecb752352",
    "tired_one_leg_jumping_R_001__A359_M/joint_vel.csv": "749c1291d548759e8fc55e66f6796dac82b06fcd114060f4a487a21fac28a938",
    "walking_quip_360_R_002__A428/body_quat.csv": "ac868ecb5abfab8251563bec2e0eb3c46ff9200874c914cc2f6f981220da0d62",
    "walking_quip_360_R_002__A428/joint_pos.csv": "49aa248f12d69304a928191e9942ad3236991dc82bbf001e0ced9442d409a42f",
    "walking_quip_360_R_002__A428/joint_vel.csv": "ad103e8e4bacbb057461305ccd6cfb6aecbe86179b4077afd7ffdce1ad659b1a",
    "walking_quip_360_R_002__A428_M/body_quat.csv": "28012cab8d4e08f214cdc22e88aa1422da96b4730683d8b3e6fe2617270cb238",
    "walking_quip_360_R_002__A428_M/joint_pos.csv": "3c6b44f81b193243d9c1a85a596fefa2e9f928d455df498675341e90925674f0",
    "walking_quip_360_R_002__A428_M/joint_vel.csv": "3a9d6d35b492a38ded99ef8cf86cfbee22e62da79354cf25df6e88ed4175f82a",
}
