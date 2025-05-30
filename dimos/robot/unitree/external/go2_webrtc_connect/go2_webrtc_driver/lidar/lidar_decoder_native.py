# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import numpy as np
import lz4.block

def decompress(compressed_data, decomp_size):
    decompressed = lz4.block.decompress(
        compressed_data,
        uncompressed_size=decomp_size
    )
    return decompressed

def bits_to_points(buf, origin, resolution=0.05):
    buf = np.frombuffer(bytearray(buf), dtype=np.uint8)
    nonzero_indices = np.nonzero(buf)[0]
    points = []

    for n in nonzero_indices:
        byte_value = buf[n]
        z = n // 0x800
        n_slice = n % 0x800
        y = n_slice // 0x10
        x_base = (n_slice % 0x10) * 8

        for bit_pos in range(8):
            if byte_value & (1 << (7 - bit_pos)):
                x = x_base + bit_pos
                points.append((x,y,z))

    return np.array(points) * resolution + origin

class LidarDecoder:
    def decode(self, compressed_data, data):
        def points():
            decompressed = decompress(compressed_data, data["src_size"])
            points = bits_to_points(decompressed, data["origin"], data["resolution"])
            return points

        return {
                "points": points(),
                # "raw": compressed_data,
        }
