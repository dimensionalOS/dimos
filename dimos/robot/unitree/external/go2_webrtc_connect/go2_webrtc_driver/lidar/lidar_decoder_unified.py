from .lidar_decoder_libvoxel import LidarDecoder as LibVoxelDecoder
from .lidar_decoder_native import LidarDecoder as NativeDecoder


class UnifiedLidarDecoder:
    def __init__(self, decoder_type="libvoxel"):
        """
        Initialize the UnifiedLidarDecoder with the specified decoder type.

        :param decoder_type: The type of decoder to use ("libvoxel" or "native").
                             Defaults to "libvoxel".
        """
        if decoder_type == "libvoxel":
            self.decoder = LibVoxelDecoder()
            self.decoder_name = "LibVoxelDecoder"
        elif decoder_type == "native":
            self.decoder = NativeDecoder()
            self.decoder_name = "NativeDecoder"
        else:
            raise ValueError("Invalid decoder type. Choose 'libvoxel' or 'native'.")

    def decode(self, compressed_data, metadata):
        """
        Decode the compressed data using the selected decoder.

        :param compressed_data: The compressed data to decode.
        :param metadata: Metadata required for decoding (e.g., origin, resolution).
        :return: Decoded result from the selected decoder.
        """
        return self.decoder.decode(compressed_data, metadata)

    def get_decoder_name(self):
        """
        Get the name of the currently selected decoder.

        :return: Name of the decoder (e.g., "LibVoxelDecoder" or "NativeDecoder").
        """
        return self.decoder_name