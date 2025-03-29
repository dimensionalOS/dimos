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

import boto3
import os
from io import BytesIO
try:
    import open3d as o3d
except Exception as e:
    print(f"Open3D not importing, assuming to be running outside of docker. {e}")

class S3Utils:
    def __init__(self, bucket_name):
        self.s3 = boto3.client('s3')
        self.bucket_name = bucket_name

    def download_file(self, s3_key, local_path):
        try:
            self.s3.download_file(self.bucket_name, s3_key, local_path)
            print(f"Downloaded {s3_key} to {local_path}")
        except Exception as e:
            print(f"Error downloading {s3_key}: {e}")

    def upload_file(self, local_path, s3_key):
        try:
            self.s3.upload_file(local_path, self.bucket_name, s3_key)
            print(f"Uploaded {local_path} to {s3_key}")
        except Exception as e:
            print(f"Error uploading {local_path}: {e}")

    def save_pointcloud_to_s3(self, inlier_cloud, s3_key):

        try:
            temp_pcd_file = "/tmp/temp_pointcloud.pcd"
            o3d.io.write_point_cloud(temp_pcd_file, inlier_cloud)
            with open(temp_pcd_file, 'rb') as pcd_file:
                self.s3.put_object(Bucket=self.bucket_name, Key=s3_key, Body=pcd_file.read())
            os.remove(temp_pcd_file)
            print(f"Saved pointcloud to {s3_key}")
        except Exception as e:
            print(f"error downloading {s3_key}: {e}")

    def restore_pointcloud_from_s3(self, pointcloud_paths):
        restored_pointclouds = []

        for path in pointcloud_paths:
            # Download the point cloud file from S3 to memory
            pcd_obj = self.s3.get_object(Bucket=self.bucket_name, Key=path)
            pcd_data = pcd_obj['Body'].read()

            # Save the point cloud data to a temporary file
            temp_pcd_file = "/tmp/temp_pointcloud.pcd"
            with open(temp_pcd_file, 'wb') as f:
                f.write(pcd_data)

            # Read the point cloud from the temporary file
            pcd = o3d.io.read_point_cloud(temp_pcd_file)
            restored_pointclouds.append(pcd)

            # Remove the temporary file
            os.remove(temp_pcd_file)

        return restored_pointclouds
    @staticmethod
    def upload_text_file(bucket_name, local_path, s3_key):
        s3 = boto3.client('s3')
        try:
            with open(local_path, 'r') as file:
                content = file.read()

            # Ensure the s3_key includes the file name
            if not s3_key.endswith('/'):
                s3_key = s3_key + '/'

            # Extract the file name from the local_path
            file_name = local_path.split('/')[-1]
            full_s3_key = s3_key + file_name

            s3.put_object(Bucket=bucket_name, Key=full_s3_key, Body=content)
            print(f"Uploaded text file {local_path} to {full_s3_key}")
        except Exception as e:
            print(f"Error uploading text file {local_path}: {e}")