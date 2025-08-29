import time
import pyzed.sl as sl
import ogl_viewer.viewer as gl


def main():
    init = sl.InitParameters()
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP  # TODO: change in dimos
    init.depth_maximum_distance = 8.0

    zed = sl.Camera()

    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print("Camera Open : "+repr(status)+". Exit program.")
        exit()
    
    zed.get_camera_information()
    pose = sl.Pose()
    
    positional_tracking_parameters = sl.PositionalTrackingParameters()
    positional_tracking_parameters.set_floor_as_origin = True
    returned_state = zed.enable_positional_tracking(positional_tracking_parameters)
    if returned_state != sl.ERROR_CODE.SUCCESS:
        print("Enable Positional Tracking : "+repr(status)+". Exit program.")
        exit()
    
    spatial_mapping_parameters = sl.SpatialMappingParameters(
        resolution = sl.MAPPING_RESOLUTION.MEDIUM,
        mapping_range =  sl.MAPPING_RANGE.MEDIUM,
        max_memory_usage = 2048,
        save_texture = False,
        use_chunk_only = True,
        reverse_vertex_order = False,
        map_type = sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD
    )
    pymesh = sl.FusedPointCloud()

    tracking_state = sl.POSITIONAL_TRACKING_STATE.OFF
    mapping_state = sl.SPATIAL_MAPPING_STATE.NOT_ENABLED
    
    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.confidence_threshold = 50
    
    mapping_activated = False

    image = sl.Mat()  
    point_cloud = sl.Mat()
    pose = sl.Pose()

    viewer = gl.GLViewer()

    viewer.init(zed.get_camera_information().camera_configuration.calibration_parameters.left_cam, pymesh, 0)
    print("Press on 'Space' to enable / disable spatial mapping")
    print("Disable the spatial mapping after enabling it will output a .obj mesh file")
    last_call = time.time()
    while viewer.is_available():
        if zed.grab(runtime_parameters) <= sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            tracking_state = zed.get_position(pose)

            if mapping_activated:
                mapping_state = zed.get_spatial_mapping_state()
                duration = time.time() - last_call
                if duration > 0.5 and viewer.chunks_updated():
                    zed.request_spatial_map_async()
                    last_call = time.time()

                if zed.get_spatial_map_request_status_async() == sl.ERROR_CODE.SUCCESS:
                    zed.retrieve_spatial_map_async(pymesh)
                    viewer.update_chunks()

            change_state = viewer.update_view(image, pose.pose_data(), tracking_state, mapping_state)

            if change_state:
                if not mapping_activated:
                    init_pose = sl.Transform()
                    zed.reset_positional_tracking(init_pose)

                    spatial_mapping_parameters.resolution_meter = sl.SpatialMappingParameters().get_resolution_preset(sl.MAPPING_RESOLUTION.MEDIUM)
                    spatial_mapping_parameters.use_chunk_only = True
                    spatial_mapping_parameters.save_texture = False
                    spatial_mapping_parameters.map_type = sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD

                    zed.enable_spatial_mapping(spatial_mapping_parameters)

                    pymesh.clear()
                    viewer.clear_current_mesh()

                    last_call = time.time()

                    mapping_activated = True
                else:
                    zed.extract_whole_spatial_map(pymesh)

                    filepath = "mesh_gen_with_viewer.obj"
                    status = pymesh.save(filepath)
                    if status:
                        print("Mesh saved under " + filepath)
                    else:
                        print("Failed to save the mesh under " + filepath)
                    
                    mapping_state = sl.SPATIAL_MAPPING_STATE.NOT_ENABLED
                    mapping_activated = False
    
    image.free(memory_type=sl.MEM.CPU)
    pymesh.clear()
    zed.disable_spatial_mapping()
    zed.disable_positional_tracking()
    zed.close()
    pymesh.clear()
    image.free()
    point_cloud.free()
    zed.close()
   

if __name__ == "__main__":
    main()
