import time
import pyzed.sl as sl


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

    start_time = time.time()
    last_call = time.time()

    while True:
        time.sleep(0.02)

        time_elapsed = time.time() - start_time

        # After 20s quit.
        if time_elapsed > 15:
            break

        if zed.grab(runtime_parameters) > sl.ERROR_CODE.SUCCESS:
            continue

        print('grabbed')

        zed.retrieve_image(image, sl.VIEW.LEFT)
        tracking_state = zed.get_position(pose)
        print('tracking_state', tracking_state)

        # After 5s turn on mapping.
        if time_elapsed > 5 and not mapping_activated:
            print('turning mapping on')

            init_pose = sl.Transform()
            zed.reset_positional_tracking(init_pose)

            spatial_mapping_parameters.resolution_meter = sl.SpatialMappingParameters().get_resolution_preset(sl.MAPPING_RESOLUTION.MEDIUM)
            zed.enable_spatial_mapping(spatial_mapping_parameters)

            pymesh.clear()

            last_call = time.time()

            mapping_activated = True

        if mapping_activated:
            mapping_state = zed.get_spatial_mapping_state()
            print('mapping_state', mapping_state)
            duration = time.time() - last_call
            if duration > 0.5:
                print('requested spatial map')
                zed.request_spatial_map_async()
                last_call = time.time()

            if zed.get_spatial_map_request_status_async() == sl.ERROR_CODE.SUCCESS:
                print('received spatial map')
                zed.retrieve_spatial_map_async(pymesh)
            else:
                print('spatial map not received yet')
    
    # Saving the mesh.
    zed.extract_whole_spatial_map(pymesh)
    status = pymesh.save("mesh_gen_no_viewer.obj")
    print("saved mesh", status)
    
    # Turn everything off.
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
