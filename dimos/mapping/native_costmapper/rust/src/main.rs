use dimos_module::{Input, Module, Output};
use dimos_native_costmapper::costmapper::Config;
use lcm_msgs::nav_msgs::OccupancyGrid;
use lcm_msgs::sensor_msgs::PointCloud2;

#[derive(Module)]
struct CostMapper {
    #[input(decode = PointCloud2::decode, handler = on_global_map)]
    global_map: Input<PointCloud2>,

    #[input(decode = PointCloud2::decode, handler = on_merged_map)]
    merged_map: Input<PointCloud2>,

    #[output(encode = OccupancyGrid::encode)]
    global_costmap: Output<OccupancyGrid>,

    #[config]
    config: Config,
}

impl CostMapper {
    async fn on_global_map(&mut self, _msg: PointCloud2) {
        todo!()
    }

    async fn on_merged_map(&mut self, _msg: PointCloud2) {
        todo!()
    }
}

#[tokio::main]
async fn main() {
    todo!()
}
