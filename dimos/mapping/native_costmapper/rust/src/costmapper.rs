use dimos_module::native_config;
use lcm_msgs::nav_msgs::OccupancyGrid;
use lcm_msgs::sensor_msgs::PointCloud2;

#[native_config]
pub struct Config {
    pub algo: String,
    pub config: serde_json::Value,
    pub initial_safe_radius_meters: f64,
}

pub fn calculate_costmap(_msg: &PointCloud2, _config: &Config) -> OccupancyGrid {
    todo!()
}

pub fn apply_initial_safe_radius(_grid: &mut OccupancyGrid, _radius_meters: f64) {
    todo!()
}
