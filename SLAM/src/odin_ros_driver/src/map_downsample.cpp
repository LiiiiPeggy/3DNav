#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// ################################
// C++: downsample Odin PLY map and save as PCD
// ################################
int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "Usage: map_downsample <input.ply> [output.pcd] [leaf_size]" << std::endl;
    std::cerr << "  output.pcd defaults to <input> with .ply -> .pcd" << std::endl;
    std::cerr << "  leaf_size defaults to 0.05 m (VoxelGrid, same for x/y/z)" << std::endl;
    return 1;
  }

  const std::string input_path = argv[1];
  std::string output_path = (argc >= 3) ? argv[2] : std::string();
  double leaf_size = (argc >= 4) ? std::stod(argv[3]) : 0.05;

  if (output_path.empty()) {
    output_path = input_path;
    if (output_path.size() >= 4 &&
        output_path.compare(output_path.size() - 4, 4, ".ply") == 0) {
      output_path.replace(output_path.size() - 4, 4, ".pcd");
    } else {
      output_path += ".pcd";
    }
  }

  if (leaf_size <= 0.0) {
    std::cerr << "Error: leaf_size must be > 0 (got " << leaf_size << ")" << std::endl;
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcl::io::loadPLYFile<pcl::PointXYZ>(input_path, *cloud) < 0) {
    std::cerr << "Error: failed to load PLY: " << input_path << std::endl;
    return 1;
  }
  if (cloud->empty()) {
    std::cerr << "Error: PLY contains no points: " << input_path << std::endl;
    return 1;
  }

  auto t0 = std::chrono::steady_clock::now();
  pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel.filter(*down);
  auto t1 = std::chrono::steady_clock::now();
  double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  if (pcl::io::savePCDFileBinary(output_path, *down) < 0) {
    std::cerr << "Error: failed to save PCD: " << output_path << std::endl;
    return 1;
  }

  std::error_code ec;
  auto out_bytes = std::filesystem::file_size(output_path, ec);
  std::cout << "[map_downsample] " << input_path << " (" << cloud->size() << " pts)"
            << " -> " << output_path << " (" << down->size() << " pts)"
            << ", leaf=" << leaf_size << " m, " << ms << " ms"
            << ", out_size=" << (ec ? 0 : out_bytes / (1024 * 1024)) << " MiB" << std::endl;
  return 0;
}
