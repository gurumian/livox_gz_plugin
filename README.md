# Livox Gazebo Plugin (`livox_gz_plugin`)

A **Gazebo Sim (Ignition)** System Plugin that simulates Livox LiDARs by bridging Gazebo's `gpu_lidar` data to `livox_ros_driver2/msg/CustomMsg`.

This plugin is designed to replace the legacy Gazebo Classic plugins for modern Gazebo Sim environments.

## Features
- **Direct Bridge**: Reads `gz::msgs::PointCloudPacked` directly from Gazebo's internal transport.
- **CustomMsg Output**: Publishes standard `livox_ros_driver2::msg::CustomMsg` used by algorithms like FAST_LIO.
- **Time Sync**: Correctly stamps ROS messages using Gazebo's simulation time (fixes TF errors).

## Dependencies
- ROS 2 (Jazzy/Humble)
- Gazebo Sim (Harmonic/Citadel/etc.)
- `livox_ros_driver2` (for message definitions)

## Usage

### 1. Build
```bash
colcon build --packages-select livox_gz_plugin
```

### 2. Environment Setup
Add the plugin directory to `GZ_SIM_SYSTEM_PLUGIN_PATH` so Gazebo can find the shared library.

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:$(ros2 pkg prefix livox_gz_plugin)/lib/livox_gz_plugin
```

### 3. URDF/SDF Configuration
Add the `<plugin>` tag inside your `<sensor>` element.

**Example (URDF/Xacro):**
```xml
<gazebo reference="base_scan">
  <sensor name="mid360" type="gpu_lidar">
    <topic>points</topic> <!-- Gazebo internal topic -->
    <update_rate>10</update_rate>
    
    <!-- Standard GpuLidar Params -->
    <lidar>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>0</min_angle>
          <max_angle>6.28318</max_angle>
        </horizontal>
        <vertical>
          <samples>100</samples>
          <resolution>1</resolution>
          <min_angle>-0.126</min_angle> <!-- -7.22 deg -->
          <max_angle>0.963</max_angle>  <!-- 55.22 deg -->
        </vertical>
      </scan>
    </lidar>

    <!-- Livox Plugin -->
    <plugin filename="livox_gz_plugin" name="livox_gz_plugin::LivoxPointsPlugin">
      <!-- Input: Gazebo internal topic (use absolute path recommended) -->
      <scan_topic>/points/points</scan_topic> 
      
      <!-- Output: ROS 2 Topic -->
      <ros_topic>livox/lidar</ros_topic>
      
      <!-- Frame ID for the ROS header -->
      <frame_id>base_scan</frame_id>
    </plugin>
  </sensor>
</gazebo>
```

### Parameters
| Parameter | Description | Default |
|-----------|-------------|---------|
| `scan_topic` | The Gazebo internal topic to subscribe to (e.g., `/points/points`). Check with `gz topic -l`. | (Required) |
| `ros_topic` | The ROS 2 topic to publish `CustomMsg` to. | `livox/lidar` |
| `frame_id` | The TF frame ID populated in the ROS message header. | `livox_frame` |
