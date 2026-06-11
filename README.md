[![CI Build colcon](https://github.com/MOLAorg/mola_mapper_2d/actions/workflows/build-ros.yml/badge.svg)](https://github.com/MOLAorg/mola_mapper_2d/actions/workflows/build-ros.yml)
[![CI clang-format](https://github.com/MOLAorg/mola_mapper_2d/actions/workflows/check-clang-format.yml/badge.svg)](https://github.com/MOLAorg/mola_mapper_2d/actions/workflows/check-clang-format.yml)
[![Docs](https://img.shields.io/badge/docs-latest-brightgreen.svg)](https://docs.mola-slam.org/latest/)
[![codecov](https://codecov.io/gh/MOLAorg/mola_mapper_2d/graph/badge.svg?token=PLACEHOLDER)](https://codecov.io/gh/MOLAorg/mola_mapper_2d)

# mola_mapper_2d

2D LiDAR Graph-SLAM backend component based on the MOLA and MRPT frameworks,
compatible with ROS 2.


| Distro | Develop Branch | Releases | Stable Release |
| ---    | ---            | ---      |  ---           |
| ROS2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mola_mapper_2d__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mola_mapper_2d__ubuntu_jammy_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mola_mapper_2d__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mola_mapper_2d__ubuntu_jammy_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Hbin_ujv8_uJv8__mola_mapper_2d__ubuntu_jammy_arm64__binary/badge/icon)](https://build.ros2.org/job/Hbin_ujv8_uJv8__mola_mapper_2d__ubuntu_jammy_arm64__binary/) | [![Version](https://img.shields.io/ros/v/humble/mola_mapper_2d)](https://index.ros.org/?search_packages=true&pkgs=mola_mapper_2d) |
| ROS 2 Jazzy (u24.04) | [![Build Status](https://build.ros2.org/job/Jdev__mola_mapper_2d__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mola_mapper_2d__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Jbin_uN64__mola_mapper_2d__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mola_mapper_2d__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Jbin_unv8_uNv8__mola_mapper_2d__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Jbin_unv8_uNv8__mola_mapper_2d__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/jazzy/mola_mapper_2d)](https://index.ros.org/?search_packages=true&pkgs=mola_mapper_2d) |
| ROS 2 Kilted (u24.04) | [![Build Status](https://build.ros2.org/job/Kdev__mola_mapper_2d__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__mola_mapper_2d__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Kbin_uN64__mola_mapper_2d__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mola_mapper_2d__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Kbin_unv8_uNv8__mola_mapper_2d__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Kbin_unv8_uNv8__mola_mapper_2d__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/kilted/mola_mapper_2d)](https://index.ros.org/?search_packages=true&pkgs=mola_mapper_2d) |
| ROS 2 Lyrical (u26.04) | [![Build Status](https://build.ros2.org/job/Ldev__mola_mapper_2d__ubuntu_resolute_amd64/badge/icon)](https://build.ros2.org/job/Ldev__mola_mapper_2d__ubuntu_resolute_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Lbin_uR64__mola_mapper_2d__ubuntu_resolute_amd64__binary/badge/icon)](https://build.ros2.org/job/Lbin_uR64__mola_mapper_2d__ubuntu_resolute_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Lbin_unv8_uRv8__mola_mapper_2d__ubuntu_resolute_arm64__binary/badge/icon)](https://build.ros2.org/job/Lbin_unv8_uRv8__mola_mapper_2d__ubuntu_resolute_arm64__binary/) | [![Version](https://img.shields.io/ros/v/lyrical/mola_mapper_2d)](https://index.ros.org/?search_packages=true&pkgs=mola_mapper_2d) |
| ROS 2 Rolling (u26.04) | [![Build Status](https://build.ros2.org/job/Rdev__mola_mapper_2d__ubuntu_resolute_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mola_mapper_2d__ubuntu_resolute_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Rbin_uR64__mola_mapper_2d__ubuntu_resolute_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uR64__mola_mapper_2d__ubuntu_resolute_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Rbin_unv8_uRv8__mola_mapper_2d__ubuntu_resolute_arm64__binary/badge/icon)](https://build.ros2.org/job/Rbin_unv8_uRv8__mola_mapper_2d__ubuntu_resolute_arm64__binary/) | [![Version](https://img.shields.io/ros/v/rolling/mola_mapper_2d)](https://index.ros.org/?search_packages=true&pkgs=mola_mapper_2d) |


## Overview

This repository provides a C++ library `mola_mapper_2d` implementing a complete 2D LiDAR 
SLAM system (Simultaneous Localization and Mapping) using pose graph optimization. 

The mapper uses:
- **GTSAM** for factor graph optimization
- **mp2p_icp** for scan-to-map registration and loop closure detection
- **MRPT** for common robotics types and utilities
- **MOLA framework** for system integration and component management

Sensor input is provided via MOLA observation interfaces, making it compatible with various 
sensor front-ends (odometry, LiDAR, etc.). ROS 2 example launch files are provided in 
[ros2-launches](ros2-launches/).

A CLI interface `mola-mapper-2d-cli` is also provided for running on offline datasets.

## Features

- **Graph-based SLAM**: Maintains a pose graph of keyframes with SE(3) constraints
- **Loop Closure Detection**: Automatic detection and correction of large-scale loops
- **Flexible ICP Pipelines**: Separate configurable pipelines for odometry and loop closure
- **Keyframe Management**: Intelligent keyframe selection and old-keyframe replacement
- **Robust Optimization**: Levenberg-Marquardt optimization with robust noise models
- **Multi-Sensor Support**: Handles 2D/3D LiDAR observations with flexible sensor labeling
- **Session Resumption**: Supports resuming SLAM sessions from saved states for multi-session mapping
- **Visualization Support**: Built-in 3D scene generation for debugging and visualization
- **Full State Serialization**: Complete state save/load for archival and analysis

## Contents

The repository provides:

- **Core Library**: `mola_mapper_2d` - The main 2D SLAM backend implementation
- **CLI Tool**: `mola-mapper-2d-cli` - Command-line interface for offline processing
- **Example Configurations**: YAML configuration templates for different scenarios
- **ROS 2 Bridges**: Integration components for ROS 2 (in separate ROS-specific package)

## Quick Start

### Build and Install

Refer to: https://docs.mola-slam.org/latest/#installing

```bash
# Standard MOLA installation (with ROS 2)
colcon build --packages-select mola_mapper_2d

# Or standalone (without ROS 2)
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
cmake --install build
```

### Basic Usage

```cpp
#include <mola_mapper_2d/Mapper2D.h>

// Create mapper instance
mola::Mapper2D mapper;

// Initialize from YAML configuration
mapper.initialize("config.yaml");

// Process observations in a loop
mapper.onObservation(observation);

// Get current estimates
auto pose = mapper.get_current_pose();
auto map = mapper.get_current_map();

// Save state for later resumption
auto state = mapper.get_state();
// ... serialize state ...

// Resume from saved state
mapper.resume_session(saved_pose);
```

## Architecture

### Core Components

#### `SlamMapperState`
Encapsulates the complete SLAM state:
- Factor graph (GTSAM)
- Keyframe observations and metric maps
- Odometry accumulation
- Keyframe connectivity graph

#### `Mapper2D`
Main SLAM backend implementation:
- Observation processing pipeline
- Relocalization/loop closure detection
- Pose graph optimization
- Keyframe management and replacement

### Processing Pipeline

1. **Relocalization Check**: Evaluates if incoming observation warrants processing
2. **Nearby Keyframe Search**: Finds candidate keyframes for ICP registration
3. **ICP Registration**: Runs ICP against nearby keyframes (odometry or loop closure)
4. **Localization Step**: Optimizes temporary graph to estimate current pose
5. **Keyframe Decision**: Decides whether to insert new keyframe or replace old ones
6. **Graph Optimization**: Full pose graph optimization with GTSAM

## Documentation and Tutorials

For comprehensive documentation, tutorials, and examples, see:
https://docs.mola-slam.org/

## Performance Characteristics

- **Computational Complexity**: O(n log n) for graph optimization where n = number of keyframes
- **Memory Usage**: Proportional to number of keyframes and point cloud density
- **Frequency**: Designed for real-time operation at 10-20 Hz typical LiDAR rates
- **Loop Closure**: Automatic detection with configurable topological distance threshold

## Dependencies

### Core Dependencies
- **MRPT** (≥ 2.0): Robotics types and utilities
- **GTSAM** (≥ 4.0): Factor graph optimization
- **mp2p_icp** (latest): Multi-scale scan registration
- **mola-yaml**: YAML configuration handling

### Optional Dependencies (ROS 2 Integration)
- **ROS 2** (Humble or newer)
- **geometry_msgs**, **sensor_msgs**: ROS message types

## Citation

If you use `mola_mapper_2d` in your research, please cite the MOLA framework publication:

```bibtex
@article{blanco2025mola_lo,
    author = {Blanco-Claraco, Jose Luis},
    title = {{A flexible framework for accurate LiDAR odometry, map manipulation, and localization}},
    journal = {The International Journal of Robotics Research},
    volume = {44},
    number = {9},
    pages = {1553--1599},
    year = {2025},
    doi = {10.1177/02783649251316881},
    URL = {https://doi.org/10.1177/02783649251316881},
    eprint = {2407.20465},
}
```

## Troubleshooting

### Poor ICP Quality
- Check sensor calibration and data quality
- Adjust `min_icp_quality_*` thresholds
- Verify ICP pipeline configuration (filters and parameters)
- Increase point cloud density if using downsampling

### Excessive Loop Closures
- Increase `min_topological_distance_for_loop_closure`
- Raise `min_icp_quality_loop_closure` threshold
- Reduce `max_icp_search_distance`

### Memory Issues
- Increase `max_distance_to_keep_keyframes` to remove old frames
- Enable point cloud downsampling in ICP filters
- Reduce `observations_decay_seconds` for visualization

### Divergent Optimization
- Check graph structure for inconsistent constraints
- Enable `debug_print_factor_graphs` for detailed inspection
- Reduce `icp_edge_robust_parameter` for weaker robustness (or vice versa)
- Verify odometry quality

## License

Copyright (C) 2024-2026 Jose Luis Blanco <jlblanco@ual.es>, University of Almeria

This package is released under the GNU GPL v3 license as open source, with the main 
intention of being useful for research and evaluation purposes.

Commercial licenses [available upon request](https://docs.mola-slam.org/latest/solutions.html).

## Contributing

Contributions are welcome! Please follow the MOLA project guidelines:
- Format code with `clang-format` (use the project's `.clang-format` file)
- Write comprehensive commit messages
- Add tests for new features
- Update documentation accordingly

For major changes, please open an issue first to discuss proposed changes.

Contributions require acceptance of the Contributor License Agreement (CLA).
