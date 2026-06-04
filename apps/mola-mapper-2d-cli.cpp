/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/

/**
 * @file   mola-mapper-2d-cli.cpp
 * @brief  main() for the cli app running 2D Graph-SLAM for offline datasets.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 28, 2026
 */

// Mapper:
#include <mola_mapper_2d/Mapper2D.h>

// Libraries:
#include <mola_kernel/MinimalModuleContainer.h>
#include <mola_kernel/interfaces/OfflineDatasetSource.h>
#include <mola_kernel/pretty_print_exception.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/progress.h>
#include <CLI/CLI.hpp>

#include <memory>

#if defined(HAVE_MOLA_SE_SIMPLE)
#include <mola_state_estimation_simple/StateEstimationSimple.h>
#endif

#if defined(HAVE_MOLA_INPUT_RAWLOG)
#include <mola_input_rawlog/RawlogDataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
#include <mola_input_rosbag2/Rosbag2Dataset.h>
#endif

#include <csignal>  // sigaction
#include <cstdlib>
#include <iostream>
#include <string>

struct Cli
{
  CLI::App app{"mola-mapper-2d-cli"};

  std::string argYAML;
  std::string arg_verbosity_level;
  std::string arg_plugins;
  std::string arg_stateEstimatorClass;
  std::string arg_stateEstimatorParams;
  std::string arg_outTwist;
  std::string arg_outSimpleMap;
  int arg_firstN{0};
  int arg_skipFirstN{0};
  std::string arg_lidarLabel;
  std::string arg_imuLabel;
  std::string arg_baseLinkName{"base_link"};

#if defined(HAVE_MOLA_INPUT_RAWLOG)
  std::string argRawlog;
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG2)
  std::string argRosbag2;
#endif

  void setup()
  {
    app.add_option("-c,--config", argYAML, "Input pipeline YAML config file (required) (*.yml)")->required();
    app.add_option("-v,--verbosity", arg_verbosity_level, "Verbosity level: ERROR|WARN|INFO|DEBUG {Default: INFO}");
    app.add_option("-l,--load-plugins", arg_plugins, "One or more {comma separated} *.so files to load as plugins");
    app.add_option("--state-estimator", arg_stateEstimatorClass, "The C++ class name of the state estimator to use");
    app.add_option("--state-estimator-param-file", arg_stateEstimatorParams, "Path to YAML parameters file to configure the state estimator.");
    app.add_option("--output-twist", arg_outTwist, "Save the estimated twist as a TXT file");
    app.add_option("--output-simplemap", arg_outSimpleMap, "Enables building and saving the simplemap for the mapping session");
    app.add_option("--only-first-n", arg_firstN, "Run for the first N steps only {0=default, not used}");
    app.add_option("--skip-first-n", arg_skipFirstN, "Skip the first N dataset entries {0=default, not used}");
    app.add_option("--lidar-sensor-label", arg_lidarLabel, "Supersedes lidar_sensor_labels in the pipeline; sensor label/topic name for LIDAR data");
    app.add_option("--imu-sensor-label", arg_imuLabel, "Supersedes imu_sensor_label in the pipeline; sensor label/topic name for IMU data");
    app.add_option("--base-link-frame-id", arg_baseLinkName, "Only for rosbag input: /tf frame_id used as reference frame for the vehicle");

#if defined(HAVE_MOLA_INPUT_RAWLOG)
    app.add_option("--input-rawlog", argRawlog, "INPUT DATASET: rawlog. Input dataset in rawlog format {*.rawlog}");
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG2)
    app.add_option("--input-rosbag2", argRosbag2, "INPUT DATASET: rosbag2. Input dataset in rosbag2 format {*.mcap}");
#endif
  }
};  // end struct "Cli"

namespace
{
#if defined(HAVE_MOLA_INPUT_RAWLOG)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_rawlog(
  const std::string & rawlogFile, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::RawlogDataset>();
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(
    mola::parse_yaml(
      mrpt::format(
        R""""(
    params:
      rawlog_filename: '%s'
      read_all_first: true
)"""",
        rawlogFile.c_str())));

  o->initialize(cfg);

  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_rosbag2(
  Cli & cli, const std::string & rosbag2file, const mrpt::system::VerbosityLevel logLevel)
{
  ASSERTMSG_(
    !cli.arg_lidarLabel.empty(),
    "Using a rosbag2 as input requires telling what is the lidar topic "
    "with --lidar-sensor-label <TOPIC_NAME>");

  auto o = std::make_shared<mola::Rosbag2Dataset>();
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(
    mola::parse_yaml(
      mrpt::format(
        R""""(
    params:
      rosbag_filename: '%s'
      base_link_frame_id: '%s'
      sensors:
        - topic: '%s'
          type: CObservation2DRangeScan
          # If present, this will override whatever /tf tells about the sensor pose:
          fixed_sensor_pose: "${LIDAR_POSE_X|0} ${LIDAR_POSE_Y|0} ${LIDAR_POSE_Z|0} ${LIDAR_POSE_YAW|0} ${LIDAR_POSE_PITCH|0} ${LIDAR_POSE_ROLL|0}"  # 'x y z yaw_deg pitch_deg roll_deg'
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_LIDAR_POSE|false}
        - topic: ${MOLA_GNSS_TOPIC|'/gps'}
          sensorLabel: 'gps'
          type: CObservationGPS
          fixed_sensor_pose: "${GPS_POSE_X|0} ${GPS_POSE_Y|0} ${GPS_POSE_Z|0} ${GPS_POSE_YAW|0} ${GPS_POSE_PITCH|0} ${GPS_POSE_ROLL|0}"  # 'x y z yaw_deg pitch_deg roll_deg'
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_GNSS_POSE|false}
        - topic: '%s'
          type: CObservationIMU
          # If present, this will override whatever /tf tells about the sensor pose:
          fixed_sensor_pose: "${IMU_POSE_X|0} ${IMU_POSE_Y|0} ${IMU_POSE_Z|0} ${IMU_POSE_YAW|0} ${IMU_POSE_PITCH|0} ${IMU_POSE_ROLL|0}" # 'x y z yaw_deg pitch_deg roll_deg''
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_IMU_POSE|false}
)"""",
        rosbag2file.c_str(), cli.arg_baseLinkName.c_str(),
        cli.arg_lidarLabel.c_str(), cli.arg_imuLabel.c_str())));

  o->initialize(cfg);

  return o;
}
#endif

void mola_signal_handler(int s);
void mola_install_signal_handler();

void mola_signal_handler(int s)
{
  std::cerr << "Caught signal " << s << ". Shutting down..."
            << "\n";
  exit(0);  // NOLINT
}

void mola_install_signal_handler()
{
  struct sigaction sigIntHandler{};

  sigIntHandler.sa_handler = &mola_signal_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, nullptr);
}

int main_odometry(Cli & cli)  // NOLINT
{
  // Declare main LO module:
  // ------------------------------------------
  auto mapper = mola::Mapper2D::Create();

  // Declare state estimator module:
  // ------------------------------------------
  mola::NavStateFilter::Ptr stateEstimator;
  if (!cli.arg_stateEstimatorClass.empty()) {
    const auto sClass = cli.arg_stateEstimatorClass;
    auto o = mrpt::rtti::classFactory(sClass);
    ASSERTMSG_(
      o, mrpt::format(
           "Apparently unknown class name: '%s' (missing plugin .so file?)", sClass.c_str()));
    stateEstimator = std::dynamic_pointer_cast<mola::NavStateFilter>(o);
    ASSERTMSG_(
      stateEstimator,
      mrpt::format(
        "Class '%s' does not implemented the expected interface mola::NavStateFilter",
        sClass.c_str()));
  }

#if defined(HAVE_MOLA_SE_SIMPLE)
  // Default?
  if (!stateEstimator) {
    stateEstimator = mola::state_estimation_simple::StateEstimationSimple::Create();
  }
#endif

  ASSERTMSG_(
    stateEstimator,
    "Either provide an explicit --state-estimator flag or build against "
    "mola::state_estimation_simple");

  // Cast to the interface that accepts raw sensor data:
  auto stateEstimatorAsRawConsumer =
    std::dynamic_pointer_cast<mola::RawDataConsumer>(stateEstimator);
  if (!stateEstimatorAsRawConsumer) {
    std::cerr << "[Warning] The state estimator '" << stateEstimator->GetRuntimeClass()->className
              << "' does not implement the mola::RawDataConsumer interface, so it will not receive "
                 "raw sensor data.\n";
  }

  if (!cli.arg_stateEstimatorParams.empty()) {
    const auto seParamsFile = cli.arg_stateEstimatorParams;
    auto seParams = mrpt::containers::yaml::FromFile(seParamsFile);
    stateEstimator->initialize(mola::parse_yaml(seParams));
  }

  // Make both modules discoverables to each other:
  // -------------------------------------------------
  const mola::MinimalModuleContainer moduleContainer = {{mapper, stateEstimator}};

  // Logging level:
  mrpt::system::VerbosityLevel logLevel = mapper->getMinLoggingLevel();
  if (!cli.arg_verbosity_level.empty()) {
    using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
    logLevel = vl::name2value(cli.arg_verbosity_level);
    mapper->setVerbosityLevel(logLevel);
    stateEstimator->setVerbosityLevel(logLevel);
  }

  // Add a logger hook to detect visible messages to the terminal
  // and avoid overwriting them with the CLI progress bar:
  bool liodom_emitted_log = false;
  std::mutex liodom_emitted_log_mtx;
  const auto mark_emitted_log = [&]() {
    auto lck = mrpt::lockHelper(liodom_emitted_log_mtx);
    liodom_emitted_log = true;
  };
  const auto has_emitted_log = [&]() -> bool {
    auto lck = mrpt::lockHelper(liodom_emitted_log_mtx);
    return liodom_emitted_log;
  };
  const auto unmark_emitted_log = [&]() {
    auto lck = mrpt::lockHelper(liodom_emitted_log_mtx);
    liodom_emitted_log = false;
  };
  mapper->mrpt::system::COutputLogger::logRegisterCallback(
    [&](
      [[maybe_unused]] std::string_view msg, const mrpt::system::VerbosityLevel level,
      [[maybe_unused]] std::string_view loggerName,
      [[maybe_unused]] const mrpt::Clock::time_point timestamp) {
      if (level < mapper->getMinLoggingLevel()) {
        return;
      }
      mark_emitted_log();
    });

  // Initialize mapper:
  const auto file_yml = cli.argYAML;
  const auto cfg = mola::load_yaml_file(file_yml);

  mapper->initialize(cfg);

  // Select dataset input:
  std::shared_ptr<mola::OfflineDatasetSource> dataset;

#if defined(HAVE_MOLA_INPUT_RAWLOG)
  if (!cli.argRawlog.empty()) {
    dataset = dataset_from_rawlog(cli.argRawlog, logLevel);
  } else
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG2)
  if (!cli.argRosbag2.empty()) {
    dataset = dataset_from_rosbag2(cli, cli.argRosbag2, logLevel);
  } else
#endif
  {
    THROW_EXCEPTION(
      "At least one of the dataset input CLI flags must be defined. "
      "Use --help.");
  }
  ASSERT_(dataset);

  // Optional output twist:
  std::optional<mrpt::poses::CPose3DInterpolator> outTwist;
  if (!cli.arg_outTwist.empty()) {
    outTwist.emplace();
  }

  const double tStart = mrpt::Clock::nowDouble();

  size_t lastDatasetEntry = dataset->datasetSize();
  size_t firstDatasetEntry = 0;

  if (cli.arg_skipFirstN > 0) {
    firstDatasetEntry = cli.arg_skipFirstN;
  }

  if (cli.arg_firstN > 0) {
    lastDatasetEntry = firstDatasetEntry + cli.arg_firstN;
  }

  mrpt::keep_min(lastDatasetEntry, dataset->datasetSize());

  std::cout << "\n";  // Needed for the VT100 codes below.

  // Run:
  for (size_t i = firstDatasetEntry; i < lastDatasetEntry; i++) {
    // Get observations from the dataset:
    using mrpt::obs::CObservation2DRangeScan;
    using mrpt::obs::CObservationGPS;
    using mrpt::obs::CObservationIMU;
    using mrpt::obs::CObservationOdometry;
    using mrpt::obs::CObservationPointCloud;

    const auto sf = dataset->datasetGetObservations(i);
    ASSERT_(sf);

    mrpt::obs::CObservation::Ptr obs;
    if (!obs) {
      obs = sf->getObservationByClass<CObservationPointCloud>();
    }
    if (!obs) {
      obs = sf->getObservationByClass<CObservation2DRangeScan>();
    }
    if (!obs) {
      obs = sf->getObservationByClass<CObservationGPS>();
    }
    if (!obs) {
      obs = sf->getObservationByClass<CObservationOdometry>();
    }
    if (!obs) {
      obs = sf->getObservationByClass<CObservationIMU>();
    }
    if (!obs) {
      continue;
    }

    // Send it to the odometry pipeline & the state estimator:
    if (stateEstimatorAsRawConsumer) {
      stateEstimatorAsRawConsumer->onNewObservation(obs);
    }

    mapper->onNewObservation(obs);

    // Show stats:
    static int cnt = 0;
    if (cnt++ % 100 == 0) {
      cnt = 0;
      const size_t N = (dataset->datasetSize() - 1);
      const double pc = static_cast<double>(i) / static_cast<double>(N);

      const double tNow = mrpt::Clock::nowDouble();
      const double ETA = pc > 0 ? (tNow - tStart) * (1.0 / pc - 1) : .0;
      const double totalTime = ETA + (tNow - tStart);

      // VT100 codes: cursor up and clear line
      if (!has_emitted_log()) {
        std::cout << "\033[A\33[2KT\r";
      }
      unmark_emitted_log();

      const auto lastPose = mapper->get_current_pose();

      std::cout << mrpt::system::progress(pc, 30)
                << mrpt::format(
                     " %6zu/%6zu (%.02f%%) ETA=%s/T=%s | Pose=%s\n", i, N, 100 * pc,
                     mrpt::system::formatTimeInterval(ETA).c_str(),
                     mrpt::system::formatTimeInterval(totalTime).c_str(),
                     lastPose.asString().c_str());
      std::cout.flush();
    }
  }

  if (!cli.arg_outSimpleMap.empty()) {
    const auto fil = cli.arg_outSimpleMap;

    auto sm = mapper->get_current_map();

    std::cout << "\nSaving reconstructed map with " << sm.size() << " keyframes to: " << fil
              << std::endl;  // NOLINT(performance-avoid-endl)

    sm.saveToFile(fil);
  }

  if (outTwist) {
    const auto fil = cli.arg_outTwist;
    std::cout << "\nSaving estimated twist to: " << fil
              << std::endl;  // NOLINT(performance-avoid-endl)
    outTwist->saveToTextFile(fil);
  }

  return 0;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    Cli cli;
    cli.setup();

    // Parse arguments:
    CLI11_PARSE(cli.app, argc, argv);

    // Load plugins:
    if (!cli.arg_plugins.empty()) {
      std::string errMsg;
      const auto plugins = cli.arg_plugins;
      std::cout << "Loading plugin(s): " << plugins << "\n";
      if (!mrpt::system::loadPluginModules(plugins, errMsg)) {
        std::cerr << errMsg << std::endl;  // NOLINT(performance-avoid-endl)
        return 1;
      }
    }

    mola_install_signal_handler();

    main_odometry(cli);

    return 0;
  } catch (std::exception & e) {
    mola::pretty_print_exception(e, "Exit due to exception:");
    return 1;
  }
}
