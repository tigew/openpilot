#include "common/params.h"

#include <dirent.h>
#include <sys/file.h>

#include <algorithm>
#include <cassert>
#include <csignal>
#include <unordered_map>

#include "common/queue.h"
#include "common/swaglog.h"
#include "common/util.h"
#include "system/hardware/hw.h"

namespace {

volatile sig_atomic_t params_do_exit = 0;
void params_sig_handler(int signal) {
  params_do_exit = 1;
}

int fsync_dir(const std::string &path) {
  int result = -1;
  int fd = HANDLE_EINTR(open(path.c_str(), O_RDONLY, 0755));
  if (fd >= 0) {
    result = fsync(fd);
    close(fd);
  }
  return result;
}

bool create_params_path(const std::string &param_path, const std::string &key_path) {
  // Make sure params path exists
  if (!util::file_exists(param_path) && !util::create_directories(param_path, 0775)) {
    return false;
  }

  // See if the symlink exists, otherwise create it
  if (!util::file_exists(key_path)) {
    // 1) Create temp folder
    // 2) Symlink it to temp link
    // 3) Move symlink to <params>/d

    std::string tmp_path = param_path + "/.tmp_XXXXXX";
    // this should be OK since mkdtemp just replaces characters in place
    char *tmp_dir = mkdtemp((char *)tmp_path.c_str());
    if (tmp_dir == NULL) {
      return false;
    }

    std::string link_path = std::string(tmp_dir) + ".link";
    if (symlink(tmp_dir, link_path.c_str()) != 0) {
      return false;
    }

    // don't return false if it has been created by other
    if (rename(link_path.c_str(), key_path.c_str()) != 0 && errno != EEXIST) {
      return false;
    }
  }

  return true;
}

std::string ensure_params_path(const std::string &prefix, const std::string &path = {}) {
  std::string params_path = path.empty() ? Path::params() : path;
  if (!create_params_path(params_path, params_path + prefix)) {
    throw std::runtime_error(util::string_format(
        "Failed to ensure params path, errno=%d, path=%s, param_prefix=%s",
        errno, params_path.c_str(), prefix.c_str()));
  }
  return params_path;
}

class FileLock {
public:
  FileLock(const std::string &fn) {
    fd_ = HANDLE_EINTR(open(fn.c_str(), O_CREAT, 0775));
    if (fd_ < 0 || HANDLE_EINTR(flock(fd_, LOCK_EX)) < 0) {
      LOGE("Failed to lock file %s, errno=%d", fn.c_str(), errno);
    }
  }
  ~FileLock() { close(fd_); }

private:
  int fd_ = -1;
};

std::unordered_map<std::string, uint32_t> keys = {
    {"AccessToken", CLEAR_ON_MANAGER_START | DONT_LOG},
    {"AlwaysOnDM", PERSISTENT | FROGPILOT_STORAGE},
    {"ApiCache_Device", PERSISTENT},
    {"ApiCache_NavDestinations", PERSISTENT},
    {"AssistNowToken", PERSISTENT},
    {"AthenadPid", PERSISTENT},
    {"AthenadUploadQueue", PERSISTENT},
    {"AthenadRecentlyViewedRoutes", PERSISTENT},
    {"BootCount", PERSISTENT},
    {"CalibrationParams", PERSISTENT},
    {"CameraDebugExpGain", CLEAR_ON_MANAGER_START},
    {"CameraDebugExpTime", CLEAR_ON_MANAGER_START},
    {"CarBatteryCapacity", PERSISTENT},
    {"CarParams", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"CarParamsCache", CLEAR_ON_MANAGER_START},
    {"CarParamsPersistent", PERSISTENT},
    {"CarParamsPrevRoute", PERSISTENT},
    {"CarVin", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"CompletedTrainingVersion", PERSISTENT},
    {"ControlsReady", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"CurrentBootlog", PERSISTENT},
    {"CurrentRoute", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"DisableLogging", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"DisablePowerDown", PERSISTENT},
    {"DisableUpdates", PERSISTENT},
    {"DisengageOnAccelerator", PERSISTENT | FROGPILOT_STORAGE},
    {"DmModelInitialized", CLEAR_ON_ONROAD_TRANSITION},
    {"DongleId", PERSISTENT},
    {"DoReboot", CLEAR_ON_MANAGER_START},
    {"DoShutdown", CLEAR_ON_MANAGER_START},
    {"DoUninstall", CLEAR_ON_MANAGER_START},
    {"ExperimentalLongitudinalEnabled", PERSISTENT | DEVELOPMENT_ONLY | FROGPILOT_STORAGE},
    {"ExperimentalMode", PERSISTENT},
    {"ExperimentalModeConfirmed", PERSISTENT},
    {"FirmwareQueryDone", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"ForcePowerDown", PERSISTENT},
    {"GitBranch", PERSISTENT},
    {"GitCommit", PERSISTENT},
    {"GitCommitDate", PERSISTENT},
    {"GitDiff", PERSISTENT},
    {"GithubSshKeys", PERSISTENT},
    {"GithubUsername", PERSISTENT},
    {"GitRemote", PERSISTENT},
    {"GsmApn", PERSISTENT},
    {"GsmMetered", PERSISTENT | FROGPILOT_STORAGE},
    {"GsmRoaming", PERSISTENT | FROGPILOT_STORAGE},
    {"HardwareSerial", PERSISTENT},
    {"HasAcceptedTerms", PERSISTENT},
    {"IMEI", PERSISTENT},
    {"InstallDate", PERSISTENT},
    {"IsDriverViewEnabled", CLEAR_ON_MANAGER_START},
    {"IsEngaged", PERSISTENT},
    {"IsLdwEnabled", PERSISTENT | FROGPILOT_STORAGE},
    {"IsMetric", PERSISTENT | FROGPILOT_STORAGE},
    {"IsOffroad", CLEAR_ON_MANAGER_START},
    {"IsOnroad", PERSISTENT},
    {"IsRhdDetected", PERSISTENT},
    {"IsReleaseBranch", CLEAR_ON_MANAGER_START},
    {"IsTakingSnapshot", CLEAR_ON_MANAGER_START},
    {"IsTestedBranch", CLEAR_ON_MANAGER_START},
    {"JoystickDebugMode", CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION},
    {"LanguageSetting", PERSISTENT},
    {"LastAthenaPingTime", CLEAR_ON_MANAGER_START},
    {"LastGPSPosition", PERSISTENT},
    {"LastManagerExitReason", CLEAR_ON_MANAGER_START},
    {"LastOffroadStatusPacket", CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION},
    {"LastPowerDropDetected", CLEAR_ON_MANAGER_START},
    {"LastUpdateException", CLEAR_ON_MANAGER_START},
    {"LastUpdateTime", PERSISTENT},
    {"LiveParameters", PERSISTENT},
    {"LiveTorqueParameters", PERSISTENT | DONT_LOG},
    {"LongitudinalPersonality", PERSISTENT},
    {"NavDestination", CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION},
    {"NavDestinationWaypoints", CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION},
    {"NavPastDestinations", PERSISTENT},
    {"NavSettingLeftSide", PERSISTENT | FROGPILOT_STORAGE},
    {"NavSettingTime24h", PERSISTENT},
    {"NetworkMetered", PERSISTENT},
    {"ObdMultiplexingChanged", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"ObdMultiplexingEnabled", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"Offroad_BadNvme", CLEAR_ON_MANAGER_START},
    {"Offroad_CarUnrecognized", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"Offroad_ConnectivityNeeded", CLEAR_ON_MANAGER_START},
    {"Offroad_ConnectivityNeededPrompt", CLEAR_ON_MANAGER_START},
    {"Offroad_IsTakingSnapshot", CLEAR_ON_MANAGER_START},
    {"Offroad_NeosUpdate", CLEAR_ON_MANAGER_START},
    {"Offroad_NoFirmware", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"Offroad_Recalibration", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"Offroad_StorageMissing", CLEAR_ON_MANAGER_START},
    {"Offroad_TemperatureTooHigh", CLEAR_ON_MANAGER_START},
    {"Offroad_UnofficialHardware", CLEAR_ON_MANAGER_START},
    {"Offroad_UpdateFailed", CLEAR_ON_MANAGER_START},
    {"OpenpilotEnabledToggle", PERSISTENT | FROGPILOT_STORAGE},
    {"PandaHeartbeatLost", CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION},
    {"PandaSomResetTriggered", CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION},
    {"PandaSignatures", CLEAR_ON_MANAGER_START},
    {"PrimeType", PERSISTENT},
    {"RecordFront", PERSISTENT | FROGPILOT_STORAGE},
    {"RecordFrontLock", PERSISTENT},  // for the internal fleet
    {"ReplayControlsState", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"RouteCount", PERSISTENT},
    {"SecOCKey", PERSISTENT | DONT_LOG},
    {"SnoozeUpdate", CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION},
    {"SshEnabled", PERSISTENT | FROGPILOT_STORAGE},
    {"TermsVersion", PERSISTENT},
    {"Timezone", PERSISTENT},
    {"TrainingVersion", PERSISTENT},
    {"UbloxAvailable", PERSISTENT},
    {"UpdateAvailable", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"UpdateFailedCount", CLEAR_ON_MANAGER_START},
    {"UpdaterAvailableBranches", PERSISTENT},
    {"UpdaterCurrentDescription", CLEAR_ON_MANAGER_START},
    {"UpdaterCurrentReleaseNotes", CLEAR_ON_MANAGER_START},
    {"UpdaterFetchAvailable", CLEAR_ON_MANAGER_START},
    {"UpdaterNewDescription", CLEAR_ON_MANAGER_START},
    {"UpdaterNewReleaseNotes", CLEAR_ON_MANAGER_START},
    {"UpdaterState", CLEAR_ON_MANAGER_START},
    {"UpdaterTargetBranch", CLEAR_ON_MANAGER_START},
    {"UpdaterLastFetchTime", PERSISTENT},
    {"Version", PERSISTENT},

    // FrogPilot parameters
    {"AccelerationPath", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"AccelerationProfile", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"AdjacentLeadsUI", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"AdjacentPath", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"AdjacentPathMetrics", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"AdvancedCustomUI", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"AdvancedLateralTune", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"AggressiveFollow", PERSISTENT | FROGPILOT_CONTROLS},
    {"AggressiveJerkAcceleration", PERSISTENT | FROGPILOT_CONTROLS},
    {"AggressiveJerkDanger", PERSISTENT | FROGPILOT_CONTROLS},
    {"AggressiveJerkDeceleration", PERSISTENT | FROGPILOT_CONTROLS},
    {"AggressiveJerkSpeed", PERSISTENT | FROGPILOT_CONTROLS},
    {"AggressiveJerkSpeedDecrease", PERSISTENT | FROGPILOT_CONTROLS},
    {"AggressivePersonalityProfile", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"AlertVolumeControl", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"AlwaysOnLateral", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"AlwaysOnLateralLKAS", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"AlwaysOnLateralMain", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"AMapKey1", PERSISTENT},
    {"AMapKey2", PERSISTENT},
    {"ApiCache_DriveStats", PERSISTENT},
    {"AutomaticallyUpdateModels", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"AutomaticUpdates", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_OTHER},
    {"AvailableModels", PERSISTENT},
    {"AvailableModelNames", PERSISTENT},
    {"BigMap", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"BlacklistedModels", PERSISTENT | FROGPILOT_CONTROLS},
    {"BlindSpotMetrics", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"BlindSpotPath", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"BorderMetrics", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"CameraView", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"CancelModelDownload", PERSISTENT},
    {"CancelThemeDownload", PERSISTENT},
    {"CarMake", PERSISTENT},
    {"CarModel", PERSISTENT},
    {"CarModelName", PERSISTENT},
    {"CECurves", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"CECurvesLead", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"CELead", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"CEModelStopTime", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"CENavigation", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"CENavigationIntersections", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"CENavigationLead", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"CENavigationTurns", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"CertifiedHerbalistDrives", PERSISTENT | FROGPILOT_CONTROLS},
    {"CertifiedHerbalistScore", PERSISTENT | FROGPILOT_CONTROLS},
    {"CESignalSpeed", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"CESignalLaneDetection", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"CESlowerLead", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"CESpeed", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"CESpeedLead", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"CEStatus", CLEAR_ON_OFFROAD_TRANSITION},
    {"CEStoppedLead", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"ClusterOffset", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VEHICLES},
    {"ColorToDownload", PERSISTENT},
    {"Compass", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"ConditionalExperimental", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"CrosstrekTorque", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VEHICLES},
    {"CurveSensitivity", PERSISTENT | FROGPILOT_CONTROLS},
    {"CurveSpeedControl", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"CustomAlerts", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"CustomColors", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"CustomCruise", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"CustomCruiseLong", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"CustomDistanceIcons", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"CustomIcons", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"CustomPersonalities", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"CustomSignals", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"CustomSounds", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"CustomUI", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"DecelerationProfile", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"DeveloperUI", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"DeviceManagement", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"DeviceShutdown", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"DisableOnroadUploads", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"DisableOpenpilotLongitudinal", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VEHICLES},
    {"DissolvedOxygenDrives", PERSISTENT | FROGPILOT_CONTROLS},
    {"DissolvedOxygenScore", PERSISTENT | FROGPILOT_CONTROLS},
    {"DistanceIconToDownload", PERSISTENT},
    {"DisengageVolume", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"DoToggleReset", PERSISTENT},
    {"DownloadableColors", PERSISTENT},
    {"DownloadableDistanceIcons", PERSISTENT},
    {"DownloadableIcons", PERSISTENT},
    {"DownloadableSignals", PERSISTENT},
    {"DownloadableSounds", PERSISTENT},
    {"DownloadableWheels", PERSISTENT},
    {"DownloadAllModels", PERSISTENT},
    {"DragonRiderDrives", PERSISTENT | FROGPILOT_CONTROLS},
    {"DragonRiderScore", PERSISTENT | FROGPILOT_CONTROLS},
    {"DriverCamera", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"DuckAmigoDrives", PERSISTENT | FROGPILOT_CONTROLS},
    {"DuckAmigoScore", PERSISTENT | FROGPILOT_CONTROLS},
    {"DynamicPathWidth", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"DynamicPedalsOnUI", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"EngageVolume", PERSISTENT | FROGPILOT_VISUALS},
    {"ExperimentalGMTune", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VEHICLES},
    {"ExperimentalModeActivation", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"ExperimentalModels", PERSISTENT},
    {"ExperimentalModeViaDistance", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"ExperimentalModeViaLKAS", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"ExperimentalModeViaTap", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"Fahrenheit", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"FingerprintLogged", CLEAR_ON_MANAGER_START},
    {"ForceAutoTune", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"ForceAutoTuneOff", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"ForceFingerprint", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VEHICLES},
    {"ForceMPHDashboard", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"ForceOffroad", PERSISTENT},
    {"ForceOnroad", PERSISTENT},
    {"ForceStandstill", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"ForceStops", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"FPSCounter", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"FrogPilotDrives", PERSISTENT | FROGPILOT_TRACKING},
    {"FrogPilotKilometers", PERSISTENT | FROGPILOT_TRACKING},
    {"FrogPilotMinutes", PERSISTENT | FROGPILOT_TRACKING},
    {"FrogPilotToggles", CLEAR_ON_MANAGER_START},
    {"FrogPilotTogglesUpdated", PERSISTENT},
    {"FrogPilotTuningLevels", CLEAR_ON_MANAGER_START},
    {"FrogsGoMoosTweak", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VEHICLES},
    {"FullMap", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"GasRegenCmd", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VEHICLES},
    {"GMapKey", PERSISTENT},
    {"GoatScream", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"GreenLightAlert", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"HideAlerts", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"HideCSCUI", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"HideLeadMarker", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"HideMapIcon", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"HideMaxSpeed", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"HideSpeed", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"HideSpeedLimit", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"HolidayThemes", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"HumanAcceleration", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"HumanFollowing", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"IconToDownload", PERSISTENT},
    {"IncreasedStoppedDistance", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"IncreaseThermalLimits", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"JerkInfo", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"LaneChangeCustomizations", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"LaneChangeTime", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"LaneDetectionWidth", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"LaneLinesWidth", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"LastMapsUpdate", PERSISTENT | FROGPILOT_OTHER},
    {"LateralMetrics", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"LateralTune", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"LeadDepartingAlert", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"LeadDetectionThreshold", PERSISTENT | FROGPILOT_CONTROLS},
    {"LeadInfo", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"LockDoors", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VEHICLES},
    {"LockDoorsTimer", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VEHICLES},
    {"LongitudinalMetrics", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"LongitudinalTune", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"LongPitch", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VEHICLES},
    {"LosAngelesDrives", PERSISTENT | FROGPILOT_CONTROLS},
    {"LosAngelesScore", PERSISTENT | FROGPILOT_CONTROLS},
    {"LoudBlindspotAlert", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"LowVoltageShutdown", PERSISTENT | FROGPILOT_CONTROLS},
    {"ManualUpdateInitiated", PERSISTENT},
    {"MapAcceleration", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"MapboxPublicKey", PERSISTENT},
    {"MapboxSecretKey", PERSISTENT},
    {"MapDeceleration", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"MapGears", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"MapsSelected", PERSISTENT | FROGPILOT_OTHER},
    {"MapSpeedLimit", PERSISTENT},
    {"MapStyle", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"MapTargetVelocities", PERSISTENT},
    {"MaxDesiredAcceleration", PERSISTENT | FROGPILOT_CONTROLS},
    {"MinimumBackupSize", PERSISTENT},
    {"MinimumLaneChangeSpeed", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"Model", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"ModelDrivesAndScores", PERSISTENT | FROGPILOT_CONTROLS},
    {"ModelDownloadProgress", PERSISTENT},
    {"ModelName", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"ModelRandomizer", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"ModelToDownload", PERSISTENT},
    {"ModelUI", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"ModelVersion", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"ModelVersions", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"MTSCCurvatureCheck", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"MTSCEnabled", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"NavigationUI", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"NextMapSpeedLimit", PERSISTENT},
    {"NewLongAPI", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VEHICLES},
    {"NewLongAPIGM", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VEHICLES},
    {"NNFF", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"NNFFLite", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"NNFFModelName", CLEAR_ON_OFFROAD_TRANSITION},
    {"NoLogging", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"NorthDakotaDrives", PERSISTENT | FROGPILOT_CONTROLS},
    {"NorthDakotaScore", PERSISTENT | FROGPILOT_CONTROLS},
    {"NotreDameDrives", PERSISTENT | FROGPILOT_CONTROLS},
    {"NotreDameScore", PERSISTENT | FROGPILOT_CONTROLS},
    {"NoUploads", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"NudgelessLaneChange", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"NumericalTemp", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"OfflineMode", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"Offset1", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"Offset2", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"Offset3", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"Offset4", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"OneLaneChange", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"OnroadDistanceButton", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"OnroadDistanceButtonPressed", PERSISTENT},
    {"openpilotMinutes", PERSISTENT},
    {"OSMDownloadBounds", PERSISTENT},
    {"OSMDownloadLocations", PERSISTENT},
    {"OSMDownloadProgress", CLEAR_ON_MANAGER_START},
    {"PathEdgeWidth", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"PathWidth", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"PauseAOLOnBrake", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"PauseLateralOnSignal", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"PauseLateralSpeed", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"PedalsOnUI", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"PersonalizeOpenpilot", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"PreferredSchedule", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_OTHER},
    {"PreviousSpeedLimit", PERSISTENT},
    {"PromptDistractedVolume", PERSISTENT | FROGPILOT_VISUALS},
    {"PromptVolume", PERSISTENT | FROGPILOT_VISUALS},
    {"QOLLateral", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"QOLLongitudinal", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"QOLVisuals", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"RadicalTurtleDrives", PERSISTENT | FROGPILOT_CONTROLS},
    {"RadicalTurtleScore", PERSISTENT | FROGPILOT_CONTROLS},
    {"RainbowPath", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"RandomEvents", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"RecertifiedHerbalistDrives", PERSISTENT | FROGPILOT_CONTROLS},
    {"RecertifiedHerbalistScore", PERSISTENT | FROGPILOT_CONTROLS},
    {"RefuseVolume", PERSISTENT | FROGPILOT_VISUALS},
    {"RelaxedFollow", PERSISTENT | FROGPILOT_CONTROLS},
    {"RelaxedJerkAcceleration", PERSISTENT | FROGPILOT_CONTROLS},
    {"RelaxedJerkDanger", PERSISTENT | FROGPILOT_CONTROLS},
    {"RelaxedJerkDeceleration", PERSISTENT | FROGPILOT_CONTROLS},
    {"RelaxedJerkSpeed", PERSISTENT | FROGPILOT_CONTROLS},
    {"RelaxedJerkSpeedDecrease", PERSISTENT | FROGPILOT_CONTROLS},
    {"RelaxedPersonalityProfile", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"ReverseCruise", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"RoadEdgesWidth", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"RoadName", PERSISTENT},
    {"RoadNameUI", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"RotatingWheel", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"ScreenBrightness", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"ScreenBrightnessOnroad", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"ScreenManagement", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"ScreenRecorder", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"ScreenTimeout", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"ScreenTimeoutOnroad", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"SearchInput", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_OTHER},
    {"SecretGoodOpenpilotDrives", PERSISTENT | FROGPILOT_CONTROLS},
    {"SecretGoodOpenpilotScore", PERSISTENT | FROGPILOT_CONTROLS},
    {"SetSpeedLimit", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"SetSpeedOffset", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"ShowCEMStatus", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"ShowCPU", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"ShowGPU", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"ShowIP", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"ShowMemoryUsage", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"ShowSLCOffset", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"ShowSpeedLimits", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"ShowSteering", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"ShowStoppingPoint", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"ShowStoppingPointMetrics", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"ShowStorageLeft", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"ShowStorageUsed", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"Sidebar", PERSISTENT | FROGPILOT_OTHER},
    {"SidebarMetrics", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"SignalMetrics", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"SignalToDownload", PERSISTENT},
    {"SLCConfirmation", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"SLCConfirmationHigher", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"SLCConfirmationLower", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"SLCLookaheadHigher", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"SLCLookaheadLower", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"SLCFallback", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"SLCOverride", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"SLCPriority1", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"SLCPriority2", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"SLCPriority3", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"SNGHack", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VEHICLES},
    {"SoundToDownload", PERSISTENT},
    {"SpeedLimitAccepted", PERSISTENT},
    {"SpeedLimitChangedAlert", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"SpeedLimitController", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"SpeedLimitSources", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"StandardFollow", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"StandardJerkAcceleration", PERSISTENT | FROGPILOT_CONTROLS},
    {"StandardJerkDanger", PERSISTENT | FROGPILOT_CONTROLS},
    {"StandardJerkDeceleration", PERSISTENT | FROGPILOT_CONTROLS},
    {"StandardJerkSpeed", PERSISTENT | FROGPILOT_CONTROLS},
    {"StandardJerkSpeedDecrease", PERSISTENT | FROGPILOT_CONTROLS},
    {"StandardPersonalityProfile", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"StandbyMode", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"StartupMessageBottom", PERSISTENT | FROGPILOT_VISUALS},
    {"StartupMessageTop", PERSISTENT | FROGPILOT_VISUALS},
    {"StaticPedalsOnUI", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"SteerFriction", PERSISTENT | FROGPILOT_CONTROLS},
    {"SteerFrictionStock", PERSISTENT},
    {"SteerLatAccel", PERSISTENT | FROGPILOT_CONTROLS},
    {"SteerLatAccelStock", PERSISTENT},
    {"SteerKP", PERSISTENT | FROGPILOT_CONTROLS},
    {"SteerKPStock", PERSISTENT},
    {"SteerRatio", PERSISTENT | FROGPILOT_CONTROLS},
    {"SteerRatioStock", PERSISTENT},
    {"StoppedTimer", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"TacoTune", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"TestingSound", PERSISTENT},
    {"TetheringEnabled", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_OTHER},
    {"ThemeDownloadProgress", PERSISTENT},
    {"ToyotaDoors", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VEHICLES},
    {"TrafficFollow", PERSISTENT | FROGPILOT_CONTROLS},
    {"TrafficJerkAcceleration", PERSISTENT | FROGPILOT_CONTROLS},
    {"TrafficJerkDanger", PERSISTENT | FROGPILOT_CONTROLS},
    {"TrafficJerkDeceleration", PERSISTENT | FROGPILOT_CONTROLS},
    {"TrafficJerkSpeed", PERSISTENT | FROGPILOT_CONTROLS},
    {"TrafficJerkSpeedDecrease", PERSISTENT | FROGPILOT_CONTROLS},
    {"TrafficPersonalityProfile", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"TuningInfo", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"TuningLevel", PERSISTENT},
    {"TuningLevelConfirmed", PERSISTENT},
    {"TurnAggressiveness", PERSISTENT | FROGPILOT_CONTROLS},
    {"TurnDesires", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"UnlimitedLength", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"UnlockDoors", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VEHICLES},
    {"Updated", PERSISTENT},
    {"UpdateWheelImage", PERSISTENT},
    {"UseSI", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"UseStockColors", CLEAR_ON_MANAGER_START},
    {"UseVienna", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"VisionTurnControl", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_CONTROLS},
    {"VoltSNG", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VEHICLES},
    {"WarningImmediateVolume", PERSISTENT | FROGPILOT_VISUALS},
    {"WarningSoftVolume", PERSISTENT | FROGPILOT_VISUALS},
    {"WD40Drives", PERSISTENT | FROGPILOT_CONTROLS},
    {"WD40Score", PERSISTENT | FROGPILOT_CONTROLS},
    {"WheelIcon", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"WheelSpeed", PERSISTENT | FROGPILOT_STORAGE | FROGPILOT_VISUALS},
    {"WheelToDownload", PERSISTENT},
};

} // namespace


Params::Params(const std::string &path) {
  params_prefix = "/" + util::getenv("OPENPILOT_PREFIX", "d");
  params_path = ensure_params_path(params_prefix, path);
}

Params::~Params() {
  if (future.valid()) {
    future.wait();
  }
  assert(queue.empty());
}

std::vector<std::string> Params::allKeys() const {
  std::vector<std::string> ret;
  for (auto &p : keys) {
    ret.push_back(p.first);
  }
  return ret;
}

bool Params::checkKey(const std::string &key) {
  return keys.find(key) != keys.end();
}

ParamKeyType Params::getKeyType(const std::string &key) {
  return static_cast<ParamKeyType>(keys[key]);
}

int Params::put(const char* key, const char* value, size_t value_size) {
  // Information about safely and atomically writing a file: https://lwn.net/Articles/457667/
  // 1) Create temp file
  // 2) Write data to temp file
  // 3) fsync() the temp file
  // 4) rename the temp file to the real name
  // 5) fsync() the containing directory
  std::string tmp_path = params_path + "/.tmp_value_XXXXXX";
  int tmp_fd = mkstemp((char*)tmp_path.c_str());
  if (tmp_fd < 0) return -1;

  int result = -1;
  do {
    // Write value to temp.
    ssize_t bytes_written = HANDLE_EINTR(write(tmp_fd, value, value_size));
    if (bytes_written < 0 || (size_t)bytes_written != value_size) {
      result = -20;
      break;
    }

    // fsync to force persist the changes.
    if ((result = fsync(tmp_fd)) < 0) break;

    FileLock file_lock(params_path + "/.lock");

    // Move temp into place.
    if ((result = rename(tmp_path.c_str(), getParamPath(key).c_str())) < 0) break;

    // fsync parent directory
    result = fsync_dir(getParamPath());
  } while (false);

  close(tmp_fd);
  if (result != 0) {
    ::unlink(tmp_path.c_str());
  }
  return result;
}

int Params::remove(const std::string &key) {
  FileLock file_lock(params_path + "/.lock");
  int result = unlink(getParamPath(key).c_str());
  if (result != 0) {
    return result;
  }
  return fsync_dir(getParamPath());
}

std::string Params::get(const std::string &key, bool block) {
  if (!block) {
    return util::read_file(getParamPath(key));
  } else {
    // blocking read until successful
    params_do_exit = 0;
    void (*prev_handler_sigint)(int) = std::signal(SIGINT, params_sig_handler);
    void (*prev_handler_sigterm)(int) = std::signal(SIGTERM, params_sig_handler);

    std::string value;
    while (!params_do_exit) {
      if (value = util::read_file(getParamPath(key)); !value.empty()) {
        break;
      }
      util::sleep_for(100);  // 0.1 s
    }

    std::signal(SIGINT, prev_handler_sigint);
    std::signal(SIGTERM, prev_handler_sigterm);
    return value;
  }
}

std::map<std::string, std::string> Params::readAll() {
  FileLock file_lock(params_path + "/.lock");
  return util::read_files_in_dir(getParamPath());
}

void Params::clearAll(ParamKeyType key_type) {
  FileLock file_lock(params_path + "/.lock");

  // 1) delete params of key_type
  // 2) delete files that are not defined in the keys.
  if (DIR *d = opendir(getParamPath().c_str())) {
    struct dirent *de = NULL;
    while ((de = readdir(d))) {
      if (de->d_type != DT_DIR) {
        auto it = keys.find(de->d_name);
        if (it == keys.end() || (it->second & key_type)) {
          unlink(getParamPath(de->d_name).c_str());
        }
      }
    }
    closedir(d);
  }

  fsync_dir(getParamPath());
}

void Params::putNonBlocking(const std::string &key, const std::string &val) {
   queue.push(std::make_pair(key, val));
  // start thread on demand
  if (!future.valid() || future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
    future = std::async(std::launch::async, &Params::asyncWriteThread, this);
  }
}

void Params::asyncWriteThread() {
  // TODO: write the latest one if a key has multiple values in the queue.
  std::pair<std::string, std::string> p;
  while (queue.try_pop(p, 0)) {
    // Params::put is Thread-Safe
    put(p.first, p.second);
  }
}
