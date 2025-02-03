"""Install exception handler for process crash."""
import sentry_sdk
import traceback
from datetime import datetime
from enum import Enum
from pathlib import Path
from sentry_sdk.integrations.threading import ThreadingIntegration

from openpilot.common.params import Params, ParamKeyType
from openpilot.system.athena.registration import is_registered_device
from openpilot.system.hardware import HARDWARE, PC
from openpilot.common.swaglog import cloudlog
from openpilot.system.version import get_build_metadata, get_version

CRASHES_DIR = Path("/data/crashes")

class SentryProject(Enum):
  # python project
  SELFDRIVE = "https://5ad1714d27324c74a30f9c538bff3b8d@o4505034923769856.ingest.us.sentry.io/4505034930651136"
  # native project
  SELFDRIVE_NATIVE = "https://5ad1714d27324c74a30f9c538bff3b8d@o4505034923769856.ingest.us.sentry.io/4505034930651136"


def report_tombstone(fn: str, message: str, contents: str) -> None:
  cloudlog.error({'tombstone': message})

  with sentry_sdk.configure_scope() as scope:
    scope.set_extra("tombstone_fn", fn)
    scope.set_extra("tombstone", contents)
    sentry_sdk.capture_message(message=message)
    sentry_sdk.flush()


def capture_exception(*args, **kwargs) -> None:
  exc_text = traceback.format_exc()

  phrases_to_check = [
    "already exists. To overwrite it, set 'overwrite' to True",
    "setup_quectel failed after retry",
  ]

  if any(phrase in exc_text for phrase in phrases_to_check):
    return

  save_exception(exc_text)
  cloudlog.error("crash", exc_info=kwargs.get("exc_info", 1))

  try:
    sentry_sdk.capture_exception(*args, **kwargs)
    sentry_sdk.flush()  # https://github.com/getsentry/sentry-python/issues/291
  except Exception:
    cloudlog.exception("sentry exception")


def capture_fingerprint(frogpilot_toggles, params, params_tracking):
  if frogpilot_toggles.block_user:
    sentry_sdk.capture_message("Blocked user from using the development branch", level="warning")
    sentry_sdk.flush()
    return

  param_types = {
    "FrogPilot Controls": ParamKeyType.FROGPILOT_CONTROLS,
    "FrogPilot Vehicles": ParamKeyType.FROGPILOT_VEHICLES,
    "FrogPilot Visuals": ParamKeyType.FROGPILOT_VISUALS,
    "FrogPilot Other": ParamKeyType.FROGPILOT_OTHER,
    "FrogPilot Tracking": ParamKeyType.FROGPILOT_TRACKING,
  }

  matched_params = {label: {} for label in param_types}
  for key in params.all_keys():
    for label, key_type in param_types.items():
      if params.get_key_type(key) & key_type:
        if key_type == ParamKeyType.FROGPILOT_TRACKING:
          value = f"{params_tracking.get_int(key):,}"
        else:
          if isinstance(params.get(key), bytes):
            value = params.get(key, encoding="utf-8")
          else:
            value = params.get(key) or "0"

        if isinstance(value, str) and "." in value:
          value = value.rstrip("0").rstrip(".")
        matched_params[label][key.decode("utf-8")] = value

  with sentry_sdk.push_scope() as scope:
    for label, key_values in matched_params.items():
      scope.set_context(label, key_values)

    fingerprint = [params.get("DongleId", encoding="utf-8"), frogpilot_toggles.car_model]
    scope.fingerprint = fingerprint
    sentry_sdk.capture_message(f"Logged user: {fingerprint}", level="info")
    sentry_sdk.flush()


def capture_frogpilot_stats(frogpilot_toggles, branch):
  sentry_sdk.capture_message(f"Logged user on: {branch}", level="info")
  sentry_sdk.capture_message(f"User driving a: {frogpilot_toggles.car_model}", level="info")
  sentry_sdk.capture_message(f"User has a: {frogpilot_toggles.car_make.capitalize()}", level="info")
  sentry_sdk.capture_message(f"User using: {frogpilot_toggles.model_name.replace(' (Default)', '')}", level="info")
  sentry_sdk.flush()


def capture_report(discord_user, report, frogpilot_toggles):
  error_file_path = CRASHES_DIR / "error.txt"
  error_content = "No error log found."

  if error_file_path.exists():
    error_content = error_file_path.read_text()

  with sentry_sdk.push_scope() as scope:
    scope.set_context("Error Log", {"content": error_content})
    scope.set_context("Toggle Values", frogpilot_toggles)
    sentry_sdk.capture_message(f"{discord_user} submitted report: {report}", level="fatal")
    sentry_sdk.flush()


def set_tag(key: str, value: str) -> None:
  sentry_sdk.set_tag(key, value)


def save_exception(exc_text: str) -> None:
  CRASHES_DIR.mkdir(parents=True, exist_ok=True)

  files = [
    CRASHES_DIR / datetime.now().strftime("%Y-%m-%d--%H-%M-%S.log"),
    CRASHES_DIR / "error.txt"
  ]

  for file_path in files:
    if file_path.name == "error.txt":
      lines = exc_text.splitlines()[-10:]
      file_path.write_text("\n".join(lines))
    else:
      file_path.write_text(exc_text)

  print(f"Logged current crash to {[str(file) for file in files]}")


def init(project: SentryProject) -> bool:
  build_metadata = get_build_metadata()
  FrogPilot = "frogai" in build_metadata.openpilot.git_origin.lower()
  if not FrogPilot or PC:
    return False

  short_branch = build_metadata.channel

  if short_branch == "FrogPilot-Development":
    env = "Development"
  elif build_metadata.release_channel:
    env = "Release"
  elif build_metadata.tested_channel:
    env = "Staging"
  else:
    env = short_branch

  params = Params()
  dongle_id = params.get("DongleId", encoding="utf-8")
  installed = params.get("InstallDate", encoding="utf-8")
  updated = params.get("Updated", encoding="utf-8")

  integrations = []
  if project == SentryProject.SELFDRIVE:
    integrations.append(ThreadingIntegration(propagate_hub=True))

  sentry_sdk.init(project.value,
                  default_integrations=False,
                  release=get_version(),
                  integrations=integrations,
                  traces_sample_rate=1.0,
                  max_value_length=8192,
                  environment=env)

  sentry_sdk.set_user({"id": dongle_id})
  sentry_sdk.set_tag("origin", build_metadata.openpilot.git_origin)
  sentry_sdk.set_tag("branch", short_branch)
  sentry_sdk.set_tag("commit", build_metadata.openpilot.git_commit)
  sentry_sdk.set_tag("updated", updated)
  sentry_sdk.set_tag("installed", installed)

  if project == SentryProject.SELFDRIVE:
    sentry_sdk.Hub.current.start_session()

  return True
