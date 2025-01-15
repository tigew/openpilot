"""Install exception handler for process crash."""
import os
import sentry_sdk
import traceback
from datetime import datetime
from enum import Enum
from sentry_sdk.integrations.threading import ThreadingIntegration

from openpilot.common.params import Params, ParamKeyType
from openpilot.system.athena.registration import is_registered_device
from openpilot.system.hardware import HARDWARE, PC
from openpilot.common.swaglog import cloudlog
from openpilot.system.version import get_build_metadata, get_version

CRASHES_DIR = "/data/crashes/"

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

  errors_to_ignore = [
    "already exists. To overwrite it, set 'overwrite' to True",
    "setup_quectel failed after retry",
  ]

  if any(error in exc_text for error in errors_to_ignore):
    return

  save_exception(exc_text)
  cloudlog.error("crash", exc_info=kwargs.get('exc_info', 1))

  try:
    sentry_sdk.capture_exception(*args, **kwargs)
    sentry_sdk.flush()  # https://github.com/getsentry/sentry-python/issues/291
  except Exception:
    cloudlog.exception("sentry exception")


def capture_fingerprint(candidate, params, blocked=False):
  params_tracking = Params("/persist/tracking")

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
          value = params_tracking.get_int(key)
        else:
          if isinstance(params.get(key), bytes):
            value = params.get(key, encoding='utf-8')
          else:
            value = params.get(key) or "0"

        if isinstance(value, str) and "." in value:
          value = value.rstrip("0").rstrip(".")
        matched_params[label][key.decode('utf-8')] = value

  for label, key_values in matched_params.items():
    if label == "FrogPilot Tracking":
      matched_params[label] = {key: f"{value:,}" for key, value in key_values.items()}
    else:
      matched_params[label] = {key: f"{value:}" for key, value in key_values.items()}

  with sentry_sdk.push_scope() as scope:
    for label, key_values in matched_params.items():
      scope.set_context(label, key_values)

    scope.fingerprint = [params.get("DongleId", encoding='utf-8'), candidate]

    if blocked:
      sentry_sdk.capture_message("Blocked user from using the development branch", level='warning')
    else:
      sentry_sdk.capture_message(f"Fingerprinted {candidate}", level='info')

    params.put_bool("FingerprintLogged", True)
    sentry_sdk.flush()


def capture_model(frogpilot_toggles):
  sentry_sdk.capture_message(f"User using: {frogpilot_toggles.model_name} - Model Randomizer: {frogpilot_toggles.model_randomizer}", level='info')


def capture_user(channel):
  sentry_sdk.capture_message(f"Logged user on: {channel}", level='info')


def set_tag(key: str, value: str) -> None:
  sentry_sdk.set_tag(key, value)


def save_exception(exc_text: str) -> None:
  os.makedirs(CRASHES_DIR, exist_ok=True)

  files = [
    os.path.join(CRASHES_DIR, datetime.now().strftime('%Y-%m-%d--%H-%M-%S.log')),
    os.path.join(CRASHES_DIR, 'error.txt')
  ]

  for file in files:
    with open(file, 'w') as f:
      if file.endswith("error.txt"):
        lines = exc_text.splitlines()[-10:]
        f.write("\n".join(lines))
      else:
        f.write(exc_text)

  print('Logged current crash to {}'.format(files))


def init(project: SentryProject) -> bool:
  build_metadata = get_build_metadata()
  FrogPilot = "frogai" in build_metadata.openpilot.git_origin.lower()
  if not FrogPilot or PC:
    return False

  params = Params()
  installed = params.get("InstallDate", encoding='utf-8')
  updated = params.get("Updated", encoding='utf-8')

  short_branch = build_metadata.channel

  if short_branch == "FrogPilot-Development":
    env = "Development"
  elif build_metadata.release_channel:
    env = "Release"
  elif build_metadata.tested_channel:
    env = "Staging"
  else:
    env = short_branch

  dongle_id = params.get("DongleId", encoding='utf-8')

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
