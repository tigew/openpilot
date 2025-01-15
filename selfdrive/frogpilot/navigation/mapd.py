# PFEIFER - MAPD - Modified by FrogAi for FrogPilot
#!/usr/bin/env python3
import json
import os
import stat
import subprocess
import time
import urllib.request

import openpilot.system.sentry as sentry

from pathlib import Path

from openpilot.selfdrive.frogpilot.frogpilot_utilities import is_url_pingable
from openpilot.selfdrive.frogpilot.frogpilot_variables import MAPD_PATH, MAPS_PATH

VERSION = "v1"

GITHUB_VERSION_URL = f"https://github.com/FrogAi/FrogPilot-Resources/raw/Versions/mapd_version_{VERSION}.json"
GITLAB_VERSION_URL = f"https://gitlab.com/FrogAi/FrogPilot-Resources/-/raw/Versions/mapd_version_{VERSION}.json"

VERSION_PATH = Path("/data/media/0/osm/mapd_version")

def download():
  while not (is_url_pingable("https://github.com") or is_url_pingable("https://gitlab.com")):
    time.sleep(60)

  latest_version = get_latest_version()

  urls = [
    f"https://github.com/pfeiferj/openpilot-mapd/releases/download/{latest_version}/mapd",
    f"https://gitlab.com/FrogAi/FrogPilot-Resources/-/raw/Mapd/{latest_version}"
  ]

  os.makedirs(os.path.dirname(MAPD_PATH), exist_ok=True)

  for url in urls:
    try:
      with urllib.request.urlopen(url) as f:
        with open(MAPD_PATH, 'wb') as output:
          output.write(f.read())
          os.fsync(output)
          current_permissions = stat.S_IMODE(os.lstat(MAPD_PATH).st_mode)
          os.chmod(MAPD_PATH, current_permissions | stat.S_IEXEC)
        with open(VERSION_PATH, 'w') as output:
          output.write(latest_version)
          os.fsync(output)
      return
    except Exception as error:
      print(f"Failed to download mapd from {url}: {error}")
      sentry.capture_exception(error)

def get_latest_version():
  for url in [GITHUB_VERSION_URL, GITLAB_VERSION_URL]:
    try:
      with urllib.request.urlopen(url, timeout=5) as response:
        return json.loads(response.read().decode('utf-8'))['version']
    except TimeoutError as error:
      print(f"Timeout while fetching mapd version from {url}: {error}")
    except URLError as error:
      print(f"URLError encountered for {url}: {error}")
    except Exception as error:
      print(f"Error fetching mapd version from {url}: {error}")
      sentry.capture_exception(error)
  print("Failed to get the latest mapd version")
  return "v0"

def mapd_thread():
  while True:
    try:
      if not os.path.exists(MAPD_PATH):
        print(f"{MAPD_PATH} not found. Downloading...")
        download()
        continue
      else:
        current_permissions = stat.S_IMODE(os.lstat(MAPD_PATH).st_mode)
        desired_permissions = current_permissions | stat.S_IEXEC

        if current_permissions != desired_permissions:
          print(f"{MAPD_PATH} has the wrong permissions. Attempting to fix...")
          os.chmod(MAPD_PATH, desired_permissions)
      if not os.path.exists(VERSION_PATH):
        download()
        continue
      with open(VERSION_PATH) as f:
        current_version = f.read()
        if is_url_pingable("https://github.com") or is_url_pingable("https://gitlab.com"):
          if current_version != get_latest_version():
            print("New mapd version available. Downloading...")
            download()
            continue

      process = subprocess.Popen(MAPD_PATH)
      process.wait()
    except Exception as error:
      print(f"Error in mapd_thread: {error}")
      sentry.capture_exception(error)
      time.sleep(60)

def main():
  mapd_thread()

if __name__ == "__main__":
  main()
