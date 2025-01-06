# PFEIFER - MAPD - Modified by FrogAi for FrogPilot
#!/usr/bin/env python3
import json
import stat
import subprocess
import urllib.request

from pathlib import Path

VERSION = "v1"

GITHUB_VERSION_URL = f"https://github.com/FrogAi/FrogPilot-Resources/raw/Versions/mapd_version_{VERSION}.json"
GITLAB_VERSION_URL = f"https://gitlab.com/FrogAi/FrogPilot-Resources/-/raw/Versions/mapd_version_{VERSION}.json"

MAPD_PATH = Path("/data/media/0/osm/mapd")
MAPS_PATH = Path("/data/media/0/osm/offline")
VERSION_PATH = Path("/data/media/0/osm/mapd_version")

def download(current_version):
  urls = [
    f"https://github.com/pfeiferj/openpilot-mapd/releases/download/{current_version}/mapd",
    f"https://gitlab.com/FrogAi/FrogPilot-Resources/-/raw/Mapd/{current_version}"
  ]

  MAPD_PATH.parent.mkdir(parents=True, exist_ok=True)

  for url in urls:
    try:
      with urllib.request.urlopen(url, timeout=5) as f:
        with MAPD_PATH.open('wb') as output:
          output.write(f.read())

        MAPD_PATH.chmod(MAPD_PATH.stat().st_mode | stat.S_IEXEC)
        VERSION_PATH.write_text(current_version)
        print(f"Successfully downloaded mapd from {url}")
        return True
    except Exception as error:
      print(f"Failed to download mapd from {url}: {error}")

  print(f"Failed to download mapd for version {current_version}")
  return False

def get_installed_version():
  return VERSION_PATH.read_text().strip()

def get_latest_version():
  for url in [GITHUB_VERSION_URL, GITLAB_VERSION_URL]:
    try:
      with urllib.request.urlopen(url, timeout=5) as response:
        return json.loads(response.read().decode('utf-8'))['version']
    except Exception as error:
      print(f"Failed to get version from {url}: {error}")

  print("Failed to get the latest mapd version")
  return None

def update_mapd():
  installed_version = get_installed_version()
  latest_version = get_latest_version()

  if latest_version is None:
    print("Could not get the latest mapd version")
    return

  if installed_version != latest_version:
    print("New mapd version available, stopping the mapd process for update")
    subprocess.run(["pkill", "-f", MAPD_PATH], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    if download(latest_version):
      print(f"Updated mapd to version {latest_version}")
    else:
      print("Failed to update mapd")
  else:
    print("Mapd is up to date")

def ensure_mapd_is_running():
  subprocess.run([MAPD_PATH], check=True)
