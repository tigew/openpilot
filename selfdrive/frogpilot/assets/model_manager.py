#!/usr/bin/env python3
import json
import re
import requests
import shutil
import time
import urllib.parse
import urllib.request

from pathlib import Path

from openpilot.selfdrive.frogpilot.assets.download_functions import GITLAB_URL, download_file, get_repository_url, handle_error, handle_request_error, verify_download
from openpilot.selfdrive.frogpilot.frogpilot_utilities import delete_file
from openpilot.selfdrive.frogpilot.frogpilot_variables import DEFAULT_MODEL, DEFAULT_CLASSIC_MODEL, MODELS_PATH, params, params_memory

VERSION = "v12"

CANCEL_DOWNLOAD_PARAM = "CancelModelDownload"
DOWNLOAD_PROGRESS_PARAM = "ModelDownloadProgress"
MODEL_DOWNLOAD_PARAM = "ModelToDownload"

class ModelManager:
  def __init__(self):
    self.downloading_model = False

  @staticmethod
  def fetch_models(url):
    try:
      with urllib.request.urlopen(url, timeout=10) as response:
        return json.loads(response.read().decode('utf-8'))['models']
    except Exception as error:
      handle_request_error(error, None, None, None, None)
      return []

  @staticmethod
  def fetch_all_model_sizes(repo_url):
    project_path = "FrogAi/FrogPilot-Resources"
    branch = "Models"

    if "github" in repo_url:
      api_url = f"https://api.github.com/repos/{project_path}/contents?ref={branch}"
    elif "gitlab" in repo_url:
      api_url = f"https://gitlab.com/api/v4/projects/{urllib.parse.quote_plus(project_path)}/repository/tree?ref={branch}"
    else:
      return {}

    try:
      response = requests.get(api_url)
      response.raise_for_status()
      thneed_files = [file for file in response.json() if file['name'].endswith('.thneed')]

      if "gitlab" in repo_url:
        model_sizes = {}
        for file in thneed_files:
          file_path = file['path']
          metadata_url = f"https://gitlab.com/api/v4/projects/{urllib.parse.quote_plus(project_path)}/repository/files/{urllib.parse.quote_plus(file_path)}/raw?ref={branch}"
          metadata_response = requests.head(metadata_url)
          metadata_response.raise_for_status()
          model_sizes[file['name'].replace('.thneed', '')] = int(metadata_response.headers.get('content-length', 0))
        return model_sizes
      else:
        return {file['name'].replace('.thneed', ''): file['size'] for file in thneed_files if 'size' in file}
    except Exception as error:
      handle_request_error(f"Failed to fetch model sizes from {'GitHub' if 'github' in repo_url else 'GitLab'}: {error}", None, None, None, None)
      return {}

  @staticmethod
  def fetch_all_metadata_sizes(repo_url):
    project_path = "FrogAi/FrogPilot-Resources"
    branch = "Model-Metadata"

    if "github" in repo_url:
      api_url = f"https://api.github.com/repos/{project_path}/contents?ref={branch}"
    elif "gitlab" in repo_url:
      api_url = f"https://gitlab.com/api/v4/projects/{urllib.parse.quote_plus(project_path)}/repository/tree?ref={branch}"
    else:
      return {}

    try:
      response = requests.get(api_url)
      response.raise_for_status()
      metadata_files = [file for file in response.json() if re.match(r'supercombo_metadata_v\d+\.pkl', file['name'])]

      if "gitlab" in repo_url:
        metadata_sizes = {}
        for file in metadata_files:
          file_path = file['path']
          metadata_url = f"https://gitlab.com/api/v4/projects/{urllib.parse.quote_plus(project_path)}/repository/files/{urllib.parse.quote_plus(file_path)}/raw?ref={branch}"
          metadata_response = requests.head(metadata_url)
          metadata_response.raise_for_status()
          metadata_sizes[file['name']] = int(metadata_response.headers.get('content-length', 0))
        return metadata_sizes
      else:
        return {file['name']: file['size'] for file in metadata_files if 'size' in file}
    except Exception as error:
      handle_request_error(f"Failed to fetch metadata sizes from {'GitHub' if 'github' in repo_url else 'GitLab'}: {error}", None, None, None, None)
      return {}

  def handle_verification_failure(self, model, model_path):
    print(f"Verification failed for model {model}. Retrying from GitLab...")
    model_url = f"{GITLAB_URL}/Models/{model}.thneed"
    download_file(CANCEL_DOWNLOAD_PARAM, model_path, DOWNLOAD_PROGRESS_PARAM, model_url, MODEL_DOWNLOAD_PARAM, params_memory)

    if params_memory.get_bool(CANCEL_DOWNLOAD_PARAM):
      handle_error(None, "Download cancelled...", "Download cancelled...", MODEL_DOWNLOAD_PARAM, DOWNLOAD_PROGRESS_PARAM, params_memory)
      self.downloading_model = False
      return

    if verify_download(model_path, model_url):
      print(f"Model {model} downloaded and verified successfully!")
      params_memory.put(DOWNLOAD_PROGRESS_PARAM, "Downloaded!")
      params_memory.remove(MODEL_DOWNLOAD_PARAM)
      self.downloading_model = False
    else:
      handle_error(model_path, "Verification failed...", "Gitlab verification failed", MODEL_DOWNLOAD_PARAM, DOWNLOAD_PROGRESS_PARAM, params_memory)
      self.downloading_model = False

  def download_model(self, model_to_download):
    self.downloading_model = True

    repo_url = get_repository_url()
    if not repo_url:
      handle_error(None, "GitHub and GitLab are offline...", "Repository unavailable", MODEL_DOWNLOAD_PARAM, DOWNLOAD_PROGRESS_PARAM, params_memory)
      self.downloading_model = False
      return

    model_path = MODELS_PATH / f"{model_to_download}.thneed"
    model_url = f"{repo_url}/Models/{model_to_download}.thneed"
    print(f"Downloading model: {model_to_download}")
    download_file(CANCEL_DOWNLOAD_PARAM, model_path, DOWNLOAD_PROGRESS_PARAM, model_url, MODEL_DOWNLOAD_PARAM, params_memory)

    if params_memory.get_bool(CANCEL_DOWNLOAD_PARAM):
      handle_error(None, "Download cancelled...", "Download cancelled...", MODEL_DOWNLOAD_PARAM, DOWNLOAD_PROGRESS_PARAM, params_memory)
      self.downloading_model = False
      return

    if verify_download(model_path, model_url):
      print(f"Model {model_to_download} downloaded and verified successfully!")
      params_memory.put(DOWNLOAD_PROGRESS_PARAM, "Downloaded!")
      params_memory.remove(MODEL_DOWNLOAD_PARAM)
      self.downloading_model = False
    else:
      self.handle_verification_failure(model_to_download, model_path)

  def download_missing_metadata(self, repo_url):
    metadata_sizes = self.fetch_all_metadata_sizes(repo_url)
    if not metadata_sizes:
      print("No metadata size data available. Skipping metadata downloads.")
      return

    for metadata_file, expected_size in metadata_sizes.items():
      metadata_path = MODELS_PATH / metadata_file

      if metadata_path.is_file():
        local_size = metadata_path.stat().st_size
        if local_size == expected_size:
          continue
        print(f"Metadata {metadata_file} is outdated. Deleting...")
        delete_file(metadata_path)

      metadata_url = f"{repo_url}/Model-Metadata/{metadata_file}"
      print(f"Downloading metadata: {metadata_file}")
      download_file(CANCEL_DOWNLOAD_PARAM, metadata_path, DOWNLOAD_PROGRESS_PARAM, metadata_url, MODEL_DOWNLOAD_PARAM, params_memory)

      if verify_download(metadata_path, metadata_url):
        print(f"Metadata {metadata_file} downloaded and verified successfully!")
      else:
        handle_error(metadata_path, "Verification failed...", "Metadata verification failed", MODEL_DOWNLOAD_PARAM, DOWNLOAD_PROGRESS_PARAM, params_memory)

  @staticmethod
  def copy_default_model():
    classic_default_model_path = MODELS_PATH / f"{DEFAULT_CLASSIC_MODEL}.thneed"
    source_path = Path(__file__).parents[2] / "selfdrive/classic_modeld/models/supercombo.thneed"
    if source_path.is_file() and not classic_default_model_path.is_file():
      shutil.copyfile(source_path, classic_default_model_path)
      print(f"Copied the classic default model from {source_path} to {classic_default_model_path}")

    default_model_path = MODELS_PATH / f"{DEFAULT_MODEL}.thneed"
    source_path = Path(__file__).parents[2] / "selfdrive/modeld/models/supercombo.thneed"
    if source_path.is_file() and not default_model_path.is_file():
      shutil.copyfile(source_path, default_model_path)
      print(f"Copied the default model from {source_path} to {default_model_path}")

  def check_models(self, available_models, boot_run, repo_url):
    available_models = set(available_models) - {DEFAULT_MODEL, DEFAULT_CLASSIC_MODEL}
    downloaded_models = set(path.stem for path in MODELS_PATH.glob("*.thneed")) - {DEFAULT_MODEL, DEFAULT_CLASSIC_MODEL}

    outdated_models = downloaded_models - available_models
    for model in outdated_models:
      model_path = MODELS_PATH / f"{model}.thneed"
      print(f"Removing outdated model: {model}")
      delete_file(model_path)

    for tmp_file in MODELS_PATH.glob("tmp*"):
      if tmp_file.is_file():
        delete_file(tmp_file)

    automatically_update_models = not boot_run and params.get_bool("AutomaticallyUpdateModels")
    if not automatically_update_models:
      return

    model_sizes = self.fetch_all_model_sizes(repo_url)
    if not model_sizes:
      print("No model size data available. Skipping model checks")
      return

    for model in available_models:
      model_path = MODELS_PATH / f"{model}.thneed"
      expected_size = model_sizes.get(model)

      if expected_size is None:
        print(f"Size data for {model} not available.")
        continue

      if model_path.is_file():
        local_size = model_path.stat().st_size
        if local_size == expected_size:
          continue
        print(f"Model {model} is outdated. Deleting...")
        delete_file(model_path)

    self.download_all_models()

  def update_model_params(self, model_info, repo_url):
    available_models = []
    for model in model_info:
      available_models.append(model['id'])

    params.put("AvailableModels", ",".join(available_models))
    params.put("AvailableModelNames", ",".join([model['name'] for model in model_info]))
    params.put("ExperimentalModels", ",".join([model['id'] for model in model_info if model.get("experimental", False)]))
    params.put("ModelVersions", ",".join([model['version'] for model in model_info if model.get("version", "v0")]))
    print("Models list updated successfully")

    return available_models

  def update_models(self, boot_run=False):
    if self.downloading_model:
      return

    if boot_run:
      self.copy_default_model()

    repo_url = get_repository_url()
    if repo_url is None:
      print("GitHub and GitLab are offline...")
      return

    model_info = self.fetch_models(f"{repo_url}/Versions/model_names_{VERSION}.json")
    if model_info:
      available_models = self.update_model_params(model_info, repo_url)
      self.check_models(available_models, boot_run, repo_url)
      self.download_missing_metadata(repo_url)

  def queue_model_download(self, model, model_name=None):
    while params_memory.get(MODEL_DOWNLOAD_PARAM, encoding='utf-8'):
      time.sleep(1)

    params_memory.put(MODEL_DOWNLOAD_PARAM, model)
    if model_name:
      params_memory.put(DOWNLOAD_PROGRESS_PARAM, f"Downloading \"{model_name}\"...")

  def download_all_models(self):
    repo_url = get_repository_url()
    if not repo_url:
      handle_error(None, "GitHub and GitLab are offline...", "Repository unavailable", MODEL_DOWNLOAD_PARAM, DOWNLOAD_PROGRESS_PARAM, params_memory)
      return

    model_info = self.fetch_models(f"{repo_url}/Versions/model_names_{VERSION}.json")
    if model_info:
      available_models = [model["id"] for model in model_info]
      available_model_names = [re.sub(r'[🗺️👀📡]', '', model["name"]).strip() for model in model_info]
      for model_id, model_name in zip(available_models, available_model_names):
        model_path = MODELS_PATH / f"{model_id}.thneed"
        if not model_path.is_file():
          print(f"Model {model_id} does not exist. Preparing to download...")

          if params_memory.get_bool(CANCEL_DOWNLOAD_PARAM):
            handle_error(None, "Download cancelled...", "Download cancelled...", MODEL_DOWNLOAD_PARAM, DOWNLOAD_PROGRESS_PARAM, params_memory)
            return

          self.queue_model_download(model_id, model_name)

      while not all((MODELS_PATH / f"{model}.thneed").is_file() for model in available_models):
        if params_memory.get_bool(CANCEL_DOWNLOAD_PARAM):
          handle_error(None, "Download cancelled...", "Download cancelled...", MODEL_DOWNLOAD_PARAM, DOWNLOAD_PROGRESS_PARAM, params_memory)
          return
        time.sleep(1)

      params_memory.put(DOWNLOAD_PROGRESS_PARAM, "All models downloaded!")
    else:
      handle_error(None, "Unable to fetch models...", "Model list unavailable", MODEL_DOWNLOAD_PARAM, DOWNLOAD_PROGRESS_PARAM, params_memory)
      return
