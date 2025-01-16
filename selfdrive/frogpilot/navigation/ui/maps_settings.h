#pragma once

#include "selfdrive/frogpilot/navigation/ui/navigation_functions.h"
#include "selfdrive/frogpilot/ui/qt/offroad/frogpilot_settings.h"

class FrogPilotMapsPanel : public FrogPilotListWidget {
  Q_OBJECT

public:
  explicit FrogPilotMapsPanel(FrogPilotSettingsWindow *parent);

signals:
  void openMapSelection();

private:
  void cancelDownload();
  void startDownload();
  void updateState(const UIState &s);

  bool downloadActive;

  ButtonControl *downloadMapsButton;
  ButtonControl *removeMapsButton;

  FrogPilotSettingsWindow *parent;

  LabelControl *downloadETA;
  LabelControl *downloadStatus;
  LabelControl *downloadTimeElapsed;
  LabelControl *lastMapsDownload;
  LabelControl *mapsSize;

  Params params;
  Params params_memory{"/dev/shm/params"};

  QString mapsFolderPath;

  QStackedLayout *mapsLayout;
};
