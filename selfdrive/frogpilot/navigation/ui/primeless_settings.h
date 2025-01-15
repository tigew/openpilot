#pragma once

#include "selfdrive/frogpilot/ui/qt/offroad/frogpilot_settings.h"

class FrogPilotPrimelessPanel : public FrogPilotListWidget {
  Q_OBJECT

public:
  explicit FrogPilotPrimelessPanel(FrogPilotSettingsWindow *parent);

signals:
  void closeMapBoxInstructions();
  void openMapBoxInstructions();

private:
  void createMapboxKeyControl(ButtonControl *&control, const QString &label, const std::string &paramKey, const QString &prefix);
  void displayMapboxInstructions(bool visible);
  void hideEvent(QHideEvent *event);
  void mousePressEvent(QMouseEvent *event);
  void showEvent(QShowEvent *event);
  void updateState();
  void updateStep();

  bool mapboxPublicKeySet;
  bool mapboxSecretKeySet;
  bool setupCompleted;

  ButtonControl *publicMapboxKeyControl;
  ButtonControl *secretMapboxKeyControl;
  ButtonControl *setupButton;

  ButtonParamControl *searchInput;

  FrogPilotSettingsWindow *parent;

  LabelControl *ipLabel;

  Params params;
  Params paramsStorage{"/persist/params"};

  QLabel *imageLabel;
};
