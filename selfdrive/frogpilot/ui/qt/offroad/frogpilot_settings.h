#pragma once

#include "selfdrive/ui/qt/offroad/settings.h"

class FrogPilotSettingsWindow : public QFrame {
  Q_OBJECT
public:
  explicit FrogPilotSettingsWindow(SettingsWindow *parent);

signals:
  void closeParentToggle();
  void closeSubParentToggle();
  void closeSubSubParentToggle();
  void openPanel();
  void openParentToggle();
  void openSubParentToggle();
  void openSubSubParentToggle();
  void updateMetric();

private:
  QStackedLayout *mainLayout;

  QWidget *frogpilotSettingsWidget;

  void addPanelControl(FrogPilotListWidget *list, const QString &title, const QString &desc, const std::vector<QString> &button_labels, const QString &icon, const std::vector<QWidget*> &panels);
};
