#include <regex>

#include <QtConcurrent>

#include "selfdrive/frogpilot/navigation/ui/maps_settings.h"

FrogPilotMapsPanel::FrogPilotMapsPanel(FrogPilotSettingsWindow *parent) : FrogPilotListWidget(parent), parent(parent), mapsFolderPath{"/data/media/0/osm/offline"} {
  QVBoxLayout *mainLayout = new QVBoxLayout();
  addItem(mainLayout);

  mapsLayout = new QStackedLayout();
  mainLayout->addLayout(mapsLayout);

  FrogPilotListWidget *settingsList = new FrogPilotListWidget(this);

  std::vector<QString> scheduleOptions{tr("Manually"), tr("Weekly"), tr("Monthly")};
  ButtonParamControl *preferredSchedule = new ButtonParamControl("PreferredSchedule", tr("Automatically Update Maps"),
                                          tr("Controls the frequency at which maps update with the latest OpenStreetMap (OSM) changes. "
                                             "Weekly updates begin at midnight every Sunday, while monthly updates start at midnight on the 1st of each month."),
                                             "",
                                             scheduleOptions);
  settingsList->addItem(preferredSchedule);

  std::vector<QString> mapOptions{tr("COUNTRIES"), tr("STATES")};
  FrogPilotButtonsControl *selectMaps = new FrogPilotButtonsControl(tr("Select Map Data Sources"),
                                        tr("Select map data sources to use with 'Curve Speed Control' and 'Speed Limit Controller'."),
                                           mapOptions);
  QObject::connect(selectMaps, &FrogPilotButtonsControl::buttonClicked, [this](int id) {
    mapsLayout->setCurrentIndex(id + 1);
    openMapSelection();
  });
  settingsList->addItem(selectMaps);

  downloadMapsButton = new ButtonControl(tr("Download Maps"), tr("DOWNLOAD"), tr("Downloads the selected maps to use with 'Curve Speed Control' and 'Speed Limit Controller'."));
  QObject::connect(downloadMapsButton, &ButtonControl::clicked, [this] {
    if (downloadMapsButton->text() == tr("CANCEL")) {
      if (FrogPilotConfirmationDialog::yesorno(tr("Are you sure you want to cancel the download?"), this)) {
        cancelDownload();
      }
    } else {
      startDownload();
    }
  });
  settingsList->addItem(downloadMapsButton);

  settingsList->addItem(downloadETA = new LabelControl(tr("Download Completion ETA")));
  settingsList->addItem(downloadStatus = new LabelControl(tr("Download Progress")));
  settingsList->addItem(downloadTimeElapsed = new LabelControl(tr("Download Time Elapsed")));
  settingsList->addItem(lastMapsDownload = new LabelControl(tr("Maps Last Updated"), params.get("LastMapsUpdate").empty() ? "Never" : QString::fromStdString(params.get("LastMapsUpdate"))));
  settingsList->addItem(mapsSize = new LabelControl(tr("Downloaded Maps Size"), calculateDirectorySize(mapsFolderPath)));

  downloadETA->setVisible(false);
  downloadStatus->setVisible(false);
  downloadTimeElapsed->setVisible(false);

  removeMapsButton = new ButtonControl(tr("Remove Maps"), tr("REMOVE"), tr("Removes downloaded maps to clear up storage space."));
  removeMapsButton->setVisible(QDir(mapsFolderPath).exists());
  settingsList->addItem(removeMapsButton);

  ScrollView *settingsPanel = new ScrollView(settingsList, this);
  mapsLayout->addWidget(settingsPanel);

  FrogPilotListWidget *countriesList = new FrogPilotListWidget(this);
  std::vector<std::pair<QString, QMap<QString, QString>>> countries = {
    {tr("Africa"), africaMap},
    {tr("Antarctica"), antarcticaMap},
    {tr("Asia"), asiaMap},
    {tr("Europe"), europeMap},
    {tr("North America"), northAmericaMap},
    {tr("Oceania"), oceaniaMap},
    {tr("South America"), southAmericaMap}
  };

  for (std::pair<QString, QMap<QString, QString>> country : countries) {
    countriesList->addItem(new LabelControl(country.first, ""));
    countriesList->addItem(new MapSelectionControl(country.second, true));
  }

  ScrollView *countryMapsPanel = new ScrollView(countriesList, this);
  mapsLayout->addWidget(countryMapsPanel);

  FrogPilotListWidget *statesList = new FrogPilotListWidget(this);
  std::vector<std::pair<QString, QMap<QString, QString>>> states = {
    {tr("United States - Midwest"), midwestMap},
    {tr("United States - Northeast"), northeastMap},
    {tr("United States - South"), southMap},
    {tr("United States - West"), westMap},
    {tr("United States - Territories"), territoriesMap}
  };

  for (std::pair<QString, QMap<QString, QString>> state : states) {
    statesList->addItem(new LabelControl(state.first, ""));
    statesList->addItem(new MapSelectionControl(state.second));
  }

  ScrollView *stateMapsPanel = new ScrollView(statesList, this);
  mapsLayout->addWidget(stateMapsPanel);

  QObject::connect(parent, &FrogPilotSettingsWindow::closeMapSelection, [this] {mapsLayout->setCurrentIndex(0);});
  QObject::connect(uiState(), &UIState::uiUpdate, this, &FrogPilotMapsPanel::updateState);
}

void FrogPilotMapsPanel::updateState(const UIState &s) {
  if (!isVisible()) {
    return;
  }

  if (downloadActive && s.sm->frame % (UI_FREQ / 2) == 0) {
  }

  parent->keepScreenOn = downloadActive;
}

void FrogPilotMapsPanel::cancelDownload() {
  std::system("pkill mapd");

  downloadMapsButton->setText(tr("DOWNLOAD"));

  downloadETA->setVisible(false);
  downloadStatus->setVisible(false);
  downloadTimeElapsed->setVisible(false);

  lastMapsDownload->setVisible(true);
  removeMapsButton->setVisible(true);

  downloadActive = true;

  params_memory.remove("OSMDownloadLocations");
}

void FrogPilotMapsPanel::startDownload() {
  downloadMapsButton->setText(tr("CANCEL"));

  downloadETA->setVisible(true);
  downloadStatus->setVisible(true);
  downloadTimeElapsed->setVisible(true);

  lastMapsDownload->setVisible(false);
  removeMapsButton->setVisible(false);

  downloadActive = true;

  params_memory.put("OSMDownloadLocations", params.get("MapsSelected"));
}
