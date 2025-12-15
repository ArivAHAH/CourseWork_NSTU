#pragma once

#include "intro_widget.h"
#include "simulation_backend.h"
#include "simulation_view.h"
#include "build_mode_view.h"

#include <QCheckBox>
#include <QDateTime>
#include <QDateTimeEdit>
#include <QDialog>
#include <QDialogButtonBox>
#include <QGraphicsOpacityEffect>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPropertyAnimation>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>
#include <QWidget>

#include <QEasingCurve>
#include <QMainWindow>

class QDialog;
class QDateTimeEdit;
class QSlider;
class QLabel;
class QCheckBox;

namespace app {
class SimulationBackend;
class SimulationView;
class BuildModeView;
}

class IntroWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr)
        : QMainWindow(parent)
    {
        setupUi();
        applyStyle();
        initBackend();
        setupConnections();
    }

private:
    void setupUi();
    void applyStyle();
    void setupConnections();
    void initBackend();
    void createSettingsDialog();
    void openSettingsDialog();

    void handleStartRequested();
    void handleSimulationHomeRequested();
    void handleBuildModeRequested();
    void openBuildModeSettingsDialog();

private:
    app::SimulationBackend* backend_ = nullptr;
    app::SimulationView* simulationView_ = nullptr;
    app::BuildModeView* buildModeView_ = nullptr;
    IntroWidget* introWidget_ = nullptr;

    QDialog* settingsDialog_ = nullptr;
    QDateTimeEdit* timeEdit_ = nullptr;
    QSlider* vehiclesSlider_ = nullptr;
    QLabel* vehiclesValueLabel_ = nullptr;

    QCheckBox* accidentCheck_ = nullptr;
    QCheckBox* iceCheck_ = nullptr;
    QCheckBox* roadWorkCheck_ = nullptr;
    QCheckBox* lightsCheck_ = nullptr;

    bool simulationStarted_ = false;
};
