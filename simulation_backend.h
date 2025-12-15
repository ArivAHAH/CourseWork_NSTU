#pragma once

#include <QSqlError>
#include <QSqlQuery>
#include <QVariant>
#include <QObject>
#include <QSqlDatabase>
#include <QTimer>
#include <QProcessEnvironment>
#include <QDebug>

#include <algorithm>
#include <cmath>
#include <map>
#include <random>
#include <unordered_set>
#include <vector>
#include <unordered_map>

#include "traffic_core.h"

namespace app {

class SimulationBackend : public QObject
{
    Q_OBJECT

public:
    explicit SimulationBackend(QObject* parent = nullptr)
        : QObject(parent){}

    traffic::TrafficWorld& world() { return world_; }
    const traffic::TrafficWorld& world() const { return world_; }

    bool init();

    void setSimulationDateTime(int year, int month, int day, int hour, int minute);
    void setAccidentEnabled(bool enabled);
    void setRoadWorkEnabled(bool enabled);
    void setIceEnabled(bool enabled);
    void setLightsEnabled(bool enabled);

    void setManualVehicleTarget(int count);

    bool isPaused() const { return paused_; }
    void setPaused(bool paused);

signals:
    void worldUpdated();

private slots:
    void onTick();

private:
    void openDatabase(const QString& dbPath);
    void setupInitialConditions();
    void loadPlaces();
    void loadPaths();
    void loadTrafficLights();
    void registerVehicleConfigs();
    void spawnInitialTraffic(int count);
    void spawnDynamicTraffic(float dtSimSeconds);

    double hourlyFactor(const traffic::SimulationTime& t) const;
    double monthlyFactor(const traffic::SimulationTime& t) const;
    double weekdayFactor(const traffic::SimulationTime& t) const;
    int targetVehicleCount(const traffic::SimulationTime& t) const;

    void buildOdMatrix();
    double odWeight(const traffic::Place& from, const traffic::Place& to) const;
    double odTimeFactor(const traffic::Place& from,
                        const traffic::Place& to,
                        const traffic::SimulationTime& t) const;

    bool sampleOdPair(int& originWorldId, int& destWorldId, const traffic::SimulationTime& t) const;

private:
    int frameSkipCounter_ = 0;

    traffic::TrafficWorld world_;
    QSqlDatabase db_;
    QTimer timer_;

    traffic::VehicleConfigIds cfgIds_;
    std::unordered_map<int, int> placeIdMap_;

    float spawnAccumulator_ = 0.0f;

    bool manualAccidentEnabled_ = false;
    bool manualRoadWorkEnabled_ = false;
    bool manualIceEnabled_ = false;

    bool manualVehicleControl_ = false;
    int manualVehicleTarget_ = 0;

    struct ODEntry {
        int originWorldId;
        int destWorldId;
        double weight;
    };

    std::vector<ODEntry> odMatrix_;
    double odTotalWeight_ = 0.0;

    bool paused_ = true;
};

} 
