#pragma once

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <cstdint>
#include <limits>
#include <cmath>
#include <algorithm>
#include <queue>
#include <random>
#include <sstream>
#include <utility>
#include <unordered_set>

namespace traffic {

struct VehicleConfigIds {
    int passenger = 0;
    int taxi      = 0;
    int bus       = 0;
    int truck     = 0;
};

enum class Weekday : std::uint8_t {
    Monday = 1, Tuesday, Wednesday, Thursday, Friday, Saturday, Sunday
};

enum class Month : std::uint8_t {
    January = 1, February, March, April, May, June,
    July, August, September, October, November, December
};

enum class TurnType : std::uint8_t {
    Right,
    Straight,
    Left,
    UTurn
};

enum class Season : std::uint8_t {
    Winter,
    Spring,
    Summer,
    Autumn
};

enum class TimeOfDay : std::uint8_t {
    Night,
    Morning,
    Day,
    Evening
};

enum class VehicleKind : std::uint8_t {
    Passenger,
    Taxi,
    Bus,
    TruckLight,
    TruckHeavy
};

enum class PlaceCategory : std::uint8_t {
    Residential,
    Mall,
    Campus,
    MetroStation,
    Office,
    Intersection,
    Other
};

struct SimulationTime {
    int     year    {2025}; 
    Weekday weekday {Weekday::Monday};
    Month   month   {Month::September};
    int     day     {1};
    int     hour    {12};
    int     minute  {0};

    TimeOfDay timeOfDay() const;
    bool isWeekend() const;
    Season season() const;
};

struct TrafficConditions {
    SimulationTime time;

    bool accident {false};
    bool roadWork {false};
    bool ice      {false};

    float speedFactor() const;
};

struct Vec2 {
    float x {0.0f};
    float y {0.0f};
};

struct Place {
    int id {0};
    std::string name;
    PlaceCategory category {PlaceCategory::Other};
    Vec2 position;
};

struct Intersection {
    int placeId {0};

    std::vector<int> incomingPaths;
    std::vector<int> outgoingPaths;

    std::unordered_map<int, int> pathPriority;

    struct Movement {
        int inPathId  {0};
        int outPathId {0};
        TurnType turn {TurnType::Straight};
    };

    std::vector<Movement> movements;
};

struct Path {
    int id {0};
    int originPlaceId {0};
    int destinationPlaceId {0};

    std::vector<Vec2> points;
    float length {0.0f};

    float speedLimitMps {std::numeric_limits<float>::infinity()};

    bool hasAccident {false};
    bool hasRoadWork {false};
    bool hasIce      {false};

    int   laneCount {2};
    float laneWidth {3.5f};

    float congestionFactor {1.0f};

    inline float localSpeedFactor() const {
        float f = congestionFactor;
        if (hasAccident) f *= 0.5f;
        if (hasRoadWork) f *= 0.7f;
        if (hasIce)      f *= 0.6f;
        return std::max(f, 0.2f);
    }

    void recomputeLength();
    Vec2 positionAt(float distance) const;
};

enum class LightColor : std::uint8_t {
    Red,
    Yellow,
    Green
};

struct TrafficLight {
    int   id {0};
    Vec2  position;
    int   pathId {0};
    float distanceOnPath {0.0f};
    int   groupId {0};
};

struct VehicleConfig {
    int id {0};
    std::string name;
    VehicleKind kind {VehicleKind::Passenger};
    std::string modelId;
    float baseSpeedMps {30.0f};
    float length {4.0f};
    float width  {1.8f};
};

class Vehicle {
public:
    inline Vehicle(
        int id,
        const Place* origin,
        const Place* destination,
        std::vector<const Path*> route,
        const VehicleConfig& config
    )
        : id_(id)
        , origin_(origin)
        , destination_(destination)
        , route_(std::move(route))
        , config_(config)
    {
        speed_ = 0.0f;

        if (route_.empty()) {
            finished_ = true;
            currentSegmentIndex_ = 0;
            distance_ = 0.0f;
        } else {
            finished_ = false;
            currentSegmentIndex_ = 0;
            distance_ = 0.0f;
        }

        switch (config_.kind) {
            case VehicleKind::Bus:
            case VehicleKind::TruckLight:
            case VehicleKind::TruckHeavy:
                maxAccel_ = 1.0f;
                maxDecel_ = 3.0f;
                break;
            default:
                maxAccel_ = 2.0f;
                maxDecel_ = 4.0f;
                break;
        }
    }

    int id() const { return id_; }

    const Place* origin() const { return origin_; }
    const Place* destination() const { return destination_; }

    const Path* path() const { return currentPath(); }

    const VehicleConfig& config() const { return config_; }

    float traveledDistance() const { return distance_; }
    float remainingDistance() const;

    Vec2 position() const;

    float remainingTimeSeconds(const TrafficConditions& conditions) const;
    float estimatedSpeedMps(const TrafficConditions& conditions) const;

    bool finished() const { return finished_; }

    int laneIndex() const { return laneIndex_; }
    void setLaneIndex(int lane) { laneIndex_ = lane; }

    void step(
        float dtSeconds,
        const TrafficConditions& conditions,
        const std::vector<TrafficLight>& lights,
        const std::vector<LightColor>& colors,
        float maxDistance = std::numeric_limits<float>::infinity()
    );

    const Path* nextPathInRoute() const;

    void forceSetDistanceAndStop(float newDistance);

private:
    const Path* currentPath() const;
    float effectiveSpeed(const TrafficConditions& conditions) const;

    int id_ {0};

    const Place* origin_ {nullptr};
    const Place* destination_ {nullptr};

    std::vector<const Path*> route_;
    int   currentSegmentIndex_ {0};
    float distance_ {0.0f};

    VehicleConfig config_;
    float driverFactor_ {1.0f};

    bool finished_ {false};

    float speed_ {0.0f};
    float maxAccel_ {2.0f};
    float maxDecel_ {4.0f};

    int laneIndex_ {0};
};

class TrafficWorld {
public:
    inline TrafficWorld()
        : rng_(std::random_device{}())
    {
        secondsWithinDay_ =
            conditions_.time.hour * 3600.0f +
            conditions_.time.minute * 60.0f;
    }

    void setConditions(const TrafficConditions& conditions);
    const TrafficConditions& conditions() const { return conditions_; }

    void setAutoMode(bool enabled) { autoMode_ = enabled; }
    bool autoMode() const { return autoMode_; }

    void setTimeScale(float s) { timeScale_ = s; }
    float timeScale() const { return timeScale_; }

    const SimulationTime& simulationTime() const { return conditions_.time; }

    void setLightsEnabled(bool enabled) { lightsEnabled_ = enabled; }
    bool lightsEnabled() const { return lightsEnabled_; }

    void setManualAccident(bool enabled);
    void setManualRoadWork(bool enabled);
    void setManualIce(bool enabled);

    int registerVehicleConfig(const VehicleConfig& cfg);
    const VehicleConfig* getVehicleConfig(int configId) const;

    int addPlace(
        const std::string& name,
        PlaceCategory category,
        const Vec2& pos
    );

    const Place* getPlace(int id) const;

    int addPath(
        int originPlaceId,
        int destinationPlaceId,
        const std::vector<Vec2>& points
    );

    const Path* getPath(int id) const;
    const Path* findPath(int originPlaceId, int destinationPlaceId) const;

    void setPathSpeedLimit(int pathId, float speedLimitKph);

    int addTrafficLight(int dbId, const Vec2& pos);
    const std::vector<TrafficLight>& trafficLights() const { return trafficLights_; }

    LightColor trafficLightColor(const TrafficLight& light) const;

    int spawnVehicle(
        int originPlaceId,
        int destinationPlaceId,
        int vehicleConfigId
    );

    void step(float dtSeconds);

    const std::vector<std::unique_ptr<Vehicle>>& vehicles() const { return vehicles_; }

    const Vehicle* getVehicle(int vehicleId) const;

    std::string buildTooltipForVehicle(int vehicleId) const;

    void removeRandomVehicles(int count);

    void setPathAccident(int pathId, bool on);
    void clearAllAccidents();

    void setPathRoadWork(int pathId, bool on);
    void clearAllRoadWorks();

    void setPathIce(int pathId, bool on);
    void clearAllIce();

    const std::unordered_map<int, Path>& paths() const { return paths_; }

    void buildIntersections();
    const std::vector<Intersection>& intersections() const { return intersections_; }

    float pathCongestionFactor(int pathId) const;

private:
    void advanceTimeAndEvents(float dtSimSeconds);

    int daysInMonth(Month m) const;
    Month nextMonth(Month m) const;
    Weekday nextWeekday(Weekday w) const;

    void updateConditionsFlags();

    bool buildRoute(
        int originPlaceId,
        int destinationPlaceId,
        std::vector<const Path*>& out
    ) const;

    int nextPlaceId_ {1};
    int nextPathId_ {1};
    int nextVehicleId_ {1};
    int nextVehicleConfigId_ {1};

    std::vector<TrafficLight> trafficLights_;

    float simSeconds_ {0.0f};

    TrafficConditions conditions_;

    bool  autoMode_ {true};
    float timeScale_ {60.0f};

    float secondsWithinDay_ {0.0f};
    int   daysSinceStart_ {0};

    float accidentSecondsLeft_ {0.0f};
    float roadWorkSecondsLeft_ {0.0f};
    float iceSecondsLeft_ {0.0f};

    std::mt19937 rng_;

    std::unordered_map<int, Place> places_;
    std::unordered_map<int, Path> paths_;
    std::unordered_map<int, VehicleConfig> vehicleConfigs_;
    std::vector<std::unique_ptr<Vehicle>> vehicles_;
    std::vector<Intersection> intersections_;
    std::unordered_map<int, float> pathVehicleDensity_;

    bool autoAccident_ {false};
    bool autoRoadWork_ {false};
    bool autoIce_ {false};

    bool manualAccident_ {false};
    bool manualRoadWork_ {false};
    bool manualIce_ {false};

    bool lightsEnabled_ {true};
};

}
