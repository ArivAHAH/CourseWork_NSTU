#include "simulation_backend.h"

using namespace traffic;

namespace app {

//загружаем все
bool SimulationBackend::init()
{
    openDatabase(QString());
    setupInitialConditions();
    loadPlaces();
    loadPaths();
    world_.buildIntersections();

    buildOdMatrix();
    loadTrafficLights();
    registerVehicleConfigs();

    spawnInitialTraffic(100);

    connect(&timer_, &QTimer::timeout, this, &SimulationBackend::onTick);
    timer_.start(70);

    paused_ = true;
    return true;
}

// открываем соединение в БДшкой
void SimulationBackend::openDatabase(const QString&)
{
    if (QSqlDatabase::contains("traffic")) {
        db_ = QSqlDatabase::database("traffic");
    } else {
        db_ = QSqlDatabase::addDatabase("QPSQL", "traffic");
    }

    const auto env = QProcessEnvironment::systemEnvironment();

    const QString host = env.value("DB_HOST", "db");
    const int port = env.value("DB_PORT", "5432").toInt();
    const QString name = env.value("DB_NAME", "traffic");
    const QString user = env.value("DB_USER");   // без дефолта
    const QString pass = env.value("DB_PASS");   // без дефолта

    if (user.isEmpty() || pass.isEmpty()) {
        qWarning() << "DB_USER/DB_PASS is not set";
        return;
    }

    db_.setHostName(host);
    db_.setPort(port);
    db_.setDatabaseName(name);
    db_.setUserName(user);
    db_.setPassword(pass);

    db_.open();
}

// стартовые настройки
void SimulationBackend::setupInitialConditions()
{
    SimulationTime startTime;
    startTime.year = 2025;
    startTime.weekday = Weekday::Tuesday;
    startTime.month = Month::December;
    startTime.day = 1;
    startTime.hour = 17;
    startTime.minute = 0;

    TrafficConditions cond;
    cond.time = startTime;
    cond.ice = false;
    cond.roadWork = false;
    cond.accident = false;

    world_.setConditions(cond);
    world_.setAutoMode(true);
    world_.setTimeScale(4.0f);
}

// загружаем места из БДшки и сопоставляем внешние id  с внутренними
void SimulationBackend::loadPlaces()
{
    QSqlQuery query(db_);
    if (!query.exec("SELECT id, name, category, x, y FROM places")) {
        return;
    }

    placeIdMap_.clear();

    while (query.next()) {
        int dbId = query.value(0).toInt();
        QString name = query.value(1).toString();
        int catCode = query.value(2).toInt();
        double x = query.value(3).toDouble();
        double y = query.value(4).toDouble();

        PlaceCategory cat = PlaceCategory::Other;
        switch (catCode) {
        case 0: cat = PlaceCategory::Residential; break;
        case 1: cat = PlaceCategory::Mall; break;
        case 3: cat = PlaceCategory::MetroStation; break;
        case 5: cat = PlaceCategory::Intersection; break;
        case 6: cat = PlaceCategory::Intersection; break;
        default: cat = PlaceCategory::Other; break;
        }

        Vec2 pos;
        pos.x = static_cast<float>(x);
        pos.y = static_cast<float>(y);

        int worldId = world_.addPlace(name.toStdString(), cat, pos);
        placeIdMap_[dbId] = worldId;
    }
}

// загружаем дорожные пути и их полилинии из бд
void SimulationBackend::loadPaths()
{
    QSqlQuery query(db_);
    if (!query.exec("SELECT id, origin_place_id, destination_place_id, speed_limit_kph FROM paths")) {
        return;
    }

    int count = 0;

    while (query.next()) {
        int dbPathId = query.value(0).toInt();
        int dbFrom = query.value(1).toInt();
        int dbTo = query.value(2).toInt();
        double limitKph = query.value(3).toDouble();

        auto itFrom = placeIdMap_.find(dbFrom);
        auto itTo = placeIdMap_.find(dbTo);
        if (itFrom == placeIdMap_.end() || itTo == placeIdMap_.end()) {
            continue;
        }

        const Place* pFrom = world_.getPlace(itFrom->second);
        const Place* pTo = world_.getPlace(itTo->second);
        if (!pFrom || !pTo) {
            continue;
        }

        std::vector<Vec2> points;

        QSqlQuery pointsQuery(db_);
        pointsQuery.prepare(
            "SELECT x, y FROM path_points "
            "WHERE path_id = ? ORDER BY order_index");
        pointsQuery.addBindValue(dbPathId);

        if (pointsQuery.exec()) {
            while (pointsQuery.next()) {
                Vec2 p;
                p.x = static_cast<float>(pointsQuery.value(0).toDouble());
                p.y = static_cast<float>(pointsQuery.value(1).toDouble());
                points.push_back(p);
            }
        }

        if (points.size() < 2) {
            points.clear();
            points.push_back(pFrom->position);
            points.push_back(pTo->position);
        }

        int pathId = world_.addPath(itFrom->second, itTo->second, points);

        if (limitKph > 0.0) {
            world_.setPathSpeedLimit(pathId, static_cast<float>(limitKph));
        }

        ++count;
    }
}

// считает базовый вес OD-пары по категориям мест
double SimulationBackend::odWeight(const traffic::Place& from, const traffic::Place& to) const
{
    using traffic::PlaceCategory;

    if (from.id == to.id) {
        return 0.0;
    }

    double w = 1.0;

    auto cf = from.category;
    auto ct = to.category;

    auto isHub = [](PlaceCategory c) {
        return c == PlaceCategory::Mall || c == PlaceCategory::MetroStation || c == PlaceCategory::Intersection;
    };

    auto strongPair = [&](PlaceCategory a, PlaceCategory b) {
        return (a == PlaceCategory::Residential &&
                (b == PlaceCategory::Office || b == PlaceCategory::Campus || b == PlaceCategory::Mall ||
                 b == PlaceCategory::MetroStation));
    };

    if (strongPair(cf, ct) || strongPair(ct, cf)) {
        w *= 8.0;
    }

    if (cf == PlaceCategory::Residential && ct == PlaceCategory::Residential) {
        w *= 3.0;
    }

    if (isHub(cf) && isHub(ct)) {
        w *= 0.5;
    }

    if (w < 0.0) {
        w = 0.0;
    }

    return w;
}

// корректирует вес OD-пары в зависимости от времени 
double SimulationBackend::odTimeFactor(const traffic::Place& from,
                                       const traffic::Place& to,
                                       const traffic::SimulationTime& t) const
{
    using traffic::PlaceCategory;

    double f = 1.0;

    const auto cf = from.category;
    const auto ct = to.category;

    const int hour = t.hour;
    const bool weekend = t.isWeekend();

    auto isHome = [](PlaceCategory c) { return c == PlaceCategory::Residential; };
    auto isWork = [](PlaceCategory c) { return c == PlaceCategory::Office || c == PlaceCategory::Campus; };
    auto isLeisure = [](PlaceCategory c) { return c == PlaceCategory::Mall || c == PlaceCategory::MetroStation; };

    const bool homeToWork = isHome(cf) && isWork(ct);
    const bool workToHome = isWork(cf) && isHome(ct);
    const bool homeToFun = isHome(cf) && isLeisure(ct);
    const bool funToHome = isLeisure(cf) && isHome(ct);
    const bool homeToHome = isHome(cf) && isHome(ct);
    const bool workToWork = isWork(cf) && isWork(ct);

    const bool morning = (hour >= 7 && hour < 10);
    const bool day = (hour >= 10 && hour < 17);
    const bool evening = (hour >= 17 && hour < 21);
    const bool night = (hour >= 22 || hour < 5);

    if (night) {
        f *= 0.3;
    }

    if (!weekend) {
        if (morning) {
            if (homeToWork) f *= 5.0;
            else if (homeToFun) f *= 3.0;
            else if (homeToHome) f *= 1.5;
            else if (workToHome) f *= 0.7;
        } else if (day) {
            if (homeToWork) f *= 1.5;
            if (workToWork) f *= 1.5;
            if (homeToFun || funToHome) f *= 2.0;
        } else if (evening) {
            if (workToHome) f *= 5.0;
            else if (funToHome) f *= 3.0;
            else if (homeToHome) f *= 1.5;
            else if (homeToWork) f *= 0.7;
        }
    } else {
        if (morning || day) {
            if (homeToFun) f *= 4.0;
            if (funToHome) f *= 2.0;
            if (homeToHome) f *= 1.5;
            if (homeToWork || workToHome) f *= 0.3;
        } else if (evening) {
            if (funToHome) f *= 3.0;
            if (workToHome) f *= 1.5;
        }
    }

    if (f < 0.1) f = 0.1;
    if (f > 10.0) f = 10.0;

    return f;
}

// строит матрицу для выпора направлений спавна
void SimulationBackend::buildOdMatrix()
{
    odMatrix_.clear();
    odTotalWeight_ = 0.0;

    const auto& paths = world_.paths();
    if (paths.empty()) {
        return;
    }

    std::map<std::pair<int, int>, double> pairWeights;
    std::unordered_set<int> allPlacesSet;

    for (const auto& kv : paths) {
        const auto& p = kv.second;

        int o = p.originPlaceId;
        int d = p.destinationPlaceId;

        const Place* from = world_.getPlace(o);
        const Place* to = world_.getPlace(d);
        if (!from || !to) {
            continue;
        }

        double w = odWeight(*from, *to);
        if (w <= 0.0) {
            continue;
        }

        pairWeights[{o, d}] += w;
        allPlacesSet.insert(o);
        allPlacesSet.insert(d);
    }

    if (pairWeights.empty() || allPlacesSet.empty()) {
        odMatrix_.clear();
        odTotalWeight_ = 0.0;
        return;
    }

    std::vector<int> allPlaces(allPlacesSet.begin(), allPlacesSet.end());

    std::unordered_map<int, int> outCount;
    std::unordered_map<int, int> inCount;

    for (const auto& kv : pairWeights) {
        int o = kv.first.first;
        int d = kv.first.second;
        outCount[o]++;
        inCount[d]++;
    }

    if (allPlaces.size() > 1) {
        std::mt19937 rng(std::random_device{}());
        std::uniform_int_distribution<int> idxDist(0, static_cast<int>(allPlaces.size()) - 1);

        const double syntheticWeight = 1.0;

        for (int place : allPlaces) {
            if (outCount[place] == 0) {
                int dest = place;
                int tries = 0;
                while (dest == place && tries < 8) {
                    dest = allPlaces[idxDist(rng)];
                    ++tries;
                }
                if (dest != place) {
                    pairWeights[{place, dest}] += syntheticWeight;
                    outCount[place]++;
                    inCount[dest]++;
                }
            }

            if (inCount[place] == 0) {
                int orig = place;
                int tries = 0;
                while (orig == place && tries < 8) {
                    orig = allPlaces[idxDist(rng)];
                    ++tries;
                }
                if (orig != place) {
                    pairWeights[{orig, place}] += syntheticWeight;
                    outCount[orig]++;
                    inCount[place]++;
                }
            }
        }
    }

    for (const auto& kv : pairWeights) {
        ODEntry e;
        e.originWorldId = kv.first.first;
        e.destWorldId = kv.first.second;
        e.weight = kv.second;

        if (e.weight <= 0.0) {
            continue;
        }

        odTotalWeight_ += e.weight;
        odMatrix_.push_back(e);
    }

    if (odMatrix_.size() < 5 || odTotalWeight_ <= 0.0) {
        odMatrix_.clear();
        odTotalWeight_ = 0.0;
    }
}

// загружаем светофоры из бд 
void SimulationBackend::loadTrafficLights()
{
    QSqlQuery query(db_);
    if (!query.exec("SELECT id, x, y FROM traffic_lights")) {
        return;
    }

    while (query.next()) {
        int dbId = query.value(0).toInt();
        double x = query.value(1).toDouble();
        double y = query.value(2).toDouble();

        Vec2 pos;
        pos.x = static_cast<float>(x);
        pos.y = static_cast<float>(y);

        world_.addTrafficLight(dbId, pos);
    }
}

// регистрация концфигураций разных машин для спавна
void SimulationBackend::registerVehicleConfigs()
{
    VehicleConfig passenger{};
    passenger.name = "Легковая";
    passenger.kind = VehicleKind::Passenger;
    passenger.modelId = "passenger";
    passenger.baseSpeedMps = 30.0f;
    passenger.length = 4.0f;
    passenger.width = 1.8f;
    cfgIds_.passenger = world_.registerVehicleConfig(passenger);

    VehicleConfig taxi{};
    taxi.name = "Такси";
    taxi.kind = VehicleKind::Taxi;
    taxi.modelId = "taxi";
    taxi.baseSpeedMps = 32.0f;
    taxi.length = 4.2f;
    taxi.width = 1.8f;
    cfgIds_.taxi = world_.registerVehicleConfig(taxi);

    VehicleConfig bus{};
    bus.name = "Автобус";
    bus.kind = VehicleKind::Bus;
    bus.modelId = "bus";
    bus.baseSpeedMps = 17.0f;
    bus.length = 12.0f;
    bus.width = 2.5f;
    cfgIds_.bus = world_.registerVehicleConfig(bus);

    VehicleConfig truck{};
    truck.name = "Грузовик";
    truck.kind = VehicleKind::TruckHeavy;
    truck.modelId = "truck";
    truck.baseSpeedMps = 20.0f;
    truck.length = 8.0f;
    truck.width = 2.5f;
    cfgIds_.truck = world_.registerVehicleConfig(truck);
}

// создание начальных машин в мире
void SimulationBackend::spawnInitialTraffic(int count)
{
    if (placeIdMap_.empty()) {
        return;
    }

    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> carDist(1, 4);

    if (!odMatrix_.empty()) {
        const auto& t = world_.conditions().time;

        for (int i = 0; i < count; ++i) {
            int origin = 0;
            int dest = 0;
            if (!sampleOdPair(origin, dest, t)) {
                break;
            }

            int carType = carDist(rng);
            int cfgId = (carType == 1)   ? cfgIds_.passenger
                        : (carType == 2) ? cfgIds_.taxi
                        : (carType == 3) ? cfgIds_.bus
                                         : cfgIds_.truck;

            world_.spawnVehicle(origin, dest, cfgId);
        }

        return;
    }

    std::vector<int> dbPlaceIds;
    dbPlaceIds.reserve(placeIdMap_.size());
    for (const auto& kv : placeIdMap_) {
        dbPlaceIds.push_back(kv.first);
    }

    std::uniform_int_distribution<int> placeDist(0, static_cast<int>(dbPlaceIds.size()) - 1);

    for (int i = 0; i < count; ++i) {
        int dbOrigin = dbPlaceIds[placeDist(rng)];
        int dbDest = dbPlaceIds[placeDist(rng)];
        if (dbDest == dbOrigin) {
            dbDest = dbPlaceIds[placeDist(rng)];
        }

        int origin = placeIdMap_.at(dbOrigin);
        int dest = placeIdMap_.at(dbDest);

        int carType = carDist(rng);
        int cfgId = (carType == 1)   ? cfgIds_.passenger
                    : (carType == 2) ? cfgIds_.taxi
                    : (carType == 3) ? cfgIds_.bus
                                     : cfgIds_.truck;

        world_.spawnVehicle(origin, dest, cfgId);
    }
}

// динамическое удаление и добавление машин
void SimulationBackend::spawnDynamicTraffic(float dtSimSeconds)
{
    if (placeIdMap_.empty()) {
        return;
    }

    const auto& cond = world_.conditions();
    const auto& t = cond.time;

    int target = targetVehicleCount(t);

    int active = 0;
    for (const auto& vPtr : world_.vehicles()) {
        if (vPtr && !vPtr->finished()) {
            ++active;
        }
    }

    float diff = target - static_cast<float>(active);

    if (manualVehicleControl_ && diff < 0.0f) {
        int toRemoveTotal = static_cast<int>(-diff);

        const int maxRemovePerTick = 3;
        int toRemove = std::min(toRemoveTotal, maxRemovePerTick);

        if (toRemove > 0) {
            world_.removeRandomVehicles(toRemove);
        }
        return;
    }

    if (diff <= 0.0f) {
        return;
    }

    const float tauMinutes = 10.0f;
    float dtMinutes = dtSimSeconds / 60.0f;

    float spawnAmount = (diff / tauMinutes) * dtMinutes;
    if (spawnAmount <= 0.0f) {
        return;
    }

    spawnAccumulator_ += spawnAmount;

    int toSpawn = static_cast<int>(spawnAccumulator_);
    if (toSpawn <= 0) {
        return;
    }

    if (toSpawn > 15) {
        toSpawn = 15;
    }

    spawnAccumulator_ -= static_cast<float>(toSpawn);

    static thread_local std::mt19937 rng{ std::random_device{}() };
    std::uniform_int_distribution<int> carDist(1, 4);

    if (!odMatrix_.empty()) {
        const auto& t2 = world_.conditions().time;

        for (int i = 0; i < toSpawn; ++i) {
            int origin = 0;
            int dest = 0;
            if (!sampleOdPair(origin, dest, t2)) {
                break;
            }

            int carType = carDist(rng);
            int cfgId = (carType == 1)   ? cfgIds_.passenger
                        : (carType == 2) ? cfgIds_.taxi
                        : (carType == 3) ? cfgIds_.bus
                                         : cfgIds_.truck;

            world_.spawnVehicle(origin, dest, cfgId);
        }
    } else {
        std::vector<int> dbPlaceIds;
        dbPlaceIds.reserve(placeIdMap_.size());
        for (const auto& kv : placeIdMap_) {
            dbPlaceIds.push_back(kv.first);
        }

        if (!dbPlaceIds.empty()) {
            std::uniform_int_distribution<int> placeDist(0, static_cast<int>(dbPlaceIds.size()) - 1);

            for (int i = 0; i < toSpawn; ++i) {
                int dbOrigin = dbPlaceIds[placeDist(rng)];
                int dbDest = dbPlaceIds[placeDist(rng)];
                if (dbDest == dbOrigin) {
                    dbDest = dbPlaceIds[placeDist(rng)];
                }

                int origin = placeIdMap_.at(dbOrigin);
                int dest = placeIdMap_.at(dbDest);

                int carType = carDist(rng);
                int cfgId = (carType == 1)   ? cfgIds_.passenger
                            : (carType == 2) ? cfgIds_.taxi
                            : (carType == 3) ? cfgIds_.bus
                                             : cfgIds_.truck;

                world_.spawnVehicle(origin, dest, cfgId);
            }
        }
    }
}

namespace {

static const double kHourlyWeekday[24] = {
    0.10, 0.05, 0.04, 0.04, 0.06, 0.15, 0.35, 0.70, 1.00, 0.90, 0.60, 0.60,
    0.65, 0.70, 0.75, 0.80, 0.90, 1.00, 0.90, 0.70, 0.50, 0.35, 0.25, 0.15
};

static const double kHourlyWeekend[24] = {
    0.05, 0.04, 0.03, 0.03, 0.04, 0.06, 0.10, 0.20, 0.30, 0.45, 0.60, 0.75,
    0.80, 0.80, 0.75, 0.70, 0.65, 0.60, 0.55, 0.45, 0.35, 0.25, 0.15, 0.08
};

static const double kMonthlyFactorTable[13] = {
    0.0, 0.90, 0.95, 1.00, 1.00, 1.05, 1.05, 0.95, 1.05, 1.10, 1.05, 1.00, 1.00
};

static const double kWeekdayFactorTable[8] = { 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.8, 0.8 };

}

// возвращает множитель интенсивности трафика в соответствии с часом
double SimulationBackend::hourlyFactor(const traffic::SimulationTime& t) const
{
    int hour = std::clamp(t.hour, 0, 23);
    bool weekend = (t.weekday == Weekday::Saturday || t.weekday == Weekday::Sunday);
    return weekend ? kHourlyWeekend[hour] : kHourlyWeekday[hour];
}

// возвращает множитель интенсивности трафика в соответствии с месяцем
double SimulationBackend::monthlyFactor(const traffic::SimulationTime& t) const
{
    int m = std::clamp(static_cast<int>(t.month), 1, 12);
    return kMonthlyFactorTable[m];
}

// возвращает множитель интенсивности трафика в соответствии с днем недели
double SimulationBackend::weekdayFactor(const traffic::SimulationTime& t) const
{
    int d = std::clamp(static_cast<int>(t.weekday), 1, 7);
    return kWeekdayFactorTable[d];
}

// считает верхнюю планку по числу машин для текущего времени
int SimulationBackend::targetVehicleCount(const traffic::SimulationTime& t) const
{
    if (manualVehicleControl_) {
        return std::max(0, std::min(manualVehicleTarget_, 3000));
    }

    double base = 100.0;
    double f = monthlyFactor(t) * weekdayFactor(t) * hourlyFactor(t);
    f = std::clamp(f, 0.1, 1.3);

    double raw = base * f;

    int minVehicles = 50;
    int maxVehicles = 120;

    traffic::TimeOfDay tod = t.timeOfDay();

    switch (tod) {
    case traffic::TimeOfDay::Morning:
    case traffic::TimeOfDay::Evening:
        minVehicles = 70;
        maxVehicles = 150;
        break;
    case traffic::TimeOfDay::Day:
        minVehicles = 50;
        maxVehicles = 100;
        break;
    case traffic::TimeOfDay::Night:
    default:
        minVehicles = 20;
        maxVehicles = 50;
        break;
    }

    int target = static_cast<int>(std::round(raw));
    target = std::clamp(target, minVehicles, maxVehicles);

    return target;
}

// шаг симуляции
void SimulationBackend::onTick()
{
    if (paused_) {
        return;
    }

    constexpr float dtRealSeconds = 0.05f;
    float dtSimSeconds = dtRealSeconds * world_.timeScale();

    world_.step(dtRealSeconds);
    spawnDynamicTraffic(dtSimSeconds);

    if (++frameSkipCounter_ >= 2) {
        frameSkipCounter_ = 0;
        emit worldUpdated();
    }
}

// устанавливает время симуляции и пересоздает трафик
void SimulationBackend::setSimulationDateTime(int year, int month, int day, int hour, int minute)
{
    const traffic::SimulationTime oldTime = world_.conditions().time;

    year = std::max(1, year);
    month = std::clamp(month, 1, 12);
    hour = std::clamp(hour, 0, 23);
    minute = std::clamp(minute, 0, 59);
    day = std::max(1, day);

    traffic::SimulationTime t = oldTime;
    t.year = year;
    t.month = static_cast<traffic::Month>(month);
    t.day = day;
    t.hour = hour;
    t.minute = minute;

    if (t.year == oldTime.year && t.month == oldTime.month && t.day == oldTime.day && t.hour == oldTime.hour &&
        t.minute == oldTime.minute) {
        return;
    }

    traffic::TrafficConditions cond = world_.conditions();
    cond.time = t;

    cond.accident = false;
    cond.roadWork = false;
    cond.ice = false;

    world_.setConditions(cond);

    const int target = targetVehicleCount(cond.time);

    const int totalVehicles = static_cast<int>(world_.vehicles().size());
    if (totalVehicles > 0) {
        world_.removeRandomVehicles(totalVehicles);
    }

    spawnAccumulator_ = 0.0f;
    spawnInitialTraffic(target);
}

// включает/выключает аварии и расставляет по путям
void SimulationBackend::setAccidentEnabled(bool enabled)
{
    manualAccidentEnabled_ = enabled;

    world_.clearAllAccidents();

    if (!enabled) {
        return;
    }

    const auto& paths = world_.paths();
    if (paths.empty()) {
        return;
    }

    std::vector<int> pathIds;
    pathIds.reserve(paths.size());
    for (const auto& kv : paths) {
        pathIds.push_back(kv.first);
    }

    static thread_local std::mt19937 rng{ std::random_device{}() };
    std::shuffle(pathIds.begin(), pathIds.end(), rng);

    int maxIncidents = std::min<int>(5, pathIds.size());
    for (int i = 0; i < maxIncidents; ++i) {
        world_.setPathAccident(pathIds[i], true);
    }
}

// включает/выключает дорожные работы и расставляет по путям
void SimulationBackend::setRoadWorkEnabled(bool enabled)
{
    manualRoadWorkEnabled_ = enabled;

    world_.clearAllRoadWorks();

    if (!enabled) {
        return;
    }

    const auto& paths = world_.paths();
    if (paths.empty()) {
        return;
    }

    std::vector<int> pathIds;
    pathIds.reserve(paths.size());
    for (const auto& kv : paths) {
        pathIds.push_back(kv.first);
    }

    static thread_local std::mt19937 rng{ std::random_device{}() };
    std::shuffle(pathIds.begin(), pathIds.end(), rng);

    int maxIncidents = std::min<int>(5, pathIds.size());
    for (int i = 0; i < maxIncidents; ++i) {
        world_.setPathRoadWork(pathIds[i], true);
    }
}

// включает/выключает гололед и расставляет по путям
void SimulationBackend::setIceEnabled(bool enabled)
{
    manualIceEnabled_ = enabled;

    world_.clearAllIce();

    if (!enabled) {
        return;
    }

    const auto& t = world_.conditions().time;
    int month = static_cast<int>(t.month);
    if (month < 10 && month > 3) {
        return;
    }

    const auto& paths = world_.paths();
    if (paths.empty()) {
        return;
    }

    std::vector<int> pathIds;
    pathIds.reserve(paths.size());
    for (const auto& kv : paths) {
        pathIds.push_back(kv.first);
    }

    static thread_local std::mt19937 rng{ std::random_device{}() };
    std::shuffle(pathIds.begin(), pathIds.end(), rng);

    int maxIncidents = std::min<int>(8, pathIds.size());
    for (int i = 0; i < maxIncidents; ++i) {
        world_.setPathIce(pathIds[i], true);
    }
}

// включает/выключает работу светофоров
void SimulationBackend::setLightsEnabled(bool enabled)
{
    world_.setLightsEnabled(enabled);
}

// ручное управление числом машин
void SimulationBackend::setManualVehicleTarget(int count)
{
    count = std::max(0, std::min(count, 3000));
    manualVehicleTarget_ = count;
    manualVehicleControl_ = (count > 0);
}

// выбирает пару origin/destinnation из OD-матрицы с учетом времени
bool SimulationBackend::sampleOdPair(int& originWorldId, int& destWorldId, const traffic::SimulationTime& t) const
{
    if (odMatrix_.empty()) {
        return false;
    }

    double totalDynamic = 0.0;

    struct Tmp {
        const ODEntry* e;
        double dynWeight;
    };

    std::vector<Tmp> tmp;
    tmp.reserve(odMatrix_.size());

    for (const auto& e : odMatrix_) {
        const auto* from = world_.getPlace(e.originWorldId);
        const auto* to = world_.getPlace(e.destWorldId);
        if (!from || !to) {
            continue;
        }

        double base = e.weight;
        if (base <= 0.0) {
            continue;
        }

        double tf = odTimeFactor(*from, *to, t);
        double dyn = base * tf;
        if (dyn <= 0.0) {
            continue;
        }

        totalDynamic += dyn;
        tmp.push_back(Tmp{ &e, dyn });
    }

    if (tmp.empty() || totalDynamic <= 0.0) {
        return false;
    }

    static thread_local std::mt19937 rng{ std::random_device{}() };
    std::uniform_real_distribution<double> dist(0.0, totalDynamic);
    double r = dist(rng);

    for (const auto& item : tmp) {
        if (r < item.dynWeight) {
            originWorldId = item.e->originWorldId;
            destWorldId = item.e->destWorldId;
            return true;
        }
        r -= item.dynWeight;
    }

    const auto* e = tmp.back().e;
    originWorldId = e->originWorldId;
    destWorldId = e->destWorldId;
    return true;
}

// ставит симуляцию на паузу или снимает с паузы
void SimulationBackend::setPaused(bool paused)
{
    paused_ = paused;
}

}
