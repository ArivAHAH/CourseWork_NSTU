#include "traffic_core.h"

namespace traffic {

// возвращает коэффициент замедления по плотности на пути
float TrafficWorld::pathCongestionFactor(int pathId) const
{
    auto it = pathVehicleDensity_.find(pathId);
    if (it == pathVehicleDensity_.end()) {
        return 1.0f;
    }

    float rho = it->second;

    const float rhoFree = 0.05f;
    const float rhoJam  = 0.40f;

    if (rho <= rhoFree) {
        return 1.0f;
    }
    if (rho >= rhoJam) {
        return 0.25f;
    }

    float t = (rho - rhoFree) / (rhoJam - rhoFree);
    return 1.0f - 0.75f * t;
}


// синхронизирует итоговые флаги условий из авто/ручных источников
void TrafficWorld::updateConditionsFlags()
{
    conditions_.accident = autoAccident_ || manualAccident_;
    conditions_.roadWork = autoRoadWork_ || manualRoadWork_;
    conditions_.ice      = autoIce_      || manualIce_;
}

// применяет новые условия и сбрасывает связанные таймеры событий
void TrafficWorld::setConditions(const TrafficConditions& conditions)
{
    conditions_ = conditions;

    secondsWithinDay_ =
        conditions_.time.hour * 3600.0f +
        conditions_.time.minute * 60.0f;

    accidentSecondsLeft_ = 0.0f;
    roadWorkSecondsLeft_ = 0.0f;
    iceSecondsLeft_      = 0.0f;

    autoAccident_ = conditions_.accident;
    autoRoadWork_ = conditions_.roadWork;
    autoIce_      = conditions_.ice;

    manualAccident_ = false;
    manualRoadWork_ = false;
    manualIce_      = false;

    updateConditionsFlags();
}

// определяет часть суток по часу
TimeOfDay SimulationTime::timeOfDay() const
{
    if (hour < 6)  return TimeOfDay::Night;
    if (hour < 11) return TimeOfDay::Morning;
    if (hour < 18) return TimeOfDay::Day;
    return TimeOfDay::Evening;
}

// проверяет, является ли день выходным
bool SimulationTime::isWeekend() const
{
    return weekday == Weekday::Saturday || weekday == Weekday::Sunday;
}

// определяет сезон по месяцу
Season SimulationTime::season() const
{
    switch (month) {
        case Month::December:
        case Month::January:
        case Month::February:
            return Season::Winter;
        case Month::March:
        case Month::April:
        case Month::May:
            return Season::Spring;
        case Month::June:
        case Month::July:
        case Month::August:
            return Season::Summer;
        default:
            return Season::Autumn;
    }
}

// возвращает общий коэффициент скорости из календаря и времени суток
float TrafficConditions::speedFactor() const
{
    float f = 1.0f;

    switch (time.timeOfDay()) {
        case TimeOfDay::Morning:
        case TimeOfDay::Evening:
            f *= 0.7f;
            break;
        case TimeOfDay::Day:
            f *= 0.9f;
            break;
        case TimeOfDay::Night:
            f *= 1.1f;
            break;
    }

    if (time.isWeekend()) {
        f *= 1.1f;
    }

    switch (time.season()) {
        case Season::Winter:
            f *= 0.8f;
            break;
        case Season::Summer:
            f *= 1.0f;
            break;
        case Season::Spring:
        case Season::Autumn:
            f *= 0.95f;
            break;
    }

    f = std::clamp(f, 0.1f, 1.5f);
    return f;
}

// считает евклидово расстояние между двумя точками
static float distance(const Vec2& a, const Vec2& b)
{
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

// пересчитывает длину пути по набору точек
void Path::recomputeLength()
{
    length = 0.0f;
    if (points.size() < 2) {
        return;
    }

    for (std::size_t i = 1; i < points.size(); ++i) {
        length += distance(points[i - 1], points[i]);
    }
}

// возвращает позицию на ломаной по пройденной дистанции
Vec2 Path::positionAt(float d) const
{
    if (points.empty()) return {};
    if (points.size() == 1) return points.front();

    if (d <= 0.0f) return points.front();
    if (d >= length) return points.back();

    float accumulated = 0.0f;
    for (std::size_t i = 1; i < points.size(); ++i) {
        Vec2 p0 = points[i - 1];
        Vec2 p1 = points[i];

        float segLen = distance(p0, p1);
        if (accumulated + segLen >= d) {
            float t = (d - accumulated) / segLen;
            return Vec2{
                p0.x + t * (p1.x - p0.x),
                p0.y + t * (p1.y - p0.y)
            };
        }
        accumulated += segLen;
    }

    return points.back();
}

// возвращает текущий сегмент маршрута, либо nullptr если маршрут завершён.
const Path* Vehicle::currentPath() const
{
    if (finished_) return nullptr;
    if (route_.empty()) return nullptr;

    if (currentSegmentIndex_ < 0 ||
        currentSegmentIndex_ >= static_cast<int>(route_.size())) {
        return nullptr;
    }

    return route_[currentSegmentIndex_];
}

// возвращает следующий сегмент маршрута, либо nullptr
const Path* Vehicle::nextPathInRoute() const
{
    if (finished_) return nullptr;
    if (route_.empty()) return nullptr;

    int idx = currentSegmentIndex_ + 1;
    if (idx < 0 || idx >= static_cast<int>(route_.size())) {
        return nullptr;
    }

    return route_[idx];
}

// оценивает желаемую скорость автомобиля с учётом условий и ограничений на пути
float Vehicle::effectiveSpeed(const TrafficConditions& conditions) const
{
    constexpr float kVisualSpeedBoost = 2.0f;

    float v = config_.baseSpeedMps * conditions.speedFactor();

    const Path* p = currentPath();
    if (p) {
        v *= p->localSpeedFactor();

        float limit = p->speedLimitMps * kVisualSpeedBoost;
        if (std::isfinite(limit) && limit > 0.0f && v > limit) {
            v = limit;
        }
    }

    v *= kVisualSpeedBoost;
    return std::max(v, 0.1f);
}

// делает один шаг движения автомобиля с учётом светофоров и ограничения maxDistance
void Vehicle::step(
    float dtSeconds,
    const TrafficConditions& conditions,
    const std::vector<TrafficLight>& lights,
    const std::vector<LightColor>& colors,
    float maxDistance
) {
    if (finished_) return;
    if (dtSeconds <= 0.0f) return;

    const Path* p = currentPath();
    if (!p) {
        finished_ = true;
        return;
    }

    const float vFree = effectiveSpeed(conditions);
    const float halfLen = config_.length * 0.5f;
    const float myFront = distance_ + halfLen;

    float stopDist = distance_ + vFree * dtSeconds;

    if (!lights.empty() && lights.size() == colors.size()) {
        int bestIdx = -1;
        float bestDelta = std::numeric_limits<float>::infinity();

        for (std::size_t i = 0; i < lights.size(); ++i) {
            const TrafficLight& tl = lights[i];
            if (tl.pathId != p->id) continue;

            float delta = tl.distanceOnPath - myFront;
            if (delta < -2.0f) continue;
            if (delta > 80.0f) continue;

            if (delta < bestDelta) {
                bestDelta = delta;
                bestIdx = static_cast<int>(i);
            }
        }

        if (bestIdx >= 0) {
            const LightColor c = colors[static_cast<std::size_t>(bestIdx)];
            const float lightPos = lights[static_cast<std::size_t>(bestIdx)].distanceOnPath;
            const float stopMargin = 3.0f;
            float stopAt = lightPos - halfLen - stopMargin;
            if (stopAt < 0.0f) stopAt = 0.0f;

            bool pastStopLine = (distance_ > stopAt);

            if (!pastStopLine) {
                if (c == LightColor::Red) {
                    stopDist = std::min(stopDist, stopAt);
                } else if (c == LightColor::Yellow) {
                    float distToStop = stopAt - distance_;
                    float brakingDist = (speed_ * speed_) / (2.0f * maxDecel_);
                    if (distToStop > brakingDist + 1.0f) {
                        stopDist = std::min(stopDist, stopAt);
                    }
                }
            }
        }
    }

    if (maxDistance < stopDist) stopDist = maxDistance;
    if (stopDist < distance_) stopDist = distance_;

    float vAllowed = (stopDist - distance_) / dtSeconds;
    float vTarget  = std::min(vFree, vAllowed);
    if (vTarget < 0.0f) vTarget = 0.0f;

    float dv = vTarget - speed_;
    const float maxUp   = maxAccel_ * dtSeconds;
    const float maxDown = maxDecel_ * dtSeconds;

    if (dv > 0.0f) dv = std::min(dv,  maxUp);
    else           dv = std::max(dv, -maxDown);

    float newSpeed = std::max(speed_ + dv, 0.0f);
    float avgSpeed = 0.5f * (speed_ + newSpeed);
    float newDist  = distance_ + avgSpeed * dtSeconds;

    if (newDist > stopDist) {
        newDist  = stopDist;
        newSpeed = 0.0f;
    }

    if (newDist > p->length) {
        newDist  = p->length;
        newSpeed = 0.0f;
    }

    distance_ = newDist;
    speed_    = newSpeed;

    const float eps = 1e-3f;

    if (distance_ >= p->length - eps) {
        if (currentSegmentIndex_ + 1 >= static_cast<int>(route_.size())) {
            finished_ = true;
            distance_ = p->length;
            speed_    = 0.0f;
            return;
        }

        float over = distance_ - p->length;
        currentSegmentIndex_++;

        const Path* nextP = currentPath();
        if (!nextP) {
            finished_ = true;
            speed_    = 0.0f;
            return;
        }

        distance_ = std::min(over, nextP->length);
    }
}

// возвращает оставшуюся дистанцию по маршруту от текущего положения
float Vehicle::remainingDistance() const
{
    if (route_.empty()) {
        return 0.0f;
    }

    float total = 0.0f;

    for (int i = currentSegmentIndex_; i < static_cast<int>(route_.size()); ++i) {
        const Path* p = route_[i];
        if (!p) continue;

        if (i == currentSegmentIndex_) {
            float rem = p->length - distance_;
            total += std::max(rem, 0.0f);
        } else {
            total += p->length;
        }
    }

    return std::max(total, 0.0f);
}

// возвращает текущую позицию автомобиля в мировых координатах
Vec2 Vehicle::position() const
{
    const Path* p = currentPath();
    if (!p) return {};
    return p->positionAt(distance_);
}

// оценивает оставшееся время движения до финиша в секундах
float Vehicle::remainingTimeSeconds(const TrafficConditions& conditions) const
{
    float dist = remainingDistance();
    float v    = effectiveSpeed(conditions);
    if (v <= 0.01f) {
        return std::numeric_limits<float>::infinity();
    }
    return dist / v;
}

// возвращает число дней в месяце
int TrafficWorld::daysInMonth(Month m) const
{
    switch (m) {
        case Month::January:   return 31;
        case Month::February:  return 28;
        case Month::March:     return 31;
        case Month::April:     return 30;
        case Month::May:       return 31;
        case Month::June:      return 30;
        case Month::July:      return 31;
        case Month::August:    return 31;
        case Month::September: return 30;
        case Month::October:   return 31;
        case Month::November:  return 30;
        case Month::December:  return 31;
    }
    return 30;
}

// возвращает следующий месяц по кругу
Month TrafficWorld::nextMonth(Month m) const
{
    switch (m) {
        case Month::January:   return Month::February;
        case Month::February:  return Month::March;
        case Month::March:     return Month::April;
        case Month::April:     return Month::May;
        case Month::May:       return Month::June;
        case Month::June:      return Month::July;
        case Month::July:      return Month::August;
        case Month::August:    return Month::September;
        case Month::September: return Month::October;
        case Month::October:   return Month::November;
        case Month::November:  return Month::December;
        case Month::December:  return Month::January;
    }
    return Month::January;
}

// возвращает следующий день недели по кругу
Weekday TrafficWorld::nextWeekday(Weekday w) const
{
    switch (w) {
        case Weekday::Monday:    return Weekday::Tuesday;
        case Weekday::Tuesday:   return Weekday::Wednesday;
        case Weekday::Wednesday: return Weekday::Thursday;
        case Weekday::Thursday:  return Weekday::Friday;
        case Weekday::Friday:    return Weekday::Saturday;
        case Weekday::Saturday:  return Weekday::Sunday;
        case Weekday::Sunday:    return Weekday::Monday;
    }
    return Weekday::Monday;
}

// продвигает календарное время и генерирует события 
void TrafficWorld::advanceTimeAndEvents(float dtSimSeconds)
{
    if (dtSimSeconds <= 0.0f) {
        return;
    }

    secondsWithinDay_ += dtSimSeconds;

    while (secondsWithinDay_ >= 86400.0f) {
        secondsWithinDay_ -= 86400.0f;
        ++daysSinceStart_;

        auto& t = conditions_.time;

        t.weekday = nextWeekday(t.weekday);

        ++t.day;
        int dim = daysInMonth(t.month);
        if (t.day > dim) {
            t.day = 1;

            if (t.month == Month::December) {
                t.month = Month::January;
                ++t.year;
            } else {
                t.month = nextMonth(t.month);
            }
        }

        if (t.month == Month::November || t.month == Month::February) {
            if (iceSecondsLeft_ <= 0.0f) {
                std::uniform_real_distribution<float> u(0.0f, 1.0f);
                if (u(rng_) < (2.0f / 7.0f)) {
                    iceSecondsLeft_ = 86400.0f;
                }
            }
        }

        bool isSummerMonth =
            (t.month == Month::June || t.month == Month::July || t.month == Month::August);

        if (isSummerMonth && roadWorkSecondsLeft_ <= 0.0f) {
            std::uniform_real_distribution<float> u(0.0f, 1.0f);
            if (u(rng_) < 0.08f) {
                std::uniform_int_distribution<int> durDays(3, 4);
                int d = durDays(rng_);
                roadWorkSecondsLeft_ = d * 86400.0f;
            }
        }
    }

    auto& t = conditions_.time;
    int totalMinutes = static_cast<int>(secondsWithinDay_ / 60.0f);
    t.hour   = totalMinutes / 60;
    t.minute = totalMinutes % 60;

    auto dec = [dtSimSeconds](float& x) {
        if (x > 0.0f) {
            x -= dtSimSeconds;
            if (x < 0.0f) x = 0.0f;
        }
    };

    dec(accidentSecondsLeft_);
    dec(roadWorkSecondsLeft_);
    dec(iceSecondsLeft_);

    if (accidentSecondsLeft_ <= 0.0f) {
        bool iceActive = (iceSecondsLeft_ > 0.0f);
        float lambdaPerHour = iceActive ? 0.18f : 0.06f;

        float dtHours = dtSimSeconds / 3600.0f;
        float p = lambdaPerHour * dtHours;
        p = std::min(p, 0.9f);

        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        if (u(rng_) < p) {
            std::uniform_real_distribution<float> durMin(20.0f, 60.0f);
            accidentSecondsLeft_ = durMin(rng_) * 60.0f;
        }
    }

    autoIce_      = (iceSecondsLeft_ > 0.0f);
    autoRoadWork_ = (roadWorkSecondsLeft_ > 0.0f);
    autoAccident_ = (accidentSecondsLeft_ > 0.0f);

    updateConditionsFlags();
}

// регистрирует конфигурацию автомобиля и возвращает её id
int TrafficWorld::registerVehicleConfig(const VehicleConfig& cfg)
{
    int id = nextVehicleConfigId_++;
    VehicleConfig copy = cfg;
    copy.id = id;
    vehicleConfigs_[id] = copy;
    return id;
}

// возвращает конфиг автомобиля по id, либо nullptr
const VehicleConfig* TrafficWorld::getVehicleConfig(int configId) const
{
    auto it = vehicleConfigs_.find(configId);
    if (it == vehicleConfigs_.end()) return nullptr;
    return &it->second;
}

// добавляет место в мир и возвращает его id
int TrafficWorld::addPlace(const std::string& name, PlaceCategory category, const Vec2& pos)
{
    int id = nextPlaceId_++;

    Place place;
    place.id = id;
    place.name = name;
    place.category = category;
    place.position = pos;

    places_[id] = place;
    return id;
}

// возвращает место по id, либо nullptr
const Place* TrafficWorld::getPlace(int id) const
{
    auto it = places_.find(id);
    if (it == places_.end()) return nullptr;
    return &it->second;
}

// добавляет путь между местами и возвращает его id
int TrafficWorld::addPath(int originPlaceId, int destinationPlaceId, const std::vector<Vec2>& points)
{
    int id = nextPathId_++;

    Path path;
    path.id = id;
    path.originPlaceId = originPlaceId;
    path.destinationPlaceId = destinationPlaceId;
    path.points = points;
    path.recomputeLength();

    paths_[id] = path;
    return id;
}

// устанавливает ограничение скорости для пути в км/ч
void TrafficWorld::setPathSpeedLimit(int pathId, float speedLimitKph)
{
    auto it = paths_.find(pathId);
    if (it == paths_.end()) {
        return;
    }

    if (speedLimitKph <= 0.0f) {
        it->second.speedLimitMps = std::numeric_limits<float>::infinity();
    } else {
        it->second.speedLimitMps = speedLimitKph / 3.6f;
    }
}

// возвращает путь по id, либо nullptr
const Path* TrafficWorld::getPath(int id) const
{
    auto it = paths_.find(id);
    if (it == paths_.end()) return nullptr;
    return &it->second;
}

// находит прямой путь между origin и destination, выбирая случайный из кандидатов
const Path* TrafficWorld::findPath(int originPlaceId, int destinationPlaceId) const
{
    std::vector<const Path*> candidates;
    candidates.reserve(4);

    for (const auto& kv : paths_) {
        const Path& p = kv.second;
        if (p.originPlaceId == originPlaceId &&
            p.destinationPlaceId == destinationPlaceId) {
            candidates.push_back(&p);
        }
    }

    if (candidates.empty()) return nullptr;
    if (candidates.size() == 1) return candidates.front();

    static thread_local std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<std::size_t> dist(0, candidates.size() - 1);
    return candidates[dist(rng)];
}

// прикрепляет светофор к ближайшим подходящим дорогам и добавляет его в мир
int TrafficWorld::addTrafficLight(int dbId, const Vec2& pos)
{
    struct Candidate {
        int   pathId;
        float distanceAlong;
        float projX;
        float projY;
        float gapSq;
    };

    const float maxAttachDist   = 20.0f;
    const float maxAttachDistSq = maxAttachDist * maxAttachDist;

    std::unordered_map<int, Candidate> bestByPath;

    for (const auto& kv : paths_) {
        const Path& p = kv.second;
        if (p.points.size() < 2 || p.length <= 1e-3f) {
            continue;
        }

        float accumulated = 0.0f;

        for (std::size_t i = 1; i < p.points.size(); ++i) {
            Vec2 a = p.points[i - 1];
            Vec2 b = p.points[i];

            float vx = b.x - a.x;
            float vy = b.y - a.y;
            float segLenSq = vx * vx + vy * vy;
            if (segLenSq < 1e-6f) {
                continue;
            }

            float t = ((pos.x - a.x) * vx + (pos.y - a.y) * vy) / segLenSq;
            t = std::clamp(t, 0.0f, 1.0f);

            float projX = a.x + t * vx;
            float projY = a.y + t * vy;

            float dx = projX - pos.x;
            float dy = projY - pos.y;
            float gapSq = dx * dx + dy * dy;

            if (gapSq > maxAttachDistSq) {
                accumulated += std::sqrt(segLenSq);
                continue;
            }

            float segLen    = std::sqrt(segLenSq);
            float distAlong = accumulated + segLen * t;

            auto it = bestByPath.find(p.id);
            if (it == bestByPath.end() || gapSq < it->second.gapSq) {
                bestByPath[p.id] = Candidate{p.id, distAlong, projX, projY, gapSq};
            }

            accumulated += segLen;
        }
    }

    if (bestByPath.empty()) {
        return -1;
    }

    int firstId = -1;

    for (const auto& kv : bestByPath) {
        const Candidate& c = kv.second;

        TrafficLight tl;
        tl.id             = dbId;
        tl.position.x     = c.projX;
        tl.position.y     = c.projY;
        tl.pathId         = c.pathId;
        tl.distanceOnPath = c.distanceAlong;

        tl.groupId = dbId % 2;

        trafficLights_.push_back(tl);

        if (firstId < 0) {
            firstId = tl.id;
        }
    }

    return firstId;
}

// строит список перекрёстков и допустимых движений по входящим/выходящим путям
void TrafficWorld::buildIntersections()
{
    intersections_.clear();

    struct Adj {
        std::vector<int> incoming;
        std::vector<int> outgoing;
    };

    std::unordered_map<int, Adj> adj;

    for (const auto& kv : paths_) {
        const Path& p = kv.second;
        adj[p.originPlaceId].outgoing.push_back(p.id);
        adj[p.destinationPlaceId].incoming.push_back(p.id);
    }

    for (const auto& pair : adj) {
        int placeId = pair.first;
        const Adj& a = pair.second;

        const Place* place = getPlace(placeId);
        if (!place) {
            continue;
        }

        std::size_t degree = a.incoming.size() + a.outgoing.size();

        bool isIntersection =
            (place->category == PlaceCategory::Intersection) ||
            (degree >= 3);

        if (!isIntersection) {
            continue;
        }

        Intersection inter;
        inter.placeId       = placeId;
        inter.incomingPaths = a.incoming;
        inter.outgoingPaths = a.outgoing;

        for (int pathId : inter.incomingPaths) {
            const Path* p = getPath(pathId);
            if (!p) continue;

            int prio = 1;

            float v = p->speedLimitMps;
            if (!std::isfinite(v) || v <= 0.0f) {
                v = 50.0f / 3.6f;
            }

            if (v >= 70.0f / 3.6f) {
                prio = 3;
            } else if (v >= 60.0f / 3.6f) {
                prio = 2;
            } else {
                prio = 1;
            }

            inter.pathPriority[pathId] = prio;
        }

        for (int inId : inter.incomingPaths) {
            const Path* pin = getPath(inId);
            if (!pin || pin->points.size() < 2) continue;

            Vec2 a1 = pin->points[pin->points.size() - 2];
            Vec2 a2 = pin->points.back();

            float vxIn = a2.x - a1.x;
            float vyIn = a2.y - a1.y;

            float lenIn = std::sqrt(vxIn * vxIn + vyIn * vyIn);
            if (lenIn < 1e-3f) continue;

            vxIn /= lenIn;
            vyIn /= lenIn;

            for (int outId : inter.outgoingPaths) {
                const Path* pout = getPath(outId);
                if (!pout || pout->points.size() < 2) continue;

                Vec2 b1 = pout->points[0];
                Vec2 b2 = pout->points[1];

                float vxOut = b2.x - b1.x;
                float vyOut = b2.y - b1.y;

                float lenOut = std::sqrt(vxOut * vxOut + vyOut * vyOut);
                if (lenOut < 1e-3f) continue;

                vxOut /= lenOut;
                vyOut /= lenOut;

                float dot = vxIn * vxOut + vyIn * vyOut;
                dot = std::clamp(dot, -1.0f, 1.0f);

                float angleRad = std::acos(dot);
                float angleDeg = angleRad * 180.0f / 3.1415926535f;

                float cross = vxIn * vyOut - vyIn * vxOut;

                TurnType turn;
                if (angleDeg > 150.0f) {
                    turn = TurnType::UTurn;
                } else if (angleDeg < 30.0f) {
                    turn = TurnType::Straight;
                } else {
                    turn = (cross > 0.0f) ? TurnType::Left : TurnType::Right;
                }

                Intersection::Movement mv;
                mv.inPathId  = inId;
                mv.outPathId = outId;
                mv.turn      = turn;

                inter.movements.push_back(mv);
            }
        }

        intersections_.push_back(std::move(inter));
    }
}

// строит маршрут между двумя местами по графу дорог
bool TrafficWorld::buildRoute(
    int originPlaceId,
    int destinationPlaceId,
    std::vector<const Path*>& out
) const {
    out.clear();

    if (originPlaceId == destinationPlaceId) return false;
    if (paths_.empty()) return false;

    struct Edge {
        int to;
        const Path* path;
        float cost;
    };

    std::unordered_map<int, std::vector<Edge>> graph;

    for (const auto& kv : paths_) {
        const Path& p = kv.second;

        float v = p.speedLimitMps;
        if (!std::isfinite(v) || v <= 0.0f) {
            v = 50.0f / 3.6f;
        }

        float cong = pathCongestionFactor(p.id);
        cong = std::max(cong, 0.25f);

        float vEff = v * cong;
        float cost = p.length / vEff;

        graph[p.originPlaceId].push_back(Edge{p.destinationPlaceId, &p, cost});
    }

    if (graph.find(originPlaceId) == graph.end()) {
        return false;
    }

    std::unordered_map<int, float> dist;
    std::unordered_map<int, int> prevPlace;
    std::unordered_map<int, const Path*> prevPath;

    const float INF = std::numeric_limits<float>::infinity();
    for (const auto& kv : places_) {
        dist[kv.first] = INF;
    }

    auto cmp = [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
        return a.first > b.first;
    };

    std::priority_queue<
        std::pair<float, int>,
        std::vector<std::pair<float, int>>,
        decltype(cmp)
    > pq(cmp);

    dist[originPlaceId] = 0.0f;
    pq.push({0.0f, originPlaceId});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();

        if (d > dist[u]) continue;
        if (u == destinationPlaceId) break;

        auto it = graph.find(u);
        if (it == graph.end()) continue;

        for (const Edge& e : it->second) {
            float nd = d + e.cost;
            auto itDist = dist.find(e.to);
            if (itDist == dist.end()) continue;

            if (nd < itDist->second) {
                itDist->second = nd;
                prevPlace[e.to] = u;
                prevPath[e.to]  = e.path;
                pq.push({nd, e.to});
            }
        }
    }

    if (dist[destinationPlaceId] == INF) {
        return false;
    }

    std::vector<const Path*> rev;
    int cur = destinationPlaceId;

    while (cur != originPlaceId) {
        auto itP = prevPath.find(cur);
        auto itU = prevPlace.find(cur);
        if (itP == prevPath.end() || itU == prevPlace.end()) {
            break;
        }
        rev.push_back(itP->second);
        cur = itU->second;
    }

    if (rev.empty() || cur != originPlaceId) {
        return false;
    }

    for (auto it = rev.rbegin(); it != rev.rend(); ++it) {
        out.push_back(*it);
    }

    return !out.empty();
}

LightColor TrafficWorld::trafficLightColor(const TrafficLight& light) const
{
    if (!lightsEnabled_) {
        return LightColor::Green;
    }

    const SimulationTime& ti = conditions_.time;
    bool morningRush = (ti.hour >= 8 && ti.hour < 10);

    const float greenDuration  = morningRush ? 35.0f : 25.0f;
    const float yellowDuration = 2.0f;
    const float redDuration = 30.0f;
    const float totalPhase = greenDuration + yellowDuration + redDuration;

    int group = light.groupId % 2;
    float offset = group * (greenDuration + yellowDuration);

    float t = std::fmod(simSeconds_ + offset, totalPhase);

    if (t < greenDuration){
        return LightColor::Green;
    }

    if (t < greenDuration + yellowDuration){
        return LightColor::Yellow;
    }

    return LightColor::Red;
}

// спавнит машину с проверками на возможность маршрута и расстояния до других машин
int TrafficWorld::spawnVehicle(int originPlaceId, int destinationPlaceId, int vehicleConfigId)
{
    const Place* origin = getPlace(originPlaceId);
    const Place* dest   = getPlace(destinationPlaceId);
    const VehicleConfig* cfg = getVehicleConfig(vehicleConfigId);

    if (!origin || !dest || !cfg) {
        return -1;
    }

    std::vector<const Path*> route;
    bool routeOk = buildRoute(originPlaceId, destinationPlaceId, route);

    if (!routeOk) {
        const Path* single = findPath(originPlaceId, destinationPlaceId);
        if (!single) {
            return -1;
        }
        route.push_back(single);
    }

    if (route.empty()) {
        return -1;
    }

    const Path* firstPath = route.front();

    auto dist2 = [](const Vec2& a, const Vec2& b) {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        return dx * dx + dy * dy;
    };

    float spawnOffset = cfg->length * 0.5f;
    Vec2 spawnPos = firstPath->positionAt(spawnOffset);

    const float kSpawnGeomMin  = 8.0f;
    const float kSpawnGeomMin2 = kSpawnGeomMin * kSpawnGeomMin;

    for (const auto& vPtr : vehicles_) {
        if (!vPtr) continue;
        if (vPtr->finished()) continue;

        Vec2 pos = vPtr->position();
        if (dist2(pos, spawnPos) < kSpawnGeomMin2) {
            return -1;
        }
    }

    const float kSpawnGapMeters = 18.0f;

    for (const auto& vPtr : vehicles_) {
        if (!vPtr) continue;
        if (vPtr->finished()) continue;

        const Path* vp = vPtr->path();
        if (!vp || vp->id != firstPath->id) continue;

        float d = vPtr->traveledDistance();

        float minClear = 0.5f * cfg->length +
                         0.5f * vPtr->config().length +
                         kSpawnGapMeters;

        if (d < minClear) {
            return -1;
        }
    }

    int id = nextVehicleId_++;
    vehicles_.push_back(std::make_unique<Vehicle>(
        id, origin, dest, std::move(route), *cfg
    ));
    return id;
}

// время, светофоры, движение, очереди и спавн-ограничения
void TrafficWorld::step(float dtSeconds)
{
    const float dtSimSeconds = dtSeconds * timeScale_;

    simSeconds_ += dtSimSeconds;

    const float maxSimSeconds = 124.0f * 1000.0f; 
    if (simSeconds_ > maxSimSeconds) {
        simSeconds_ = std::fmod(simSeconds_, maxSimSeconds);
    }

    if (autoMode_) {
        advanceTimeAndEvents(dtSimSeconds);
    }

    std::vector<LightColor> colors;
    colors.reserve(trafficLights_.size());
    for (const auto& tl : trafficLights_) {
        colors.push_back(trafficLightColor(tl));
    }

    const float kMinGapMeters = 6.0f;

    auto unitDirAt = [&](const Path* p, float d) -> Vec2 {
        if (!p || p->length <= 1e-3f) return {1.0f, 0.0f};
        float d0 = std::max(0.0f, d - 1.0f);
        float d1 = std::min(p->length, d + 1.0f);
        Vec2 a = p->positionAt(d0);
        Vec2 b = p->positionAt(d1);
        Vec2 v { b.x - a.x, b.y - a.y };
        float len = std::sqrt(v.x*v.x + v.y*v.y);
        if (len > 1e-4f) { v.x /= len; v.y /= len; }
        else v = {1.0f, 0.0f};
        return v;
    };

    struct Sample {
        Vehicle* v;
        Vec2 pos;
        Vec2 dir;
        float halfLen;
        int lane;
        int pathId;
    };

    std::vector<Sample> samples;
    samples.reserve(vehicles_.size());

    std::unordered_map<Vehicle*, std::size_t> sampleIndex;
    sampleIndex.reserve(vehicles_.size() * 2);

    for (auto& vPtr : vehicles_) {
        if (!vPtr) continue;
        if (vPtr->finished()) continue;

        const Path* p = vPtr->path();
        if (!p) continue;

        Sample s;
        s.v = vPtr.get();
        s.pos = p->positionAt(vPtr->traveledDistance());
        s.dir = unitDirAt(p, vPtr->traveledDistance());
        s.halfLen = vPtr->config().length * 0.5f;
        s.lane = vPtr->laneIndex();
        s.pathId = p->id;

        sampleIndex[s.v] = samples.size();
        samples.push_back(s);
    }

    struct VehicleOnPath { Vehicle* v; };
    std::unordered_map<int, std::vector<VehicleOnPath>> byPath;
    byPath.reserve(paths_.size());

    for (auto& vPtr : vehicles_) {
        if (!vPtr) continue;
        if (vPtr->finished()) continue;

        const Path* p = vPtr->path();
        if (!p) continue;

        byPath[p->id].push_back(VehicleOnPath{ vPtr.get() });
    }

    pathVehicleDensity_.clear();
    for (auto& kv : byPath) {
        int pathId = kv.first;
        auto& vec = kv.second;

        const Path* pConst = getPath(pathId);
        if (!pConst || pConst->length <= 0.0f) continue;

        int laneCount = std::max(1, pConst->laneCount);

        float occupied = 0.0f;
        for (auto& ref : vec) {
            Vehicle* v = ref.v;
            occupied += v->config().length + kMinGapMeters;
        }

        float capacityLen = pConst->length * laneCount;
        if (capacityLen <= 0.0f) continue;

        float rho = occupied / capacityLen;
        pathVehicleDensity_[pathId] = rho;

        float cong = pathCongestionFactor(pathId);
        auto itMut = paths_.find(pathId);
        if (itMut != paths_.end()) itMut->second.congestionFactor = cong;
    }

    std::unordered_map<int, float> intersectionStopForPath;

    const bool useIntersectionYield = (trafficLights_.empty() || !lightsEnabled_);
    if (useIntersectionYield) {
        const float stopMargin = 3.0f;
        const float influenceRadius = 40.0f;

        for (const auto& inter : intersections_) {
            struct Candidate {
                Vehicle* v;
                float distToEnd;
                float pathLen;
                int   pathId;
                int   priority;
                TurnType turn;
            };

            const float insideRadius = 10.0f;

            bool intersectionOccupied = false;
            std::unordered_map<int, bool> occupantPaths;

            for (int pathId : inter.incomingPaths) {
                auto itByPath = byPath.find(pathId);
                if (itByPath == byPath.end()) continue;

                const Path* p = getPath(pathId);
                if (!p) continue;

                for (auto& ref : itByPath->second) {
                    Vehicle* v = ref.v;
                    float d = v->traveledDistance();
                    float distToEnd = p->length - d;
                    if (distToEnd < insideRadius) {
                        intersectionOccupied = true;
                        occupantPaths[pathId] = true;
                    }
                }
            }

            auto resolveTurn = [&](int inPathId, int outPathId) -> TurnType {
                for (const auto& mv : inter.movements) {
                    if (mv.inPathId == inPathId && mv.outPathId == outPathId) return mv.turn;
                }
                return TurnType::Straight;
            };

            std::vector<Candidate> cands;
            for (int pathId : inter.incomingPaths) {
                auto itByPath = byPath.find(pathId);
                if (itByPath == byPath.end()) continue;

                const Path* p = getPath(pathId);
                if (!p) continue;

                Vehicle* best = nullptr;
                float bestDist = -1.0f;

                for (auto& ref : itByPath->second) {
                    Vehicle* v = ref.v;
                    float d = v->traveledDistance();
                    if (d > bestDist) { bestDist = d; best = v; }
                }
                if (!best) continue;

                float distToEnd = std::max(p->length - bestDist, 0.0f);
                if (distToEnd > influenceRadius) continue;

                int prio = 1;
                auto itPr = inter.pathPriority.find(pathId);
                if (itPr != inter.pathPriority.end()) prio = itPr->second;

                const Path* nextP = best->nextPathInRoute();
                int outPathId = nextP ? nextP->id : pathId;

                TurnType turn = resolveTurn(pathId, outPathId);
                cands.push_back(Candidate{ best, distToEnd, p->length, pathId, prio, turn });
            }

            if (cands.empty()) continue;

            if (intersectionOccupied) {
                for (const auto& ci : cands) {
                    if (occupantPaths.count(ci.pathId)) continue;

                    float stopDist = ci.pathLen - (stopMargin + ci.v->config().length * 0.5f);
                    stopDist = std::max(stopDist, 0.0f);

                    auto itStop = intersectionStopForPath.find(ci.pathId);
                    if (itStop == intersectionStopForPath.end()) intersectionStopForPath[ci.pathId] = stopDist;
                    else itStop->second = std::min(itStop->second, stopDist);
                }
                continue;
            }

            std::vector<bool> canGo(cands.size(), true);
            for (std::size_t i = 0; i < cands.size(); ++i) {
                for (std::size_t j = 0; j < cands.size(); ++j) {
                    if (i == j) continue;
                    const auto& ci = cands[i];
                    const auto& cj = cands[j];

                    bool bothInZone = (ci.distToEnd <= influenceRadius && cj.distToEnd <= influenceRadius);
                    if (!bothInZone) continue;

                    if (cj.priority > ci.priority) { canGo[i] = false; break; }

                    if (cj.priority == ci.priority) {
                        if (ci.turn == TurnType::Left &&
                            (cj.turn == TurnType::Straight || cj.turn == TurnType::Right)) {
                            canGo[i] = false;
                            break;
                        }
                    }
                }
            }

            int winnerIndex = -1;
            float bestScore = -1e9f;
            for (std::size_t i = 0; i < cands.size(); ++i) {
                if (!canGo[i]) continue;
                const auto& c = cands[i];

                float turnBonus = 0.0f;
                switch (c.turn) {
                    case TurnType::Right:    turnBonus = 0.2f;  break;
                    case TurnType::Straight: turnBonus = 0.1f;  break;
                    case TurnType::Left:     turnBonus = 0.0f;  break;
                    case TurnType::UTurn:    turnBonus = -0.2f; break;
                }

                float score = c.priority * 10.0f + turnBonus * 5.0f - c.distToEnd * 0.1f;
                if (score > bestScore) { bestScore = score; winnerIndex = (int)i; }
            }

            for (std::size_t i = 0; i < cands.size(); ++i) {
                const auto& ci = cands[i];
                bool mustYield = !canGo[i] || ((int)i != winnerIndex);
                if (!mustYield) continue;

                float stopDist = ci.pathLen - (stopMargin + ci.v->config().length * 0.5f);
                stopDist = std::max(stopDist, 0.0f);

                auto itStop = intersectionStopForPath.find(ci.pathId);
                if (itStop == intersectionStopForPath.end()) intersectionStopForPath[ci.pathId] = stopDist;
                else itStop->second = std::min(itStop->second, stopDist);
            }
        }
    }

    struct LaneVehicle { Vehicle* v; };
    std::unordered_map<int, std::unordered_map<int, std::vector<LaneVehicle>>> byPathLane;

    for (auto& vPtr : vehicles_) {
        if (!vPtr) continue;
        if (vPtr->finished()) continue;

        const Path* p = vPtr->path();
        if (!p) continue;

        int pathId = p->id;
        int laneCount = std::max(1, p->laneCount);

        int lane = vPtr->laneIndex();
        if (lane < 0 || lane >= laneCount) lane = 0;

        byPathLane[pathId][lane].push_back(LaneVehicle{ vPtr.get() });
    }

    for (auto& pk : byPathLane) {
        int pathId = pk.first;

        const Path* p = getPath(pathId);
        if (!p) continue;

        auto itStop = intersectionStopForPath.find(pathId);
        bool hasStopLimit = (itStop != intersectionStopForPath.end());
        float pathStopLimit = hasStopLimit ? itStop->second : std::numeric_limits<float>::infinity();

        for (auto& lk : pk.second) {
            auto& laneVec = lk.second;
            if (laneVec.empty()) continue;

            std::sort(laneVec.begin(), laneVec.end(),
                      [](const LaneVehicle& a, const LaneVehicle& b) {
                          return a.v->traveledDistance() < b.v->traveledDistance();
                      });

            Vehicle* leader = nullptr;
            float leaderDist = 0.0f;
            float leaderLen  = 0.0f;

            for (int idx = (int)laneVec.size() - 1; idx >= 0; --idx) {
                Vehicle* v = laneVec[idx].v;

                float maxDistance = std::numeric_limits<float>::infinity();
                float allowedCenter = std::numeric_limits<float>::infinity();

                if (leader) {
                    float followerLen = v->config().length;
                    allowedCenter =
                        leaderDist - (leaderLen * 0.5f + followerLen * 0.5f + kMinGapMeters);
                    maxDistance = allowedCenter;
                }

                if (!leader && hasStopLimit && maxDistance > pathStopLimit) {
                    maxDistance = pathStopLimit;
                }

                bool greenAheadOnMyPath = false;
                {
                    const Path* vp = v->path();
                    if (vp && !trafficLights_.empty() && trafficLights_.size() == colors.size()) {
                        float dNow = v->traveledDistance();
                        int nearestIdx = -1;
                        float nearestDelta = 1e9f;
                        
                        for (std::size_t li = 0; li < trafficLights_.size(); ++li) {
                            const TrafficLight& tl = trafficLights_[li];
                            if (tl.pathId != vp->id) continue;
                            float delta = tl.distanceOnPath - dNow;
                            if (delta < -2.0f || delta > 40.0f) continue;
                            if (delta < nearestDelta) {
                                nearestDelta = delta;
                                nearestIdx = static_cast<int>(li);
                            }
                        }
                        
                        if (nearestIdx >= 0 && colors[nearestIdx] == LightColor::Green) {
                            greenAheadOnMyPath = true;
                        }
                    }
                }

                if (!leader) {
                    const float alignTol = 0.5f;
                    const float perpTol  = 6.0f;
                    const float maxLookAhead = 120.0f;

                    const Path* vp = v->path();
                    if (vp) {
                        Vec2 myPos = vp->positionAt(v->traveledDistance());
                        Vec2 myDir = unitDirAt(vp, v->traveledDistance());
                        float myHalf = v->config().length * 0.5f;
                        int myLane = v->laneIndex();

                        float bestFreeAlong = std::numeric_limits<float>::infinity();

                        for (const Sample& s : samples) {
                            if (s.v == v) continue;
                            if (s.lane != myLane) continue;

                            if (greenAheadOnMyPath && s.pathId != pathId) continue;

                            float align = myDir.x*s.dir.x + myDir.y*s.dir.y;
                            if (align < alignTol) continue;

                            Vec2 to { s.pos.x - myPos.x, s.pos.y - myPos.y };

                            float proj = to.x*myDir.x + to.y*myDir.y;
                            if (proj <= 0.0f) continue;
                            if (proj > maxLookAhead) continue;

                            float perp = std::abs(to.x*myDir.y - to.y*myDir.x);
                            if (perp > perpTol) continue;

                            float freeAlong = proj - (s.halfLen + myHalf + kMinGapMeters);
                            if (freeAlong < bestFreeAlong) bestFreeAlong = freeAlong;
                        }

                        if (std::isfinite(bestFreeAlong)) {
                            float geomMax = v->traveledDistance() + std::max(bestFreeAlong, 0.0f);
                            if (geomMax < maxDistance) maxDistance = geomMax;
                        }
                    }
                }

                const int beforePathId = (v->path() ? v->path()->id : -1);
                v->step(dtSimSeconds, conditions_, trafficLights_, colors, maxDistance);
                const int afterPathId  = (v->path() ? v->path()->id : -1);

                {
                    auto it = sampleIndex.find(v);
                    if (it != sampleIndex.end()) {
                        std::size_t si = it->second;
                        const Path* sp = v->path();
                        if (sp) {
                            samples[si].pos     = sp->positionAt(v->traveledDistance());
                            samples[si].dir     = unitDirAt(sp, v->traveledDistance());
                            samples[si].halfLen = v->config().length * 0.5f;
                            samples[si].lane    = v->laneIndex();
                            samples[si].pathId  = sp->id;
                        }
                    }
                }

                if (leader) {
                    const Path* vp2 = v->path();
                    if (vp2 && vp2->id == pathId) {
                        float cur = v->traveledDistance();
                        if (std::isfinite(allowedCenter) && cur > allowedCenter) {
                            v->forceSetDistanceAndStop(std::max(allowedCenter, 0.0f));
                        }
                    }
                }

                if (beforePathId == pathId && afterPathId == pathId) {
                    leader     = v;
                    leaderLen  = v->config().length;
                    leaderDist = v->traveledDistance();
                } else {
                    leader     = nullptr;
                    leaderLen  = 0.0f;
                    leaderDist = 0.0f;
                }
            }
        }
    }

    vehicles_.erase(
        std::remove_if(vehicles_.begin(), vehicles_.end(),
                       [](const std::unique_ptr<Vehicle>& v) {
                           return !v || v->finished();
                       }),
        vehicles_.end()
    );
}



// принудительно выставляет позицию на пути и останавливает машину
void Vehicle::forceSetDistanceAndStop(float newDistance)
{
    const Path* p = currentPath();

    if (newDistance < 0.0f) {
        newDistance = 0.0f;
    }

    if (p && newDistance > p->length) {
        newDistance = p->length;
    }

    distance_ = newDistance;
    speed_    = 0.0f;
}

// возвращает указатель на машину по id, либо nullptr
const Vehicle* TrafficWorld::getVehicle(int vehicleId) const
{
    for (const auto& v : vehicles_) {
        if (v && v->id() == vehicleId) {
            return v.get();
        }
    }
    return nullptr;
}

// формирует краткий tooltip по машине
std::string TrafficWorld::buildTooltipForVehicle(int vehicleId) const
{
    const Vehicle* v = getVehicle(vehicleId);
    if (!v) return {};

    const Place* dest = v->destination();
    float sec = v->remainingTimeSeconds(conditions_);

    int minutes;
    if (std::isinf(sec)) {
        minutes = -1;
    } else {
        minutes = static_cast<int>(std::round(sec / 60.0f));
        if (minutes < 1) minutes = 1;
    }

    const Place* orig = v->origin();
    float speedMps = v->estimatedSpeedMps(conditions_);
    float speedKmh = speedMps * 3.6f;

    std::ostringstream ss;
    ss << "Модель: " << v->config().name;
    ss << "\nОткуда: " << (orig ? orig->name : "неизвестно");
    ss << "\nКуда: " << (dest ? dest->name : "неизвестно");
    ss << "\nСкорость: ~" << static_cast<int>(std::round(speedKmh)) << " км/ч";
    ss << "\nОсталось ехать: ";

    if (minutes < 0) {
        ss << "∞ мин";
    } else {
        ss << "~" << minutes << " мин";
    }

    return ss.str();
}

// удаляет случайные активные машины из мира
void TrafficWorld::removeRandomVehicles(int count)
{
    if (count <= 0) return;

    std::vector<std::size_t> indices;
    indices.reserve(vehicles_.size());

    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        if (vehicles_[i] && !vehicles_[i]->finished()) {
            indices.push_back(i);
        }
    }

    if (indices.empty()) return;

    if (count > static_cast<int>(indices.size())) {
        count = static_cast<int>(indices.size());
    }

    std::shuffle(indices.begin(), indices.begin() + count, rng_);
    std::sort(indices.begin(), indices.begin() + count, std::greater<std::size_t>());

    for (int k = 0; k < count; ++k) {
        std::size_t idx = indices[k];
        vehicles_.erase(vehicles_.begin() + static_cast<long>(idx));
    }
}

// включает/выключает ручной режим ДТП
void TrafficWorld::setManualAccident(bool enabled)
{
    manualAccident_ = enabled;
    updateConditionsFlags();
}

// включает/выключает ручной режим дорожных работ
void TrafficWorld::setManualRoadWork(bool enabled)
{
    manualRoadWork_ = enabled;
    updateConditionsFlags();
}

// включает/выключает ручной режим гололёда
void TrafficWorld::setManualIce(bool enabled)
{
    manualIce_ = enabled;
    updateConditionsFlags();
}

// устанавливает флаг ДТП на конкретном пути
void TrafficWorld::setPathAccident(int pathId, bool on)
{
    auto it = paths_.find(pathId);
    if (it != paths_.end()) {
        it->second.hasAccident = on;
    }
}

// сбрасывает флаги ДТП на всех путях.
void TrafficWorld::clearAllAccidents()
{
    for (auto& kv : paths_) {
        kv.second.hasAccident = false;
    }
}

// устанавливает флаг дорожных работ на конкретном пути
void TrafficWorld::setPathRoadWork(int pathId, bool on)
{
    auto it = paths_.find(pathId);
    if (it != paths_.end()) {
        it->second.hasRoadWork = on;
    }
}

// сбрасывает флаги дорожных работ на всех путях
void TrafficWorld::clearAllRoadWorks()
{
    for (auto& kv : paths_) {
        kv.second.hasRoadWork = false;
    }
}

// устанавливает флаг гололёда на конкретном пути
void TrafficWorld::setPathIce(int pathId, bool on)
{
    auto it = paths_.find(pathId);
    if (it != paths_.end()) {
        it->second.hasIce = on;
    }
}

// сбрасывает флаги гололёда на всех путях
void TrafficWorld::clearAllIce()
{
    for (auto& kv : paths_) {
        kv.second.hasIce = false;
    }
}

// возвращает оценку скорости
float Vehicle::estimatedSpeedMps(const TrafficConditions& conditions) const
{
    return effectiveSpeed(conditions);
}

}
