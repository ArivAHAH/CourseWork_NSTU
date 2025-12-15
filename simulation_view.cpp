#include "simulation_view.h"

using traffic::TrafficWorld;
using traffic::Path;
using traffic::Place;
using traffic::TrafficLight;
using traffic::Vehicle;
using traffic::LightColor;
using traffic::VehicleKind;
using traffic::PlaceCategory;
using traffic::Vec2;
using traffic::Season;
using traffic::TimeOfDay;

namespace app {

// устанавливает иконку светофора для отрисовки
void SimulationView::setTrafficLightIcon(const QPixmap& pix)
{
    trafficLightIcon_    = pix;
    hasTrafficLightIcon_ = !trafficLightIcon_.isNull();
    update();
}

// перерисовка по обновлению мира
void SimulationView::onWorldUpdated()
{
    update();
}

// перестраивает bounds при изменении размеров виджета
void SimulationView::resizeEvent(QResizeEvent* e)
{
    QWidget::resizeEvent(e);
    if (backend_) {
        boundsValid_ = false;
        rebuildBounds();
        update();
    }
}

// пересчитывает мировые границы
void SimulationView::rebuildBounds()
{
    boundsValid_ = false;

    if (!backend_) {
        return;
    }

    const TrafficWorld& world = backend_->world();
    const auto& paths = world.paths();
    if (paths.empty()) {
        return;
    }

    bool first = true;
    float minX = 0.0f, maxX = 0.0f;
    float minY = 0.0f, maxY = 0.0f;

    for (const auto& kv : paths) {
        const Path& path = kv.second;
        for (const auto& pt : path.points) {
            if (first) {
                minX = maxX = pt.x;
                minY = maxY = pt.y;
                first = false;
            } else {
                minX = std::min(minX, pt.x);
                maxX = std::max(maxX, pt.x);
                minY = std::min(minY, pt.y);
                maxY = std::max(maxY, pt.y);
            }
        }
    }

    if (first) {
        return;
    }

    const float margin = 15.0f;
    minX_ = minX - margin;
    maxX_ = maxX + margin;
    minY_ = minY - margin;
    maxY_ = maxY + margin;

    const float worldW = maxX_ - minX_;
    const float worldH = maxY_ - minY_;
    if (worldW <= 0.0f || worldH <= 0.0f) {
        return;
    }

    const float w = static_cast<float>(width());
    const float h = static_cast<float>(height());
    if (w <= 0.0f || h <= 0.0f) {
        return;
    }

    const float sx = w / worldW;
    const float sy = h / worldH;
    baseScale_ = 3.0f * std::min(sx, sy);

    const float worldScreenW = worldW * baseScale_;
    const float worldScreenH = worldH * baseScale_;

    baseOffsetX_ = (w - worldScreenW) * 0.5f - minX_ * baseScale_;
    baseOffsetY_ = (h - worldScreenH) * 0.5f + maxY_ * baseScale_;

    if (zoom_ < minZoom_) zoom_ = minZoom_;
    if (zoom_ > maxZoom_) zoom_ = maxZoom_;

    boundsValid_ = true;
}

// преобразует мировую координату в экранную с учётом зума
QPointF SimulationView::mapToScreen(const Vec2& v) const
{
    double baseX = v.x * baseScale_ + baseOffsetX_;
    double baseY = -v.y * baseScale_ + baseOffsetY_;

    double cx = width()  * 0.5;
    double cy = height() * 0.5;

    double x = (baseX - cx) * zoom_ + cx;
    double y = (baseY - cy) * zoom_ + cy;

    return QPointF(x, y);
}

// преобразует экранную координату в мировую с учётом зума
Vec2 SimulationView::screenToWorld(const QPointF& p) const
{
    Vec2 w{0.0f, 0.0f};

    if (baseScale_ <= 0.0f || zoom_ <= 0.0f)
        return w;

    double cx = width()  * 0.5;
    double cy = height() * 0.5;

    double baseX = (p.x() - cx) / zoom_ + cx;
    double baseY = (p.y() - cy) / zoom_ + cy;

    w.x = static_cast<float>((baseX - baseOffsetX_) / baseScale_);

    w.y = static_cast<float>((baseOffsetY_ - baseY) / baseScale_);

    return w;
}

// рисует весь кадр симуляции
void SimulationView::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    if (zooming_) {
        if (zoomTimer_.isValid() && zoomTimer_.elapsed() > 180) {
            zooming_ = false;
        }
    }

    bool cheapMode = zooming_;

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, !cheapMode);

    if (!backend_) {
        QLinearGradient bg(rect().topLeft(), rect().bottomRight());
        bg.setColorAt(0.0, QColor(20, 20, 30));
        bg.setColorAt(1.0, QColor(10, 10, 20));
        p.fillRect(rect(), bg);
        p.setPen(Qt::red);
        p.drawText(rect(), Qt::AlignCenter,
                   tr("Бэкенд симуляции не инициализирован"));
        return;
    }

    const TrafficWorld& world = backend_->world();
    const auto& paths = world.paths();

    const auto& simTime = world.simulationTime();
    Season     season   = simTime.season();

    drawBackground(p, cheapMode);

    if (paths.empty()) {
        p.setPen(Qt::gray);
        p.drawText(rect(), Qt::AlignCenter,
                   tr("В базе нет дорожной сети"));
        return;
    }

    if (cheapMode) {
        drawRoads(p, true);
        drawPlaces(p, true);
        drawTrafficLights(p, true);

        drawVehicles(p, true);

        drawIncidents(p);
        drawWinterDecor(p);
        drawHud(p);
        return;
    }

    drawRoads(p, false);
    drawPlaces(p, false);
    drawTrafficLights(p, false);

    drawVehicles(p, false);

    drawIncidents(p);
    drawWinterDecor(p);
    drawHud(p);
}

// рисует зимний декор
void SimulationView::drawWinterDecor(QPainter& p)
{
    int w = width();
    int h = height();
    if (w <= 0 || h <= 0)
        return;

    p.save();
    p.setRenderHint(QPainter::Antialiasing, true);

    const int garlandY = h / 10;          
    const int marginX  = 0;

    QPainterPath garland;
    garland.moveTo(marginX, garlandY);

    int gx0 = marginX;
    int gx1 = w - marginX;
    int midX = (gx0 + gx1) / 2;

    garland.cubicTo(QPointF(gx0 + (gx1 - gx0) * 0.25, garlandY + 18),
                    QPointF(midX,                     garlandY - 12),
                    QPointF(gx0 + (gx1 - gx0) * 0.75, garlandY + 18));
    garland.cubicTo(QPointF(gx1 - (gx1 - gx0) * 0.15, garlandY + 26),
                    QPointF(gx1 - (gx1 - gx0) * 0.05, garlandY + 6),
                    QPointF(gx1,                      garlandY + 18));

    QPen garlandPen(QColor(180, 190, 210, 220), 2.0);
    p.setPen(garlandPen);
    p.setBrush(Qt::NoBrush);
    p.drawPath(garland);

    static const QVector<QColor> bulbs = {
        QColor(255, 105, 97),  
        QColor(255, 214, 10),   
        QColor(64, 156, 255),  
        QColor(52, 199, 89),    
        QColor(191, 90, 242)   
    };

    int bulbCount = 9;
    for (int i = 0; i < bulbCount; ++i) {
        double t = (i + 0.5) / static_cast<double>(bulbCount);
        QPointF onWire = garland.pointAtPercent(t);

        QPointF bulbCenter(onWire.x(), onWire.y() + 16);

        float radius = 7.0f;
        QRectF bulbRect(bulbCenter.x() - radius,
                        bulbCenter.y() - radius,
                        radius * 2.0f,
                        radius * 2.0f);

        QColor c = bulbs[i % bulbs.size()];

        QRadialGradient rg(bulbCenter, radius);
        rg.setColorAt(0.0, QColor(255, 255, 255, 230));
        rg.setColorAt(0.4, c.lighter(110));
        rg.setColorAt(1.0, c.darker(115));

        p.setPen(QPen(QColor(30, 30, 40, 200), 1.0));
        p.setBrush(rg);
        p.drawEllipse(bulbRect);

        QRectF capRect(bulbCenter.x() - 3.0,
                       bulbCenter.y() - radius - 2.0,
                       6.0,
                       4.0);
        p.setBrush(QColor(80, 80, 90, 220));
        p.drawRoundedRect(capRect, 2.0, 2.0);

        p.setPen(QPen(QColor(150, 160, 180, 220), 1.0));
        p.drawLine(onWire, QPointF(bulbCenter.x(), bulbCenter.y() - radius));
    }

    auto drawSnowflake = [&](QPointF center, float size) {
        p.setPen(QPen(QColor(255, 255, 255, 200), 1.3));
        p.drawLine(QPointF(center.x() - size, center.y()),
                   QPointF(center.x() + size, center.y()));
        p.drawLine(QPointF(center.x(), center.y() - size),
                   QPointF(center.x(), center.y() + size));

        float d = size * 0.7f;
        p.drawLine(QPointF(center.x() - d, center.y() - d),
                   QPointF(center.x() + d, center.y() + d));
        p.drawLine(QPointF(center.x() - d, center.y() + d),
                   QPointF(center.x() + d, center.y() - d));
    };

    float flakeSize = std::max(8, w / 80); 

    drawSnowflake(QPointF(marginX + 20, garlandY - 25), flakeSize);
    drawSnowflake(QPointF(w - marginX - 30, garlandY - 18), flakeSize * 0.9f);

    drawSnowflake(QPointF(marginX + 10, h / 3), flakeSize * 0.8f);
    drawSnowflake(QPointF(w - marginX - 10, h / 2), flakeSize * 0.85f);

    p.restore();
}

// рисует фон с учетом времени
void SimulationView::drawBackground(QPainter& p, bool cheapMode)
{
    Season    season = Season::Summer;
    TimeOfDay tod    = TimeOfDay::Day;

    if (backend_) {
        const auto& simTime = backend_->world().simulationTime();
        season = simTime.season();
        tod    = simTime.timeOfDay();
    }

    QColor top;
    QColor bottom;

    if (tod == TimeOfDay::Night) {
        top    = QColor(5,  8,  20);
        bottom = QColor(10, 15, 35);
    } else if (tod == TimeOfDay::Evening || tod == TimeOfDay::Morning) {
        top    = QColor(40, 30, 70);
        bottom = QColor(200, 140, 120);
    } else {
        if (season == Season::Winter) {
            top    = QColor(220, 230, 240);
            bottom = QColor(200, 210, 225);
        } else if (season == Season::Summer) {
            top    = QColor(246, 244, 235);
            bottom = QColor(231, 242, 243);
        } else {
            top    = QColor(238, 240, 242);
            bottom = QColor(224, 234, 238);
        }
    }

    if (season == Season::Winter && tod != TimeOfDay::Night) {
        top    = top.darker(110);
        bottom = bottom.darker(115);
    }

    QLinearGradient bg(rect().topLeft(), rect().bottomRight());
    bg.setColorAt(0.0, top);
    bg.setColorAt(1.0, bottom);
    p.fillRect(rect(), bg);

    QColor gridColor;
    if (tod == TimeOfDay::Night) {
        gridColor = QColor(40, 50, 70, cheapMode ? 40 : 70);
    } else {
        gridColor = QColor(220, 225, 230, cheapMode ? 40 : 80);
    }

    p.save();
    QPen gridPen(gridColor);
    gridPen.setWidthF(1.0);
    p.setPen(gridPen);

    const int step = 80;
    for (int x = 0; x < width(); x += step) {
        p.drawLine(x, 0, x, height());
    }
    for (int y = 0; y < height(); y += step) {
        p.drawLine(0, y, width(), y);
    }
    p.restore();

    if (tod == TimeOfDay::Night) {
        drawNightSky(p);
    }
}

// рисует ночное небо с ЛУНОЙ!!!
void SimulationView::drawNightSky(QPainter& p)
{
    p.save();
    p.setRenderHint(QPainter::Antialiasing, true);

    p.setPen(Qt::NoPen);
    p.setBrush(QColor(255, 255, 255, 170));

    int stepX = 70;
    int stepY = 60;

    for (int y = 0; y < height(); y += stepY) {
        for (int x = 0; x < width(); x += stepX) {
            int pattern = ((x / stepX) + (y / stepY)) % 4;
            if (pattern == 0) {
                QRectF r(x + 5, y + 3, 2, 2);
                p.drawEllipse(r);
            } else if (pattern == 1) {
                QRectF r(x + 20, y + 18, 1.6, 1.6);
                p.drawEllipse(r);
            }
        }
    }

    int baseSize = std::min(width(), height()) / 8;
    if (baseSize < 40) baseSize = 40;

    QRectF moonRect(width() - baseSize - 40,
                    40,
                    baseSize,
                    baseSize);

    QRadialGradient g(moonRect.center(), baseSize * 0.6);
    g.setColorAt(0.0, QColor(255, 255, 245, 230));
    g.setColorAt(0.4, QColor(255, 255, 230, 180));
    g.setColorAt(1.0, QColor(0, 0, 0, 0));

    p.setBrush(g);
    p.setPen(Qt::NoPen);
    p.drawEllipse(moonRect);

    QRectF cutRect = moonRect.adjusted(baseSize * 0.25, 0,
                                       0, 0);
    p.setBrush(QColor(10, 12, 25, 220));
    p.drawEllipse(cutRect);

    p.restore();
}

// рисуем дороги
void SimulationView::drawRoads(QPainter& p, bool cheapMode)
{
    const TrafficWorld& world = backend_->world();
    const auto& paths = world.paths();

    p.save();
    p.setBrush(Qt::NoBrush);

    for (const auto& kv : paths) {
        const Path& path = kv.second;
        if (path.points.size() < 2) continue;
        if (path.length < 25.0f)    continue;

        QPainterPath painterPath;
        painterPath.moveTo(mapToScreen(path.points.front()));
        for (size_t i = 1; i < path.points.size(); ++i) {
            painterPath.lineTo(mapToScreen(path.points[i]));
        }

        float lanes = std::max(1, path.laneCount);
        float baseWidth = (2.0f + lanes * 1.1f) * zoom_;
        baseWidth = std::clamp(baseWidth, 3.5f, 20.0f);

        float limitKph = std::isfinite(path.speedLimitMps)
            ? path.speedLimitMps * 3.6f : 50.0f;

        QColor roadColor(210, 215, 220);       
        if (limitKph >= 80.0f) {
            roadColor = QColor(195, 205, 215);  
        } else if (limitKph <= 40.0f) {
            roadColor = QColor(218, 220, 225);  
        }

        float cong = path.congestionFactor;
        if (cong > 1.4f) {
            roadColor = QColor(245, 160, 150); 
        } else if (cong > 1.2f) {
            roadColor = QColor(245, 195, 150);  
        }

        if (path.hasAccident) {
            roadColor = QColor(230, 80, 80);
        } else if (path.hasRoadWork) {
            roadColor = QColor(240, 180, 80);
        } else if (path.hasIce) {
            roadColor = QColor(180, 210, 245);
        }

        QPen haloPen(QColor(170, 180, 190, 120));
        haloPen.setWidthF(baseWidth + 4.0f);
        haloPen.setCapStyle(Qt::RoundCap);
        haloPen.setJoinStyle(Qt::RoundJoin);
        p.setPen(haloPen);
        p.drawPath(painterPath);

        QPen roadPen(roadColor);
        roadPen.setWidthF(baseWidth);
        roadPen.setCapStyle(Qt::RoundCap);
        roadPen.setJoinStyle(Qt::RoundJoin);
        p.setPen(roadPen);
        p.drawPath(painterPath);
    }

    p.restore();
}

// генерация цветовой карты
void SimulationView::ensurePlaceColorMap() const
{
    if (placeColorsBuilt_ || !backend_) {
        return;
    }
    placeColorsBuilt_ = true;
    placeColors_.clear();

    const TrafficWorld& world = backend_->world();
    const auto& paths = world.paths();

    static const QVector<QColor> palette = {
        QColor(255, 112, 112),   
        QColor(255, 171, 64),
        QColor(255, 238, 88),    
        QColor(129, 199, 132),   
        QColor(79, 195, 247),    
        QColor(149, 117, 205),   
        QColor(244, 143, 177),   
        QColor(128, 222, 234)
    };

    int idx = 0;

    auto assignIfResidential = [&](int placeId) {
        if (placeId <= 0) return;
        if (placeColors_.find(placeId) != placeColors_.end()) return;

        const Place* pl = world.getPlace(placeId);
        if (!pl) return;

        if (pl->category == PlaceCategory::Residential) {
            QColor c = palette[idx % palette.size()];
            ++idx;
            placeColors_.emplace(placeId, c);
        }
    };

    for (const auto& kv : paths) {
        const Path& p = kv.second;
        assignIfResidential(p.originPlaceId);
        assignIfResidential(p.destinationPlaceId);
    }
}

// возвращает цвет для места по категории
QColor SimulationView::colorForPlace(const Place* place) const
{
    if (!place) {
        return QColor(220, 220, 220);
    }

    ensurePlaceColorMap();

    auto it = placeColors_.find(place->id);
    if (it != placeColors_.end()) {
        return it->second;
    }

    switch (place->category) {
        case PlaceCategory::Residential:
            return QColor(245, 245, 245);
        case PlaceCategory::Office:
            return QColor(200, 210, 230);
        case PlaceCategory::Mall:
            return QColor(230, 200, 210);
        case PlaceCategory::Campus:
            return QColor(200, 230, 210);
        case PlaceCategory::MetroStation:
            return QColor(200, 220, 230);
        case PlaceCategory::Intersection:
        case PlaceCategory::Other:
        default:
            return QColor(210, 210, 210);
    }
}

// возвращает цвет машины на основе хауз
QColor SimulationView::colorForVehicle(const Vehicle* v) const
{
    if (!v) {
        return QColor(255, 255, 255);
    }

    const Place* origin = v->origin();
    const Place* dest   = v->destination();

    const Place* primary = nullptr;
    if (origin && origin->category == PlaceCategory::Residential) {
        primary = origin;
    } else if (dest && dest->category == PlaceCategory::Residential) {
        primary = dest;
    }

    if (!primary) {
        return QColor(255, 255, 255);
    }

    QColor c = colorForPlace(primary);
    return c.darker(115); 
}

// рисует места 
void SimulationView::drawPlaces(QPainter& p, bool cheapMode)
{
    const TrafficWorld& world = backend_->world();
    const auto& paths = world.paths();

    std::unordered_set<int> drawnIds;

    p.save();
    p.setRenderHint(QPainter::Antialiasing, true);

    for (const auto& kv : paths) {
        const Path& path = kv.second;

        auto drawOne = [&](int placeId) {
            if (placeId <= 0) return;
            if (!drawnIds.insert(placeId).second) return;

            const Place* place = world.getPlace(placeId);
            if (!place) return;

            QPointF pt = mapToScreen(place->position);

            float size = 10.0f * zoom_;
            size = std::clamp(size, 7.0f, 18.0f);

            QColor base = colorForPlace(place);
            QColor outline(255, 255, 255, 230);

            if (place->category == PlaceCategory::Residential) {
                QRectF r(pt.x() - size * 0.5f, pt.y() - size * 0.5f,
                         size, size);

                QBrush shadowBrush(QColor(0, 0, 0, 40));
                p.setBrush(shadowBrush);
                QRectF shadowRect = r.adjusted(1.5, 2.0, 1.5, 4.0);
                p.setPen(Qt::NoPen);
                p.drawRoundedRect(shadowRect, size * 0.35f, size * 0.35f);

                QPen pen(outline);
                pen.setWidthF(2.0);
                p.setPen(pen);
                p.setBrush(base);
                p.drawRoundedRect(r, size * 0.35f, size * 0.35f);

                float doorW = size * 0.22f;
                float doorH = size * 0.32f;
                QRectF door(pt.x() - doorW * 0.5f,
                            pt.y() + size * 0.1f,
                            doorW, doorH);
                p.setBrush(QColor(250, 250, 250, 230));
                p.setPen(Qt::NoPen);
                p.drawRoundedRect(door, 2.0, 2.0);
            } else {
                float w = size * 1.2f;
                float h = size * 0.8f;
                QRectF r(pt.x() - w * 0.5f, pt.y() - h * 0.5f, w, h);

                QBrush shadowBrush(QColor(0, 0, 0, 30));
                p.setBrush(shadowBrush);
                QRectF shadowRect = r.adjusted(1.0, 1.5, 1.0, 3.0);
                p.setPen(Qt::NoPen);
                p.drawRoundedRect(shadowRect, 3.0, 3.0);

                QPen pen(QColor(255, 255, 255, 180));
                pen.setWidthF(1.8);
                p.setPen(pen);
                p.setBrush(base);
                p.drawRoundedRect(r, 3.0, 3.0);
            }
        };

        drawOne(path.originPlaceId);
        drawOne(path.destinationPlaceId);
    }

    p.restore();
}

// рисуем светофоры
void SimulationView::drawTrafficLights(QPainter& p, bool cheapMode)
{
    const TrafficWorld& world = backend_->world();
    const auto& lights = world.trafficLights();

    p.save();
    p.setRenderHint(QPainter::Antialiasing, true);

    for (const TrafficLight& light : lights) {
        QPointF pt = mapToScreen(light.position);

        float h = 18.0f * zoom_;
        float w = 7.0f  * zoom_;
        h = std::clamp(h, 10.0f, 26.0f);
        w = std::clamp(w, 4.0f, 10.0f);

        QRectF bodyRect(pt.x() - w * 0.5, pt.y() - h * 0.5, w, h);

        QPainterPath bodyPath;
        bodyPath.addRoundedRect(bodyRect, w * 0.4, w * 0.4);
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(15, 20, 32, 230));
        p.drawPath(bodyPath);

        float margin   = h * 0.10f;
        float lampDiam = (h - 4.0f * margin) / 3.0f;
        lampDiam = std::max(lampDiam, 2.5f);

        QPointF centers[3];
        centers[0] = QPointF(pt.x(), bodyRect.top()  + margin + lampDiam * 0.5f);
        centers[1] = QPointF(pt.x(), centers[0].y() + margin + lampDiam);
        centers[2] = QPointF(pt.x(), centers[1].y() + margin + lampDiam);

        QColor offRed   (90,  40,  40);
        QColor offYellow(90,  90,  40);
        QColor offGreen (40,  90,  50);
        QColor onRed    (235,  70,  70);
        QColor onYellow (245, 220,  80);
        QColor onGreen  ( 80, 225, 110);

        LightColor state = world.trafficLightColor(light);

        QColor colRed   = (state == LightColor::Red)    ? onRed    : offRed;
        QColor colYell  = (state == LightColor::Yellow) ? onYellow : offYellow;
        QColor colGreen = (state == LightColor::Green)  ? onGreen  : offGreen;

        p.setPen(Qt::NoPen);

        p.setBrush(colRed);
        p.drawEllipse(centers[0], lampDiam * 0.5f, lampDiam * 0.5f);

        p.setBrush(colYell);
        p.drawEllipse(centers[1], lampDiam * 0.5f, lampDiam * 0.5f);

        p.setBrush(colGreen);
        p.drawEllipse(centers[2], lampDiam * 0.5f, lampDiam * 0.5f);
    }

    p.restore();
}

// рисуем машинки
void SimulationView::drawVehicles(QPainter& p, bool cheapMode)
{
    const TrafficWorld& world = backend_->world();
    const auto& vehiclesVec = world.vehicles();

    p.save();
    p.setRenderHint(QPainter::Antialiasing, true);

    for (const auto& vPtr : vehiclesVec) {
        const Vehicle* v = vPtr.get();
        if (!v) continue;
        if (v->finished()) continue;

        const Path* path = v->path();

        Vec2 posWorld = v->position();

        double angleDeg = 0.0;
        Vec2 dirWorld{1.0f, 0.0f}; 

        if (path && path->length > 1e-3f) {
            float d = v->traveledDistance();
            float delta = std::min(5.0f, path->length * 0.1f);

            float d0 = std::max(0.0f, d - delta);
            float d1 = std::min(path->length, d + delta);

            Vec2 back  = path->positionAt(d0);
            Vec2 front = path->positionAt(d1);

            float dx = front.x - back.x;
            float dy = front.y - back.y;

            float len = std::sqrt(dx * dx + dy * dy);
            if (len > 1e-3f) {
                dirWorld.x = dx / len;
                dirWorld.y = dy / len;
                angleDeg = std::atan2(-dy, dx) * 180.0 / M_PI;
            }
        }

        Vec2 nWorld;
        nWorld.x = -dirWorld.y;
        nWorld.y =  dirWorld.x;

        int laneIndex = v->laneIndex();
        int laneCount = 1;
        float laneWidthMeters = 3.5f;

        if (path) {
            laneCount = std::max(1, path->laneCount);
            laneWidthMeters = path->laneWidth > 0.1f ? path->laneWidth : 3.5f;
        }

        float laneOffsetIndex = 0.0f;
        if (laneCount > 1) {
            laneOffsetIndex = static_cast<float>(laneIndex) - 0.5f * (laneCount - 1);
        }

        float laneOffsetMeters = laneOffsetIndex * laneWidthMeters;

        posWorld.x += nWorld.x * laneOffsetMeters;
        posWorld.y += nWorld.y * laneOffsetMeters;

        QPointF center = mapToScreen(posWorld);

        float lenMeters   = v->config().length;
        float widthMeters = v->config().width;

        float pixelScale = baseScale_ * zoom_;

        float drawLenMeters   = lenMeters   * 0.75f;
        float drawWidthMeters = widthMeters * 1.15f;

        float lenPx = std::max(18.0f, drawLenMeters   * pixelScale);
        float widPx = std::max(10.0f, drawWidthMeters * pixelScale);

        lenPx = std::clamp(lenPx, 18.0f, 48.0f);
        widPx = std::clamp(widPx, 10.0f, 26.0f);

        QColor bodyColor   = colorForVehicle(v);
        QColor outlineColor(25, 35, 45, 160);

        p.save();
        p.translate(center);
        p.rotate(angleDeg);

        QBrush shadowBrush(QColor(0, 0, 0, 90));
        p.setBrush(shadowBrush);
        p.setPen(Qt::NoPen);
        QRectF shadowRect(-lenPx * 0.5, -widPx * 0.5 + 1.5, lenPx, widPx);
        p.drawRoundedRect(shadowRect, widPx * 0.6, widPx * 0.6);

        QPen outlinePen(outlineColor);
        outlinePen.setWidthF(1.2);
        p.setPen(outlinePen);
        p.setBrush(bodyColor);

        QRectF bodyRect(-lenPx * 0.5, -widPx * 0.5, lenPx, widPx);
        p.drawRoundedRect(bodyRect, widPx * 0.6, widPx * 0.6);

        QRectF glassRect = bodyRect;
        glassRect.setWidth(lenPx * 0.22);
        glassRect.moveRight(bodyRect.right() - 1.0);
        glassRect.adjust(1.0, 1.0, -1.0, -1.0);
        p.setBrush(QColor(245, 250, 255, 230));
        p.setPen(Qt::NoPen);
        p.drawRoundedRect(glassRect, widPx * 0.4, widPx * 0.4);

        p.restore();
    }
    p.restore();
}

// обрабатывает зум колесом мыши
void SimulationView::wheelEvent(QWheelEvent* event)
{
    if (!boundsValid_) {
        event->ignore();
        return;
    }

    const int delta = event->angleDelta().y();
    if (delta == 0) {
        event->ignore();
        return;
    }

    QPointF  mousePos    = event->position();
    Vec2     worldBefore = screenToWorld(mousePos);

    const float factor = (delta > 0) ? 1.15f : (1.0f / 1.15f);
    float newZoom = zoom_ * factor;
    if (newZoom < minZoom_) newZoom = minZoom_;
    if (newZoom > maxZoom_) newZoom = maxZoom_;

    if (qFuzzyCompare(newZoom, zoom_)) {
        event->accept();
        return;
    }

    zoom_ = newZoom;

    double cx = width()  * 0.5;
    double cy = height() * 0.5;

    double targetX = mousePos.x();
    double targetY = mousePos.y();

    double baseX_target = (targetX - cx) / zoom_ + cx;
    double baseY_target = (targetY - cy) / zoom_ + cy;

       baseOffsetX_ = static_cast<float>(baseX_target - worldBefore.x * baseScale_);
    baseOffsetY_ = static_cast<float>(baseY_target + worldBefore.y * baseScale_);

    zooming_ = true;
    zoomTimer_.restart();

    update();
    event->accept();
}

// рисование худа
void SimulationView::drawHud(QPainter& p)
{
    if (!backend_) return;

    const TrafficWorld& world = backend_->world();
    const auto& cond = world.conditions();
    const auto& t    = cond.time;

    const auto& vVec = world.vehicles();
    int activeVehicles = 0;
    for (const auto& v : vVec) {
        if (v && !v->finished()) {
            ++activeVehicles;
        }
    }
    p.save();

    QString timeStr = QString("%1:%2")
        .arg(t.hour,   2, 10, QChar('0'))
        .arg(t.minute, 2, 10, QChar('0'));

    QString title = tr("ЕВА: Городской трафик — симуляция");
    QString stats = tr("Машин в сети: %1").arg(activeVehicles);

    QFont titleFont = font();
    titleFont.setPointSizeF(titleFont.pointSizeF() + 2.0);
    titleFont.setBold(true);

    QFont smallFont = font();
    smallFont.setPointSizeF(smallFont.pointSizeF() - 1.0);

    QFontMetrics fmTitle(titleFont);
    QFontMetrics fmSmall(smallFont);

    int padding = 12;
    int lineH1  = fmTitle.height();
    int lineH2  = fmSmall.height();

    int boxW = std::max(fmTitle.horizontalAdvance(title),
                        fmSmall.horizontalAdvance(stats));
    boxW += padding * 2;

    int boxH = padding * 2 + lineH1 + lineH2 * 2 + 6;

    QRect hudRect(16, 16, boxW, boxH);

    QPainterPath path;
    path.addRoundedRect(hudRect, 10, 10);

    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillPath(path, QColor(10, 14, 25, 210));

    QPen borderPen(QColor(120, 150, 220, 230));
    borderPen.setWidthF(1.0);
    p.setPen(borderPen);
    p.drawPath(path);

    int x = hudRect.left() + padding;
    int y = hudRect.top() + padding + fmTitle.ascent();

    p.setFont(titleFont);
    p.setPen(Qt::white);
    p.drawText(x, y, title);

    y += lineH1;

    p.setFont(smallFont);
    p.setPen(QColor(190, 200, 220));
    p.drawText(x, y, timeStr + "   " + stats);

    y += lineH2;
    p.setPen(QColor(150, 200, 255));

    p.restore();

    int iconSize = 26;
    int margin   = 16;

    QRect settingsRect(width() - iconSize - margin,
                       margin,
                       iconSize,
                       iconSize);

    QRect homeRect(settingsRect.left() - iconSize - 8,
                   margin,
                   iconSize,
                   iconSize);

    homeButtonRect_     = homeRect;
    settingsButtonRect_ = settingsRect;

    QFont iconFont = smallFont;
    iconFont.setPointSizeF(iconFont.pointSizeF() + 3.0);
    p.setFont(iconFont);

    auto drawIconButton = [&](const QRect& r, const QString& glyph) {
        QPainterPath btnPath;
        btnPath.addRoundedRect(r, iconSize * 0.4, iconSize * 0.4);

        p.setPen(Qt::NoPen);
        p.setBrush(QColor(10, 14, 25, 210));
        p.drawPath(btnPath);

        p.setPen(QPen(QColor(180, 190, 220, 200), 1.0));
        p.setBrush(Qt::NoBrush);
        p.drawPath(btnPath);

        p.setPen(QColor(240, 245, 255, 230));
        p.drawText(r, Qt::AlignCenter, glyph);
    };

    drawIconButton(homeRect, QString::fromUtf8("⌂"));
    drawIconButton(settingsRect, QString::fromUtf8("⚙"));
}

// отрисовка событий 
void SimulationView::drawIncidents(QPainter& p)
{
    const TrafficWorld& world = backend_->world();
    const auto& paths = world.paths();

    p.save();
    p.setRenderHint(QPainter::Antialiasing, true);

    for (const auto& kv : paths) {
        const Path& path = kv.second;
        if ((!path.hasAccident && !path.hasRoadWork) || path.length <= 1e-3f)
            continue;

        traffic::Vec2 mid = path.positionAt(path.length * 0.5f);
        QPointF basePt = mapToScreen(mid);

        float iconSize = 14.0f * zoom_;
        iconSize = std::clamp(iconSize, 10.0f, 26.0f);

        QPointF accPt  = basePt;
        QPointF workPt = basePt;

        if (path.hasAccident && path.hasRoadWork) {
            accPt.setX(basePt.x() - iconSize * 0.8f);
            workPt.setX(basePt.x() + iconSize * 0.8f);
        }

        if (path.hasAccident) {
            QPainterPath tri;
            tri.moveTo(accPt.x(),                accPt.y() - iconSize * 0.7f);
            tri.lineTo(accPt.x() - iconSize*0.6, accPt.y() + iconSize * 0.5f);
            tri.lineTo(accPt.x() + iconSize*0.6, accPt.y() + iconSize * 0.5f);
            tri.closeSubpath();

            p.setPen(Qt::NoPen);
            p.setBrush(QColor(220, 70, 70, 230));
            p.drawPath(tri);

            QFont f = p.font();
            f.setBold(true);
            p.setFont(f);
            p.setPen(Qt::white);
            QRectF textRect(accPt.x() - iconSize * 0.4f,
                            accPt.y() - iconSize * 0.2f,
                            iconSize * 0.8f,
                            iconSize * 0.6f);
            p.drawText(textRect, Qt::AlignCenter, "!");
        }

        if (path.hasRoadWork) {
            QPainterPath diamond;
            diamond.moveTo(workPt.x(),                workPt.y() - iconSize * 0.7f);
            diamond.lineTo(workPt.x() - iconSize*0.6, workPt.y());
            diamond.lineTo(workPt.x(),                workPt.y() + iconSize * 0.7f);
            diamond.lineTo(workPt.x() + iconSize*0.6, workPt.y());
            diamond.closeSubpath();

            p.setPen(Qt::NoPen);
            p.setBrush(QColor(240, 170, 60, 230));
            p.drawPath(diamond);

            p.setPen(QPen(QColor(60, 40, 10, 220), 1.3));
            p.drawPath(diamond);
        }
    }

    p.restore();
}

// обработка кнопок хужа
void SimulationView::mousePressEvent(QMouseEvent* event)
{
    if (event->button() != Qt::LeftButton) {
        QWidget::mousePressEvent(event);
        return;
    }

    const QPoint pos = event->pos();

    if (homeButtonRect_.contains(pos)) {
        emit homeRequested();
        event->accept();
        return;
    }

    if (settingsButtonRect_.contains(pos)) {
        emit settingsRequested();
        event->accept();
        return;
    }

    QWidget::mousePressEvent(event);
}
} 