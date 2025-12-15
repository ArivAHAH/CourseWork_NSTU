#pragma once

#include <QColor>
#include <QElapsedTimer>
#include <QPixmap>
#include <QRect>
#include <QWidget>
#include <QFont>
#include <QFontMetrics>
#include <QLinearGradient>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QRadialGradient>
#include <QResizeEvent>
#include <QWheelEvent>
#include <QRandomGenerator>


#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <unordered_map>

#include "simulation_backend.h"
#include "traffic_core.h"

class QPainter;
class QPaintEvent;
class QResizeEvent;
class QWheelEvent;
class QMouseEvent;

namespace app {

class SimulationView : public QWidget
{
    Q_OBJECT
public:
    explicit SimulationView(SimulationBackend* backend, QWidget* parent = nullptr)
        : QWidget(parent)
        , backend_(backend)
    {
        setObjectName("SimulationView");
        setMinimumSize(400, 300);
        setAutoFillBackground(false);

        if (backend_) {
            connect(backend_, &SimulationBackend::worldUpdated,
                    this, &SimulationView::onWorldUpdated);
            rebuildBounds();
        }
    }

    void setTrafficLightIcon(const QPixmap& pix);

signals:
    void homeRequested();
    void settingsRequested();

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;

private slots:
    void onWorldUpdated();

private:
    QPointF       mapToScreen(const traffic::Vec2& pos) const;
    traffic::Vec2 screenToWorld(const QPointF& p) const;
    void          rebuildBounds();

    void drawBackground(QPainter& p, bool cheapMode);
    void drawNightSky(QPainter& p);
    void drawRoads(QPainter& p, bool cheapMode);
    void drawPlaces(QPainter& p, bool cheapMode);
    void drawTrafficLights(QPainter& p, bool cheapMode);
    void drawVehicles(QPainter& p, bool cheapMode);
    void drawHud(QPainter& p);
    void drawIncidents(QPainter& p);
    void drawWinterDecor(QPainter& p);

    QColor colorForPlace(const traffic::Place* place) const;
    QColor colorForVehicle(const traffic::Vehicle* v) const;
    void   ensurePlaceColorMap() const;

private:
    SimulationBackend* backend_ = nullptr;

    bool  boundsValid_ = false;
    float minX_ = 0.0f;
    float maxX_ = 0.0f;
    float minY_ = 0.0f;
    float maxY_ = 0.0f;
    float baseScale_   = 1.0f;
    float baseOffsetX_ = 0.0f;
    float baseOffsetY_ = 0.0f;

    float zoom_    = 1.0f;
    float minZoom_ = 0.4f;
    float maxZoom_ = 12.0f;

    QPixmap trafficLightIcon_;
    bool    hasTrafficLightIcon_ = false;

    mutable std::unordered_map<int, QColor> placeColors_;
    mutable bool placeColorsBuilt_ = false;

    QElapsedTimer zoomTimer_;
    bool zooming_ = false;

    QRect homeButtonRect_;
    QRect settingsButtonRect_;
};

}
