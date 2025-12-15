#pragma once

#include <QRandomGenerator>
#include <QPainter>
#include <QPainterPath>
#include <QMouseEvent>
#include <QResizeEvent>
#include <QtMath>
#include <QWidget>
#include <QVector>
#include <QTimer>
#include <QColor>
#include <QPointF>
#include <QRectF>

#include "simulation_backend.h"
#include "traffic_core.h"

namespace traffic {
enum class Season : std::uint8_t;
enum class TimeOfDay : std::uint8_t;
enum class LightColor : std::uint8_t;
}

namespace app {

class SimulationBackend;

class BuildModeView : public QWidget
{
    Q_OBJECT

public:
    explicit BuildModeView(SimulationBackend* backend, QWidget* parent = nullptr);

signals:
    void homeRequested();
    void settingsRequested();

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private slots:
    void onHomeClicked();
    void onSettingsClicked();
    void onToolHouse();
    void onToolRoad();
    void onToolStraightRoad();
    void onToolLight();
    void onToolErase();
    void onStartSimulation();
    void onStopSimulation();
    void onCarsSliderChanged(int value);
    void onSimTick();
    void onToolRoundabout();

private:
    enum class Tool {
        House,
        Road,
        StraightRoad,
        Light,
        Roundabout,
        Erase
    };

    struct House {
        QPointF pos;
        QColor color;
    };

    struct Road {
        QVector<QPointF> points;
    };

    struct Light {
        QPointF pos;
        int groupId = 0;
        float phaseOffset = 0.0f;
    };

    struct Car {
        QColor color;
        int roadIndex;
        float t;
        float speed;
        bool finished;
        QPointF pos;
        qreal angleDeg;
    };

    struct Node {
        QPointF pos;
        QVector<int> edges;
    };

    SimulationBackend* backend_ = nullptr;

    QRectF sidePanelRect() const;
    QRectF canvasRect() const;

    Tool currentTool_ = Tool::House;

    QVector<House> houses_;
    QVector<Road> roads_;
    QVector<Light> lights_;

    int baseRoadCount_ = 0;

    bool drawingRoad_ = false;
    QVector<QPointF> currentRoad_;

    QVector<QColor> palette_;
    int paletteIndex_ = 0;
    QColor currentPairColor_;
    int currentPairCount_ = 0;

    QTimer simTimer_;
    bool simulationRunning_ = false;
    int carsPerPair_ = 10;
    QVector<Car> cars_;

    QPointF clampToCanvas(const QPointF& p) const;
    int findHouseAt(const QPointF& p, qreal radius) const;
    int findLightAt(const QPointF& p, qreal radius) const;
    int findRoadAt(const QPointF& p, qreal radius, QPointF* nearestPoint = nullptr) const;

    void drawBackground(QPainter& p);
    void drawNightSky(QPainter& p, traffic::TimeOfDay tod);
    void drawGarlandAndSnow(QPainter& p, traffic::Season season, traffic::TimeOfDay tod);
    void drawGrid(QPainter& p, traffic::TimeOfDay tod);
    void drawScene(QPainter& p);
    void drawRoads(QPainter& p);
    void drawHouses(QPainter& p);
    void drawLights(QPainter& p);
    void drawCars(QPainter& p);
    void drawSidePanel(QPainter& p);

    float simSeconds_ = 0.0f;

    void initPalette();
    QColor nextPairColor();
    void addHouseAt(const QPointF& pos);
    void addLightAt(const QPointF& pos);

    void beginRoad(const QPointF& pos);
    void extendRoad(const QPointF& pos);
    void finishRoad(const QPointF& pos);

    void eraseAt(const QPointF& pos);

    bool hasValidPairAndRoad() const;
    void rebuildCars();
    QPointF roadPointAt(const Road& r, float t) const;
    void tickSimulation();

    QRectF homeButtonRect() const;
    QRectF settingsButtonRect() const;
    QRectF toolButtonRect(int index) const;
    QRectF startButtonRect() const;
    QRectF stopButtonRect() const;
    QRectF carsSliderRect() const;

    void updateCarsPerPair(int value);
    traffic::LightColor lightColorFor(const Light& l) const;
    void addRoundaboutAt(const QPointF& center);
    QPointF roadDirectionAt(const Road& r, float t) const;
    double roadLength(const Road& r) const;

    QVector<Node> buildRoadGraph() const;
    int closestNode(const QVector<Node>& graph, const QPointF& hpos) const;
    bool reachable(const QVector<Node>& graph, int start, int goal) const;
};

}
