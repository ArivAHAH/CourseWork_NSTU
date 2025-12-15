#include "build_mode_view.h"

using namespace traffic;

namespace app {

// возвращает квадрат расстояния между двумя точками
static qreal dist2(const QPointF& a, const QPointF& b)
{
    qreal dx = a.x() - b.x();
    qreal dy = a.y() - b.y();
    return dx * dx + dy * dy;
}

// радиус в котором считаем узлы соединёнными
static constexpr float NODE_EPS = 35.0f;

// сглаживает полилинию 
static QVector<QPointF> smoothPolylineChaikin(const QVector<QPointF>& pts, int iterations = 2)
{
    if (pts.size() < 3 || iterations <= 0) {
        return pts;
    }

    QVector<QPointF> result = pts;

    for (int it = 0; it < iterations; ++it) {
        if (result.size() < 3) {
            break;
        }

        QVector<QPointF> tmp;
        tmp.reserve(result.size() * 2);

        tmp.push_back(result.first());

        for (int i = 0; i + 1 < result.size(); ++i) {
            const QPointF& p0 = result[i];
            const QPointF& p1 = result[i + 1];

            QPointF q1 = 0.75 * p0 + 0.25 * p1;
            QPointF q2 = 0.25 * p0 + 0.75 * p1;

            tmp.push_back(q1);
            tmp.push_back(q2);
        }

        tmp.push_back(result.last());
        result.swap(tmp);
    }

    return result;
}

// создаёт виджет режима строительства и запускает таймер тика
BuildModeView::BuildModeView(SimulationBackend* backend, QWidget* parent)
    : QWidget(parent)
    , backend_(backend)
{
    setMouseTracking(true);

    initPalette();
    currentPairColor_ = nextPairColor();
    currentPairCount_ = 0;
    carsPerPair_ = 10;

    simTimer_.setInterval(30);
    connect(&simTimer_, &QTimer::timeout, this, &BuildModeView::onSimTick);

    simTimer_.start();
}

// инициализирует палитру цветов для пар домиков
void BuildModeView::initPalette()
{
    palette_.clear();
    palette_.push_back(QColor("#FFD54F"));
    palette_.push_back(QColor("#4FC3F7"));
    palette_.push_back(QColor("#FF8A65"));
    palette_.push_back(QColor("#81C784"));
    palette_.push_back(QColor("#BA68C8"));
    palette_.push_back(QColor("#FF5252"));
    paletteIndex_ = 0;
}

// возвращает следующий цвет для текущей пары домиков
QColor BuildModeView::nextPairColor()
{
    if (palette_.isEmpty()) {
        return QColor("#FFFFFF");
    }
    QColor c = palette_.at(paletteIndex_ % palette_.size());
    ++paletteIndex_;
    return c;
}

// возвращает прямоугольник правой панели управления
QRectF BuildModeView::sidePanelRect() const
{
    const int w = width();
    const int h = height();
    const int panelW = qMax(220, w / 5);
    return QRectF(w - panelW, 0, panelW, h);
}

// возвращает прямоугольник области рисования
QRectF BuildModeView::canvasRect() const
{
    QRectF sp = sidePanelRect();
    return QRectF(0, 0, sp.left(), height());
}

// ограничивает точку границами области рисования
QPointF BuildModeView::clampToCanvas(const QPointF& p) const
{
    QRectF c = canvasRect();
    QPointF r = p;
    if (r.x() < c.left()) r.setX(c.left());
    if (r.x() > c.right()) r.setX(c.right());
    if (r.y() < c.top()) r.setY(c.top());
    if (r.y() > c.bottom()) r.setY(c.bottom());
    return r;
}

// отправляет сигнал возврата на главный экран
void BuildModeView::onHomeClicked()
{
    emit homeRequested();
}

// отправляет сигнал открытия настроек
void BuildModeView::onSettingsClicked()
{
    emit settingsRequested();
}

// активирует инструмент добавления домиков
void BuildModeView::onToolHouse()
{
    currentTool_ = Tool::House;
    update();
}

// активирует инструмент рисования обычной дороги
void BuildModeView::onToolRoad()
{
    currentTool_ = Tool::Road;
    update();
}

// Аативирует инструмент рисования прямой дороги
void BuildModeView::onToolStraightRoad()
{
    currentTool_ = Tool::StraightRoad;
    update();
}

// активирует инструмент добавления светофоров
void BuildModeView::onToolLight()
{
    currentTool_ = Tool::Light;
    update();
}

// активирует инструмент удаления объектов
void BuildModeView::onToolErase()
{
    currentTool_ = Tool::Erase;
    update();
}

// обновляет число машин на пару домиков
void BuildModeView::updateCarsPerPair(int v)
{
    carsPerPair_ = qBound(1, v, 30);
    update();
}

// обрабатывает изменение “слайдера” количества машин
void BuildModeView::onCarsSliderChanged(int value)
{
    updateCarsPerPair(value);
}

// запускает симуляцию машин в режиме строительства
void BuildModeView::onStartSimulation()
{
    rebuildCars();

    if (cars_.isEmpty()) {
        return;
    }

    simulationRunning_ = true;
    simTimer_.start();
}

// останавливает симуляцию машин и сбрасывает список машин
void BuildModeView::onStopSimulation()
{
    simulationRunning_ = false;
    cars_.clear();
    update();
}

// вызывается таймером и двигает симуляцию на один тик
void BuildModeView::onSimTick()
{
    tickSimulation();
}

// обрабатывает изменение размера виджета
void BuildModeView::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    update();
}

// рисует фон сцену и панель управления
void BuildModeView::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    drawBackground(p);
    drawScene(p);
    drawSidePanel(p);
}

// обрабатывает нажатия мыши
void BuildModeView::mousePressEvent(QMouseEvent* event)
{
    const QPointF pos = event->pos();

    if (sidePanelRect().contains(pos)) {
        if (homeButtonRect().contains(pos)) {
            onHomeClicked();
            return;
        }
        if (settingsButtonRect().contains(pos)) {
            onSettingsClicked();
            return;
        }

        for (int i = 0; i < 6; ++i) {
            if (toolButtonRect(i).contains(pos)) {
                switch (i) {
                case 0: onToolHouse(); break;
                case 1: onToolRoad(); break;
                case 2: onToolStraightRoad(); break;
                case 3: onToolLight(); break;
                case 4: onToolRoundabout(); break;
                case 5: onToolErase(); break;
                }
                return;
            }
        }

        if (startButtonRect().contains(pos)) {
            onStartSimulation();
            return;
        }
        if (stopButtonRect().contains(pos)) {
            onStopSimulation();
            return;
        }

        if (carsSliderRect().contains(pos)) {
            QRectF r = carsSliderRect();
            if (r.width() > 0) {
                qreal t = (pos.x() - r.left()) / r.width();
                int val = 1 + int(t * 29.999);
                updateCarsPerPair(val);
            }
            return;
        }

        return;
    }

    QPointF cpos = clampToCanvas(pos);

    if (event->button() == Qt::LeftButton) {
        switch (currentTool_) {
        case Tool::House:
            addHouseAt(cpos);
            break;
        case Tool::Light:
            addLightAt(cpos);
            break;
        case Tool::Road:
        case Tool::StraightRoad:
            beginRoad(cpos);
            break;
        case Tool::Roundabout:
            addRoundaboutAt(cpos);
            break;
        case Tool::Erase:
            eraseAt(cpos);
            break;
        }
    }

    QWidget::mousePressEvent(event);
}

// обрабатывает перемещение мыши при рисовании дороги
void BuildModeView::mouseMoveEvent(QMouseEvent* event)
{
    QPointF pos = clampToCanvas(event->pos());

    if (drawingRoad_ && (currentTool_ == Tool::Road || currentTool_ == Tool::StraightRoad)) {
        extendRoad(pos);
    }

    QWidget::mouseMoveEvent(event);
}

// завершает рисование дороги при отпускании кнопки
void BuildModeView::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton &&
        drawingRoad_ &&
        (currentTool_ == Tool::Road || currentTool_ == Tool::StraightRoad)) {
        QPointF pos = clampToCanvas(event->pos());
        finishRoad(pos);
    }

    QWidget::mouseReleaseEvent(event);
}

// находит домик рядом с точкой
int BuildModeView::findHouseAt(const QPointF& p, qreal radius) const
{
    const qreal r2 = radius * radius;
    for (int i = 0; i < houses_.size(); ++i) {
        if (dist2(houses_[i].pos, p) <= r2) {
            return i;
        }
    }
    return -1;
}

// находит светофор рядом с точкой
int BuildModeView::findLightAt(const QPointF& p, qreal radius) const
{
    const qreal r2 = radius * radius;
    for (int i = 0; i < lights_.size(); ++i) {
        if (dist2(lights_[i].pos, p) <= r2) {
            return i;
        }
    }
    return -1;
}

// находит дорогу рядом с точкой и опционально возвращает ближайшую точку на сегменте
int BuildModeView::findRoadAt(const QPointF& p, qreal radius, QPointF* nearestPoint) const
{
    const qreal r2 = radius * radius;
    int bestIndex = -1;
    qreal bestD2 = r2;

    for (int i = 0; i < roads_.size(); ++i) {
        const Road& r = roads_[i];
        if (r.points.size() < 2) continue;

        for (int j = 1; j < r.points.size(); ++j) {
            QLineF seg(r.points[j - 1], r.points[j]);
            if (seg.length() < 1e-3) continue;

            QPointF a = seg.p1();
            QPointF b = seg.p2();
            QPointF ap = p - a;
            QPointF ab = b - a;

            double ab2 = ab.x() * ab.x() + ab.y() * ab.y();
            double t = (ab2 > 0) ? (ap.x() * ab.x() + ap.y() * ab.y()) / ab2 : 0.0;
            if (t < 0.0) t = 0.0;
            if (t > 1.0) t = 1.0;

            QPointF proj = a + t * ab;
            qreal d2 = dist2(p, proj);
            if (d2 <= bestD2) {
                bestD2 = d2;
                bestIndex = i;
                if (nearestPoint) *nearestPoint = proj;
            }
        }
    }

    return bestIndex;
}

// добавляет домик в указанной позиции с текущим цветом пары
void BuildModeView::addHouseAt(const QPointF& pos)
{
    if (currentPairCount_ >= 2) {
        currentPairColor_ = nextPairColor();
        currentPairCount_ = 0;
    }

    House h;
    h.pos = pos;
    h.color = currentPairColor_;
    houses_.push_back(h);

    ++currentPairCount_;

    update();
}

// добавляет светофор и пытается определить группу по ближайшей дороге
void BuildModeView::addLightAt(const QPointF& pos)
{
    Light l;
    l.pos = pos;
    l.groupId = 0;

    int rnd = QRandomGenerator::global()->bounded(0, 13000);
    l.phaseOffset = rnd / 1000.0f;

    QPointF nearest;
    int roadIndex = findRoadAt(pos, 40.0, &nearest);
    if (roadIndex >= 0 && roadIndex < roads_.size()) {
        const Road& r = roads_[roadIndex];
        if (r.points.size() >= 2) {
            QPointF a = r.points.first();
            QPointF b = r.points.last();
            QPointF d = b - a;
            if (qAbs(d.x()) >= qAbs(d.y())) {
                l.groupId = 0;
            } else {
                l.groupId = 2;
            }
        }
    }

    lights_.push_back(l);
    update();
}

// начинает рисование дороги
void BuildModeView::beginRoad(const QPointF& pos)
{
    drawingRoad_ = true;
    currentRoad_.clear();
    currentRoad_.push_back(pos);
    update();
}

// добавляет точку в текущую рисуемую дорогу
void BuildModeView::extendRoad(const QPointF& pos)
{
    if (!drawingRoad_) return;
    if (currentRoad_.isEmpty()) {
        currentRoad_.push_back(pos);
        update();
        return;
    }

    QPointF last = currentRoad_.back();
    if (QLineF(last, pos).length() > 8.0) {
        currentRoad_.push_back(pos);
        update();
    }
}

// завершает рисование дороги и сохраняет её в список
void BuildModeView::finishRoad(const QPointF& pos)
{
    if (!drawingRoad_) return;
    drawingRoad_ = false;

    if (currentRoad_.size() < 2) {
        currentRoad_.clear();
        update();
        return;
    }

    QPointF start = currentRoad_.front();
    QPointF end = pos;

    const qreal snapRadius = 45.0;

    int iStart = findHouseAt(start, snapRadius);
    if (iStart >= 0) {
        start = houses_[iStart].pos;
    }

    int iEnd = findHouseAt(end, snapRadius);
    if (iEnd >= 0) {
        end = houses_[iEnd].pos;
    }

    auto snapToRoadAnyPoint = [&](const QPointF& point) -> QPointF {
        QPointF nearest;
        int ri = findRoadAt(point, snapRadius, &nearest);
        if (ri >= 0) {
            return nearest;
        }
        return point;
    };

    start = snapToRoadAnyPoint(start);
    end = snapToRoadAnyPoint(end);

    currentRoad_.front() = start;
    currentRoad_.push_back(end);

    QVector<QPointF> finalPolyline;

    if (currentTool_ == Tool::StraightRoad) {
        finalPolyline.clear();
        finalPolyline.push_back(start);
        finalPolyline.push_back(end);
    } else {
        finalPolyline = currentRoad_;
    }

    Road r;
    r.points = finalPolyline;
    roads_.push_back(r);
    baseRoadCount_ = roads_.size();

    currentRoad_.clear();
    update();
}

// удаляет ближайший объект по позиции
void BuildModeView::eraseAt(const QPointF& pos)
{
    const qreal radius = 18.0;

    int hi = findHouseAt(pos, radius);
    if (hi >= 0) {
        houses_.removeAt(hi);
        update();
        return;
    }

    int li = findLightAt(pos, radius);
    if (li >= 0) {
        lights_.removeAt(li);
        update();
        return;
    }

    int ri = findRoadAt(pos, radius, nullptr);
    if (ri >= 0) {
        roads_.removeAt(ri);
        baseRoadCount_ = roads_.size();
        update();
        return;
    }
}

// строит граф дорожной сети по текущим дорогам
QVector<BuildModeView::Node> BuildModeView::buildRoadGraph() const
{
    QVector<Node> nodes;

    for (const Road& r : roads_) {
        for (const QPointF& pt : r.points) {
            nodes.push_back({pt, {}});
        }
    }

    const int n = nodes.size();

    int index = 0;
    for (const Road& r : roads_) {
        for (int i = 1; i < r.points.size(); ++i) {
            int a = index + i - 1;
            int b = index + i;
            nodes[a].edges.push_back(b);
            nodes[b].edges.push_back(a);
        }
        index += r.points.size();
    }

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (QLineF(nodes[i].pos, nodes[j].pos).length() <= NODE_EPS) {
                nodes[i].edges.push_back(j);
                nodes[j].edges.push_back(i);
            }
        }
    }

    return nodes;
}

// находит ближайший узел графа к позиции дома
int BuildModeView::closestNode(const QVector<Node>& g, const QPointF& hpos) const
{
    int best = -1;
    float bestD = NODE_EPS;

    for (int i = 0; i < g.size(); ++i) {
        float d = QLineF(g[i].pos, hpos).length();
        if (d < bestD) {
            bestD = d;
            best = i;
        }
    }
    return best;
}

// проверяет достижимость узла goal из start по графу дорог
bool BuildModeView::reachable(const QVector<Node>& g, int start, int goal) const
{
    if (start < 0 || goal < 0) return false;

    QVector<bool> vis(g.size(), false);
    QVector<int> q;
    q.push_back(start);
    vis[start] = true;

    for (int qi = 0; qi < q.size(); ++qi) {
        int v = q[qi];
        if (v == goal) return true;

        for (int nx : g[v].edges) {
            if (!vis[nx]) {
                vis[nx] = true;
                q.push_back(nx);
            }
        }
    }
    return false;
}

// проверяет есть ли хотя бы одна пара домиков и связный путь по дорогам между ними
bool BuildModeView::hasValidPairAndRoad() const
{
    if (houses_.size() < 2 || roads_.isEmpty())
        return false;

    QMap<QRgb, QVector<int>> byColor;
    for (int i = 0; i < houses_.size(); ++i) {
        byColor[houses_[i].color.rgb()].push_back(i);
    }

    struct Node {
        QPointF pos;
        QVector<int> edges;
    };

    QVector<Node> graph;
    graph.reserve(256);

    QVector<int> roadOffset;
    roadOffset.reserve(roads_.size());

    for (const Road& r : roads_) {
        int base = graph.size();
        roadOffset.push_back(base);
        for (const QPointF& pt : r.points) {
            Node n;
            n.pos = pt;
            graph.push_back(n);
        }
    }

    const qreal connectR = 35.0;
    const qreal connectR2 = connectR * connectR;

    int idx = 0;
    for (const Road& r : roads_) {
        for (int i = 1; i < r.points.size(); ++i) {
            int a = idx + i - 1;
            int b = idx + i;
            graph[a].edges.push_back(b);
            graph[b].edges.push_back(a);
        }
        idx += r.points.size();
    }

    for (int i = 0; i < graph.size(); ++i) {
        for (int j = i + 1; j < graph.size(); ++j) {
            if (dist2(graph[i].pos, graph[j].pos) <= connectR2) {
                graph[i].edges.push_back(j);
                graph[j].edges.push_back(i);
            }
        }
    }

    auto closestNode = [&](const QPointF& p) -> int {
        int best = -1;
        qreal bestD2 = connectR2;
        for (int i = 0; i < graph.size(); ++i) {
            qreal d2 = dist2(graph[i].pos, p);
            if (d2 < bestD2) {
                bestD2 = d2;
                best = i;
            }
        }
        return best;
    };

    auto reachable = [&](int start, int goal) -> bool {
        if (start < 0 || goal < 0) return false;
        QVector<bool> used(graph.size(), false);
        QVector<int> q;
        q.push_back(start);
        used[start] = true;

        for (int qi = 0; qi < q.size(); ++qi) {
            int v = q[qi];
            if (v == goal) return true;
            for (int to : graph[v].edges) {
                if (!used[to]) {
                    used[to] = true;
                    q.push_back(to);
                }
            }
        }
        return false;
    };

    for (auto it = byColor.begin(); it != byColor.end(); ++it) {
        const QVector<int>& idxs = it.value();
        if (idxs.size() != 2)
            continue;

        const House& h1 = houses_[idxs[0]];
        const House& h2 = houses_[idxs[1]];

        int n1 = closestNode(h1.pos);
        int n2 = closestNode(h2.pos);

        if (reachable(n1, n2)) {
            return true;
        }
    }

    return false;
}

// возвращает точку на полилинии дороги по параметру t
QPointF BuildModeView::roadPointAt(const Road& r, float t) const
{
    if (r.points.size() < 2) {
        return r.points.isEmpty() ? QPointF() : r.points.first();
    }

    QVector<double> segLen;
    segLen.reserve(r.points.size() - 1);
    double total = 0.0;
    for (int i = 1; i < r.points.size(); ++i) {
        double len = QLineF(r.points[i - 1], r.points[i]).length();
        segLen.push_back(len);
        total += len;
    }
    if (total <= 1e-3) {
        return r.points.front();
    }

    double target = t * total;
    double accum = 0.0;
    for (int i = 0; i < segLen.size(); ++i) {
        double seg = segLen[i];
        if (accum + seg >= target) {
            double localT = (target - accum) / seg;
            return r.points[i] + localT * (r.points[i + 1] - r.points[i]);
        }
        accum += seg;
    }
    return r.points.back();
}

// возвращает длину дороги в пикселях
double BuildModeView::roadLength(const Road& r) const
{
    if (r.points.size() < 2) {
        return 0.0;
    }
    double total = 0.0;
    for (int i = 1; i < r.points.size(); ++i) {
        total += QLineF(r.points[i - 1], r.points[i]).length();
    }
    return total;
}

// перестраивает список машин и служебные маршруты между парами домиков
void BuildModeView::rebuildCars()
{
    cars_.clear();

    if (baseRoadCount_ < 0 || baseRoadCount_ > roads_.size()) {
        baseRoadCount_ = roads_.size();
    }
    if (roads_.size() > baseRoadCount_) {
        roads_.resize(baseRoadCount_);
    }

    if (houses_.size() < 2 || baseRoadCount_ == 0) {
        return;
    }

    QMap<QRgb, QVector<int>> byColor;
    for (int i = 0; i < houses_.size(); ++i) {
        byColor[houses_[i].color.rgb()].push_back(i);
    }

    struct Node {
        QPointF pos;
        int roadId;
        QVector<int> adj;
    };

    QVector<Node> nodes;
    nodes.reserve(baseRoadCount_ * 16);

    QVector<int> roadStart(baseRoadCount_, -1);
    QVector<int> roadCount(baseRoadCount_, 0);

    int nodeCounter = 0;
    for (int ri = 0; ri < baseRoadCount_; ++ri) {
        const Road& r = roads_[ri];
        if (r.points.size() < 2)
            continue;

        roadStart[ri] = nodeCounter;
        roadCount[ri] = r.points.size();

        for (const QPointF& p : r.points) {
            Node n;
            n.pos = p;
            n.roadId = ri;
            nodes.push_back(n);
            ++nodeCounter;
        }
    }

    if (nodes.size() < 2) {
        return;
    }

    auto addEdge = [&](int a, int b) {
        if (a == b) return;
        if (!nodes[a].adj.contains(b))
            nodes[a].adj.push_back(b);
        if (!nodes[b].adj.contains(a))
            nodes[b].adj.push_back(a);
    };

    for (int ri = 0; ri < baseRoadCount_; ++ri) {
        int start = roadStart[ri];
        int cnt = roadCount[ri];
        if (start < 0 || cnt < 2)
            continue;

        for (int i = 1; i < cnt; ++i) {
            addEdge(start + i - 1, start + i);
        }
    }

    const qreal joinR = 18.0;
    const qreal joinR2 = joinR * joinR;

    for (int i = 0; i < nodes.size(); ++i) {
        for (int j = i + 1; j < nodes.size(); ++j) {
            if (nodes[i].roadId == nodes[j].roadId)
                continue;

            if (dist2(nodes[i].pos, nodes[j].pos) <= joinR2) {
                addEdge(i, j);
            }
        }
    }

    auto nearestNodeIndex = [&](const QPointF& p) -> int {
        int best = -1;
        qreal bestD2 = joinR2;
        for (int i = 0; i < nodes.size(); ++i) {
            qreal d2 = dist2(nodes[i].pos, p);
            if (d2 < bestD2) {
                bestD2 = d2;
                best = i;
            }
        }
        return best;
    };

    auto bfsPath = [&](int start, int goal) -> QVector<int> {
        if (start < 0 || goal < 0 || start >= nodes.size() || goal >= nodes.size()) {
            return {};
        }

        QVector<int> prev(nodes.size(), -1);
        QVector<int> queue;
        queue.reserve(nodes.size());

        queue.push_back(start);
        prev[start] = start;

        int qi = 0;
        while (qi < queue.size()) {
            int v = queue[qi++];
            if (v == goal)
                break;

            for (int to : nodes[v].adj) {
                if (to < 0 || to >= nodes.size()) continue;
                if (prev[to] != -1) continue;
                prev[to] = v;
                queue.push_back(to);
            }
        }

        if (prev[goal] == -1)
            return {};

        QVector<int> path;
        for (int v = goal;; v = prev[v]) {
            path.push_front(v);
            if (v == start) break;
        }
        return path;
    };

    struct RouteInfo {
        int roadIndex;
        QRgb colorRgb;
    };
    QVector<RouteInfo> routes;

    for (auto it = byColor.begin(); it != byColor.end(); ++it) {
        const QVector<int>& idxsH = it.value();
        if (idxsH.size() != 2)
            continue;

        const House& h1 = houses_[idxsH[0]];
        const House& h2 = houses_[idxsH[1]];

        int nStart = nearestNodeIndex(h1.pos);
        int nEnd = nearestNodeIndex(h2.pos);
        if (nStart < 0 || nEnd < 0)
            continue;

        QVector<int> nodePath = bfsPath(nStart, nEnd);
        if (nodePath.size() < 2)
            continue;

        Road route;
        route.points.reserve(nodePath.size());
        for (int ni : nodePath) {
            route.points.push_back(nodes[ni].pos);
        }

        int newIndex = roads_.size();
        roads_.push_back(route);

        RouteInfo info;
        info.roadIndex = newIndex;
        info.colorRgb = it.key();
        routes.push_back(info);
    }

    if (routes.isEmpty()) {
        return;
    }

    QRandomGenerator* rng = QRandomGenerator::global();

    int totalCars = qBound(1, carsPerPair_, 30);

    for (int i = 0; i < totalCars; ++i) {
        const RouteInfo& route = routes.at(rng->bounded(routes.size()));

        Car c;
        c.color = QColor::fromRgb(route.colorRgb).darker(115);
        c.roadIndex = route.roadIndex;

        int r01 = rng->bounded(10000);
        float rand01 = static_cast<float>(r01) / 10000.0f;
        c.t = rand01 * 0.2f;

        int rSpeed = rng->bounded(10000);
        float randSpeed = static_cast<float>(rSpeed) / 10000.0f;
        c.speed = 0.18f + randSpeed * 0.06f;

        c.finished = false;
        cars_.push_back(c);
    }
}

// делает один тик симуляции машин и обновляет экран
void BuildModeView::tickSimulation()
{
    if (simTimer_.interval() > 0) {
        simSeconds_ += simTimer_.interval() / 1000.0f;
    }

    if (!simulationRunning_) {
        update();
        return;
    }

    float speedFactor = 1.0f;
    if (backend_) {
        const auto& cond = backend_->world().conditions();
        speedFactor = cond.speedFactor();
    }

    const float baseStepPixels = 2.0f;
    const float minGapPixels = 16.0f;

    QHash<int, QVector<int>> byRoad;
    for (int i = 0; i < cars_.size(); ++i) {
        const Car& c = cars_[i];
        if (c.finished) continue;
        if (c.roadIndex < 0 || c.roadIndex >= roads_.size()) continue;
        byRoad[c.roadIndex].push_back(i);
    }

    bool anyAlive = false;

    auto it = byRoad.begin();
    for (; it != byRoad.end(); ++it) {
        int roadIdx = it.key();
        Road& r = roads_[roadIdx];

        double len = roadLength(r);
        if (len <= 1e-3) {
            for (int idx : it.value()) {
                cars_[idx].finished = true;
            }
            continue;
        }

        auto& indices = it.value();

        std::sort(indices.begin(), indices.end(), [&](int a, int b) {
            return cars_[a].t > cars_[b].t;
        });

        Car* leader = nullptr;
        double leaderS = 0.0;

        for (int idxCar : indices) {
            Car& c = cars_[idxCar];
            if (c.finished) continue;

            double sNow = c.t * len;

            float dist = baseStepPixels * c.speed * speedFactor;

            QPointF posNow = roadPointAt(r, c.t);
            QPointF dir = roadDirectionAt(r, c.t);
            double dirLen = std::sqrt(dir.x() * dir.x() + dir.y() * dir.y());
            if (dirLen > 1e-3) {
                dir /= dirLen;
            } else {
                dir = QPointF(1, 0);
            }

            for (const Light& l : lights_) {
                QPointF v = l.pos - posNow;
                double proj = v.x() * dir.x() + v.y() * dir.y();
                if (proj < 0 || proj > 40.0) continue;

                double perp = std::abs(v.x() * dir.y() - v.y() * dir.x());
                if (perp > 12.0) continue;

                LightColor col = lightColorFor(l);
                if (col == LightColor::Red || col == LightColor::Yellow) {
                    dist = 0.0f;
                    break;
                }
            }

            double sNew = sNow + dist;

            if (leader) {
                double allowedCenter = leaderS - minGapPixels;
                if (allowedCenter < sNow) {
                    allowedCenter = sNow;
                }
                if (sNew > allowedCenter) {
                    sNew = allowedCenter;
                }
            }

            if (sNew > len) {
                sNew = len;
            }

            if (len > 1e-3) {
                c.t = static_cast<float>(sNew / len);
            }

            if (c.t >= 1.0f - 1e-4f) {
                c.finished = true;
            } else {
                anyAlive = true;
            }

            leader = &c;
            leaderS = sNew;
        }
    }

    if (!anyAlive) {
        simulationRunning_ = false;
        simTimer_.stop();
    }

    update();
}

// рисует фон билдер-режима в зависимости от времени и сезона
void BuildModeView::drawBackground(QPainter& p)
{
    Season season = Season::Summer;
    TimeOfDay tod = TimeOfDay::Day;

    if (backend_) {
        const auto& simTime = backend_->world().simulationTime();
        season = simTime.season();
        tod = simTime.timeOfDay();
    }

    QColor top;
    QColor bottom;

    switch (season) {
    case Season::Winter:
        if (tod == TimeOfDay::Night) {
            top = QColor(10, 15, 40);
            bottom = QColor(5, 10, 25);
        } else {
            top = QColor(160, 190, 220);
            bottom = QColor(210, 230, 245);
        }
        break;
    case Season::Spring:
        if (tod == TimeOfDay::Night) {
            top = QColor(20, 25, 55);
            bottom = QColor(10, 20, 40);
        } else {
            top = QColor(170, 215, 190);
            bottom = QColor(230, 245, 230);
        }
        break;
    case Season::Summer:
        if (tod == TimeOfDay::Night) {
            top = QColor(18, 18, 45);
            bottom = QColor(10, 10, 30);
        } else {
            top = QColor(135, 206, 235);
            bottom = QColor(245, 250, 255);
        }
        break;
    case Season::Autumn:
    default:
        if (tod == TimeOfDay::Night) {
            top = QColor(20, 18, 40);
            bottom = QColor(12, 10, 25);
        } else {
            top = QColor(240, 200, 150);
            bottom = QColor(255, 235, 210);
        }
        break;
    }

    QLinearGradient bg(rect().topLeft(), rect().bottomRight());
    bg.setColorAt(0.0, top);
    bg.setColorAt(1.0, bottom);
    p.fillRect(rect(), bg);

    drawGrid(p, tod);
    if (tod == TimeOfDay::Night) {
        drawNightSky(p, tod);
    }
    drawGarlandAndSnow(p, season, tod);
}

// рисует сетку на фоне
void BuildModeView::drawGrid(QPainter& p, TimeOfDay tod)
{
    QColor gridColor;
    if (tod == TimeOfDay::Night) {
        gridColor = QColor(40, 50, 70, 60);
    } else {
        gridColor = QColor(220, 225, 230, 80);
    }

    p.save();
    QPen pen(gridColor);
    pen.setWidthF(1.0);
    p.setPen(pen);

    const int step = 80;
    for (int x = 0; x < width(); x += step) {
        p.drawLine(x, 0, x, height());
    }
    for (int y = 0; y < height(); y += step) {
        p.drawLine(0, y, width(), y);
    }

    p.restore();
}

// рисует ночное небо 
void BuildModeView::drawNightSky(QPainter& p, TimeOfDay tod)
{
    Q_UNUSED(tod);
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

    int baseSize = qMin(width(), height()) / 8;
    if (baseSize < 40) baseSize = 40;

    QRectF moonRect(width() - baseSize - 40, 40, baseSize, baseSize);

    QRadialGradient g(moonRect.center(), baseSize * 0.6);
    g.setColorAt(0.0, QColor(255, 255, 245, 230));
    g.setColorAt(0.4, QColor(255, 255, 230, 180));
    g.setColorAt(1.0, QColor(0, 0, 0, 0));

    p.setBrush(g);
    p.setPen(Qt::NoPen);
    p.drawEllipse(moonRect);

    QRectF cutRect = moonRect.adjusted(baseSize * 0.25, 0, 0, 0);
    p.setBrush(QColor(10, 12, 25, 220));
    p.drawEllipse(cutRect);

    p.restore();
}

// рисует зимнюю гирлянду и снежинки 
void BuildModeView::drawGarlandAndSnow(QPainter& p, Season season, TimeOfDay tod)
{
    Q_UNUSED(tod);

    if (season != Season::Winter) {
        return;
    }

    p.save();
    p.setRenderHint(QPainter::Antialiasing, true);

    int w = width();
    int h = height();

    int marginX = 0;
    int garlandY = 20;

    QPen wirePen(QColor(80, 90, 110, 220));
    wirePen.setWidthF(2.0);
    p.setPen(wirePen);
    p.setBrush(Qt::NoBrush);

    QPainterPath wirePath;
    wirePath.moveTo(marginX, garlandY);
    int span = w - 2 * marginX;
    int humpCount = 4;
    for (int i = 1; i <= humpCount; ++i) {
        qreal x1 = marginX + span * (i - 0.5) / humpCount;
        qreal y1 = garlandY + 30;
        qreal x2 = marginX + span * i / humpCount;
        qreal y2 = garlandY;
        wirePath.quadTo(QPointF(x1, y1), QPointF(x2, y2));
    }
    p.drawPath(wirePath);

    QVector<QColor> bulbColors = {QColor("#FFC107"), QColor("#FF5252"), QColor("#4CAF50"), QColor("#40C4FF")};

    int bulbCount = 14;
    for (int i = 0; i < bulbCount; ++i) {
        qreal t = (i + 0.5) / bulbCount;
        QPointF onWire = wirePath.pointAtPercent(t);

        QColor c = bulbColors[i % bulbColors.size()];

        qreal radius = 7.0;
        QRectF bulbRect(onWire.x() - radius, onWire.y() - radius, 2 * radius, 2 * radius);

        QRadialGradient rg(bulbRect.center(), radius);
        rg.setColorAt(0.0, QColor(255, 255, 255, 230));
        rg.setColorAt(0.4, c.lighter(110));
        rg.setColorAt(1.0, c.darker(115));

        p.setPen(QPen(QColor(30, 30, 40, 200), 1.0));
        p.setBrush(rg);
        p.drawEllipse(bulbRect);

        QRectF capRect(bulbRect.center().x() - 3.0, bulbRect.center().y() - radius - 2.0, 6.0, 4.0);
        p.setBrush(QColor(80, 80, 90, 220));
        p.drawRoundedRect(capRect, 2.0, 2.0);

        p.setPen(QPen(QColor(150, 160, 180, 220), 1.0));
        p.drawLine(onWire, QPointF(bulbRect.center().x(), bulbRect.center().y() - radius));
    }

    auto drawSnowflake = [&](QPointF center, float size) {
        p.setPen(QPen(QColor(255, 255, 255, 200), 1.3));
        p.drawLine(QPointF(center.x() - size, center.y()), QPointF(center.x() + size, center.y()));
        p.drawLine(QPointF(center.x(), center.y() - size), QPointF(center.x(), center.y() + size));

        float d = size * 0.7f;
        p.drawLine(QPointF(center.x() - d, center.y() - d), QPointF(center.x() + d, center.y() + d));
        p.drawLine(QPointF(center.x() - d, center.y() + d), QPointF(center.x() + d, center.y() - d));
    };

    float flakeSize = qMax(8, w / 80);
    drawSnowflake(QPointF(marginX + 20, garlandY - 25), flakeSize);
    drawSnowflake(QPointF(w - marginX - 30, garlandY - 18), flakeSize * 0.9f);
    drawSnowflake(QPointF(marginX + 10, h / 3), flakeSize * 0.8f);
    drawSnowflake(QPointF(w - marginX - 10, h / 2), flakeSize * 0.85f);

    p.restore();
}

// рисует сцену
void BuildModeView::drawScene(QPainter& p)
{
    p.save();
    p.setClipRect(canvasRect());

    drawRoads(p);
    drawHouses(p);
    drawLights(p);
    drawCars(p);

    if (drawingRoad_ &&
        (currentTool_ == Tool::Road || currentTool_ == Tool::StraightRoad) &&
        currentRoad_.size() >= 2) {
        QPen pen(QColor(80, 80, 90, 200));
        pen.setWidthF(12.0);
        pen.setCapStyle(Qt::RoundCap);
        pen.setJoinStyle(Qt::RoundJoin);
        p.setPen(pen);
        p.setBrush(Qt::NoBrush);

        QPainterPath path(currentRoad_.first());
        for (int i = 1; i < currentRoad_.size(); ++i) {
            path.lineTo(currentRoad_[i]);
        }
        p.drawPath(path);
    }

    p.restore();
}

// рисует дороги 
void BuildModeView::drawRoads(QPainter& p)
{
    if (roads_.isEmpty())
        return;

    p.save();
    p.setRenderHint(QPainter::Antialiasing, true);

    const qreal baseWidth = 11.0;
    const qreal haloWidth = baseWidth + 4.0;

    QPen haloPen(QColor(170, 180, 195, 150), haloWidth, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);

    QPen roadPen(QColor(230, 234, 244), baseWidth, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);

    int limit = baseRoadCount_ > 0 && baseRoadCount_ <= roads_.size() ? baseRoadCount_ : roads_.size();

    for (int i = 0; i < limit; ++i) {
        const Road& r = roads_[i];
        if (r.points.size() < 2)
            continue;

        QPainterPath path(r.points.front());
        for (int j = 1; j < r.points.size(); ++j) {
            path.lineTo(r.points[j]);
        }

        p.setPen(haloPen);
        p.setBrush(Qt::NoBrush);
        p.drawPath(path);

        p.setPen(roadPen);
        p.drawPath(path);
    }

    p.restore();
}

// рисует домики
void BuildModeView::drawHouses(QPainter& p)
{
    if (houses_.isEmpty())
        return;

    p.save();
    p.setRenderHint(QPainter::Antialiasing, true);

    const qreal size = 26.0;

    for (const House& h : houses_) {
        QPointF pt = h.pos;
        QRectF r(pt.x() - size * 0.5, pt.y() - size * 0.5, size, size);

        QColor base = h.color;
        QColor outline(255, 255, 255, 230);

        QBrush shadowBrush(QColor(0, 0, 0, 40));
        p.setBrush(shadowBrush);
        p.setPen(Qt::NoPen);
        QRectF shadowRect = r.adjusted(1.5, 2.0, 1.5, 4.0);
        p.drawRoundedRect(shadowRect, size * 0.35, size * 0.35);

        QPen pen(outline);
        pen.setWidthF(2.0);
        p.setPen(pen);
        p.setBrush(base);
        p.drawRoundedRect(r, size * 0.35, size * 0.35);

        qreal doorW = size * 0.25;
        qreal doorH = size * 0.38;
        QRectF doorRect(pt.x() - doorW * 0.5, pt.y() + size * 0.05, doorW, doorH);
        p.setBrush(QColor(20, 24, 35, 220));
        p.setPen(Qt::NoPen);
        p.drawRoundedRect(doorRect, doorW * 0.3, doorW * 0.3);
    }

    p.restore();
}

// рисует светофоры
void BuildModeView::drawLights(QPainter& p)
{
    for (const Light& l : lights_) {
        QPointF center = l.pos;

        const qreal w = 14;
        const qreal h = 34;
        QRectF body(center.x() - w / 2, center.y() - h / 2, w, h);

        p.save();
        p.setRenderHint(QPainter::Antialiasing, true);

        p.setBrush(QColor(0, 0, 0, 200));
        p.setPen(Qt::NoPen);
        p.drawRoundedRect(body, 4, 4);

        const qreal r = 3.5;
        QPointF cRed(center.x(), center.y() - 8);
        QPointF cYellow(center.x(), center.y());
        QPointF cGreen(center.x(), center.y() + 8);

        auto drawLamp = [&](const QPointF& c, const QColor& col, bool on) {
            QColor c2 = col;
            if (!on) c2.setAlpha(60);
            p.setBrush(c2);
            p.drawEllipse(QRectF(c.x() - r, c.y() - r, 2 * r, 2 * r));
        };

        LightColor color = lightColorFor(l);

        drawLamp(cRed, QColor("#FF5252"), color == LightColor::Red);
        drawLamp(cYellow, QColor("#FFEB3B"), color == LightColor::Yellow);
        drawLamp(cGreen, QColor("#69F0AE"), color == LightColor::Green);

        p.restore();
    }
}

// рисует машинки вдоль дорог.
void BuildModeView::drawCars(QPainter& p)
{
    for (const Car& c : cars_) {
        if (c.finished || c.roadIndex < 0 || c.roadIndex >= roads_.size()) {
            continue;
        }
        const Road& r = roads_[c.roadIndex];

        QPointF pos = roadPointAt(r, c.t);

        float t0 = qMax(0.0f, c.t - 0.01f);
        QPointF posBack = roadPointAt(r, t0);

        QPointF v = pos - posBack;
        double angleDeg = 0.0;
        if (!qFuzzyIsNull(v.x()) || !qFuzzyIsNull(v.y())) {
            angleDeg = std::atan2(v.y(), v.x()) * 180.0 / M_PI;
        }

        float len = 12.0f;
        float wid = 7.0f;

        p.save();
        p.setRenderHint(QPainter::Antialiasing, true);

        p.translate(pos);
        p.rotate(angleDeg);

        QRectF body(-len / 2, -wid / 2, len, wid);
        p.setBrush(c.color);
        p.setPen(Qt::NoPen);
        p.drawRoundedRect(body, 3.0, 3.0);

        p.setBrush(QColor(255, 255, 255, 220));
        p.drawEllipse(QRectF(body.right() - 3.5, body.top() + 1.0, 2.0, 2.0));
        p.drawEllipse(QRectF(body.right() - 3.5, body.bottom() - 3.0, 2.0, 2.0));

        p.restore();
    }
}

// возвращает прямоугольник кнопки “Домой”
QRectF BuildModeView::homeButtonRect() const
{
    QRectF sp = sidePanelRect();
    return QRectF(sp.left() + 16, sp.top() + 16, sp.width() - 32, 32);
}

// возвращает прямоугольник кнопки “Настройки”
QRectF BuildModeView::settingsButtonRect() const
{
    QRectF r = homeButtonRect();
    r.translate(0, 40);
    return r;
}

// возвращает прямоугольник кнопки инструмента по индексу
QRectF BuildModeView::toolButtonRect(int index) const
{
    QRectF sp = sidePanelRect();
    QRectF base(sp.left() + 16, sp.top() + 120, sp.width() - 32, 32);
    base.translate(0, index * 36);
    return base;
}

// возвращает прямоугольник кнопки “Старт”
QRectF BuildModeView::startButtonRect() const
{
    QRectF sp = sidePanelRect();
    return QRectF(sp.left() + 16, sp.bottom() - 110, sp.width() - 32, 32);
}

// возвращает прямоугольник кнопки “Стоп”
QRectF BuildModeView::stopButtonRect() const
{
    QRectF r = startButtonRect();
    r.translate(0, 40);
    return r;
}

// возвращает прямоугольник области машин
QRectF BuildModeView::carsSliderRect() const
{
    QRectF sp = sidePanelRect();
    return QRectF(sp.left() + 24, sp.bottom() - 180, sp.width() - 48, 20);
}

// рисует панель управления
void BuildModeView::drawSidePanel(QPainter& p)
{
    QRectF sp = sidePanelRect();

    p.save();
    QPainterPath path;
    path.addRoundedRect(sp.adjusted(4, 8, -4, -8), 16, 16);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(15, 18, 30, 230));
    p.drawPath(path);

    p.setPen(QColor(230, 230, 245));
    QFont f = font();
    f.setPointSizeF(f.pointSizeF() + 1);
    f.setBold(true);
    p.setFont(f);
    p.drawText(QRectF(sp.left() + 16, sp.top() + 12, sp.width() - 32, 24),
               Qt::AlignLeft | Qt::AlignVCenter,
               tr(""));

    auto drawButton = [&](const QRectF& r, const QString& text, bool active = false) {
        p.save();
        QPainterPath bp;
        bp.addRoundedRect(r, 10, 10);
        QColor bg = active ? QColor(80, 120, 255, 220) : QColor(40, 50, 80, 210);
        p.setBrush(bg);
        p.setPen(QPen(QColor(10, 12, 25, 255), 1.0));
        p.drawPath(bp);

        p.setPen(Qt::white);
        p.drawText(r.adjusted(8, 0, -8, 0), Qt::AlignLeft | Qt::AlignVCenter, text);
        p.restore();
    };

    drawButton(homeButtonRect(), tr("На главный экран"));
    drawButton(settingsButtonRect(), tr("Настройки времени"));

    QString toolNames[6] = {
        tr("Домики (парами)"),
        tr("Дороги"),
        tr("Прямая дорога"),
        tr("Светофоры"),
        tr("Кольцо"),
        tr("Ластик")
    };

    for (int i = 0; i < 6; ++i) {
        bool active =
            (i == 0 && currentTool_ == Tool::House) ||
            (i == 1 && currentTool_ == Tool::Road) ||
            (i == 2 && currentTool_ == Tool::StraightRoad) ||
            (i == 3 && currentTool_ == Tool::Light) ||
            (i == 4 && currentTool_ == Tool::Roundabout) ||
            (i == 5 && currentTool_ == Tool::Erase);

        drawButton(toolButtonRect(i), toolNames[i], active);
    }

    QRectF sr = carsSliderRect();
    p.setPen(QColor(180, 180, 200));
    p.drawText(QRectF(sr.left(), sr.top() - 18, sr.width(), 16),
               Qt::AlignLeft | Qt::AlignVCenter,
               tr("Машин в паре: %1").arg(carsPerPair_));

    p.setPen(QPen(QColor(80, 90, 120), 3.0));
    p.drawLine(QPointF(sr.left(), sr.center().y()), QPointF(sr.right(), sr.center().y()));

    qreal t = (carsPerPair_ - 1) / 29.0;
    t = qBound(0.0, t, 1.0);
    qreal x = sr.left() + t * sr.width();
    QRectF knob(x - 6, sr.center().y() - 6, 12, 12);
    p.setBrush(QColor(120, 170, 255));
    p.setPen(Qt::NoPen);
    p.drawEllipse(knob);

    drawButton(startButtonRect(),
               simulationRunning_ ? tr("Симуляция идёт") : tr("Запустить симуляцию"),
               simulationRunning_);

    drawButton(stopButtonRect(), tr("Остановить и сбросить"));

    p.restore();
}

// возвращает текущий цвет светофора по внутреннему времени и группе
traffic::LightColor BuildModeView::lightColorFor(const Light& l) const
{
    const float greenDuration = 4.0f;
    const float yellowDuration = 1.0f;
    const float allRedDuration = 8.0f;

    const int phaseCount = 4;
    const float phaseDuration = greenDuration + yellowDuration + allRedDuration;
    const float cycleDuration = phaseDuration * phaseCount;

    if (phaseDuration <= 0.0f || cycleDuration <= 0.0f) {
        return LightColor::Green;
    }

    int group = l.groupId % phaseCount;
    if (group < 0) group += phaseCount;

    float t = std::fmod(simSeconds_ + l.phaseOffset, cycleDuration);
    if (t < 0.0f) {
        t += cycleDuration;
    }

    float base = t - group * phaseDuration;
    float local = std::fmod(base, phaseDuration);
    if (local < 0.0f) {
        local += phaseDuration;
    }

    if (local < greenDuration) {
        return LightColor::Green;
    }
    if (local < greenDuration + yellowDuration) {
        return LightColor::Yellow;
    }
    return LightColor::Red;
}

// активирует инструмент добавления кольца
void BuildModeView::onToolRoundabout()
{
    currentTool_ = Tool::Roundabout;
    update();
}

// добавляет кольцо в указанной точке.
void BuildModeView::addRoundaboutAt(const QPointF& center)
{
    const int segments = 20;
    const qreal radius = 40.0;

    Road r;
    r.points.reserve(segments + 1);
    for (int i = 0; i <= segments; ++i) {
        qreal ang = (2.0 * M_PI * i) / segments;
        qreal x = center.x() + radius * std::cos(ang);
        qreal y = center.y() + radius * std::sin(ang);
        r.points.push_back(QPointF(x, y));
    }

    roads_.push_back(r);
    baseRoadCount_ = roads_.size();

    update();
}

// возвращает направление дороги в точке t
QPointF BuildModeView::roadDirectionAt(const Road& r, float t) const
{
    if (r.points.size() < 2) {
        return QPointF(1.0, 0.0);
    }

    QVector<double> segLen;
    segLen.reserve(r.points.size() - 1);
    double total = 0.0;
    for (int i = 1; i < r.points.size(); ++i) {
        double len = QLineF(r.points[i - 1], r.points[i]).length();
        segLen.push_back(len);
        total += len;
    }
    if (total <= 1e-3) {
        return QPointF(1.0, 0.0);
    }

    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    double target = t * total;
    double accum = 0.0;
    for (int i = 0; i < segLen.size(); ++i) {
        double seg = segLen[i];
        if (accum + seg >= target) {
            QPointF a = r.points[i];
            QPointF b = r.points[i + 1];
            QPointF dir = b - a;
            return dir;
        }
        accum += seg;
    }

    QPointF a = r.points[r.points.size() - 2];
    QPointF b = r.points[r.points.size() - 1];
    return b - a;
}
} 