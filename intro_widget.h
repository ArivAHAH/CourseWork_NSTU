#pragma once

#include "webbridge.h"

#include <QWebEngineView>
#include <QVBoxLayout>
#include <QUrl>
#include <QWidget>
#include <QWebChannel>

class QWebEngineView;
class WebBridge;

class IntroWidget : public QWidget
{
    Q_OBJECT

public:
    explicit IntroWidget(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        view_ = new QWebEngineView(this);
        bridge_ = new WebBridge(this);
        channel_ = new QWebChannel(this);

        channel_->registerObject(QStringLiteral("bridge"), bridge_);
        view_->page()->setWebChannel(channel_);

        view_->load(QUrl("qrc:/ui/start_page.html"));

        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->addWidget(view_);
        setLayout(layout);

        connect(bridge_, &WebBridge::startRequested, this, &IntroWidget::startRequested);
        connect(bridge_, &WebBridge::settingsRequested, this, &IntroWidget::settingsRequested);
        connect(bridge_, &WebBridge::buildModeRequested, this, &IntroWidget::buildModeRequested);
    }

signals:
    void startRequested();
    void settingsRequested();
    void buildModeRequested();

protected:
    bool eventFilter(QObject* obj, QEvent* event) override;

private:
    QWebEngineView* view_ = nullptr;
    QWebChannel* channel_ = nullptr;
    WebBridge* bridge_ = nullptr;
};
