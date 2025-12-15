#pragma once
#include <QObject>
#include <QCoreApplication>

class WebBridge : public QObject
{
    Q_OBJECT
public:
    WebBridge(QObject* parent) : QObject(parent = nullptr){}

public slots:
    Q_INVOKABLE void requestExit();
    Q_INVOKABLE void requestStart();
    Q_INVOKABLE void requestSettings();
    Q_INVOKABLE void requestBuildMode();   

signals:
    void startRequested();
    void settingsRequested();
    void buildModeRequested();  
};