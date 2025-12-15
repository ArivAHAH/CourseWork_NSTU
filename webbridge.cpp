#include "webbridge.h"


//обработки запросов
void WebBridge::requestExit()
{
    qApp->quit();
}

void WebBridge::requestStart()
{
    emit startRequested();
}

void WebBridge::requestSettings()
{
    emit settingsRequested();
}

void WebBridge::requestBuildMode()
{
    emit buildModeRequested();
}