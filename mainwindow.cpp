#include "mainwindow.h"

using app::SimulationBackend;

// cоздаёт центральный layout и показывает стартовый экран
void MainWindow::setupUi()
{
    auto* central = new QWidget(this);
    setCentralWidget(central);

    auto* mainLayout = new QVBoxLayout(central);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(0);

    introWidget_ = new IntroWidget(this);
    introWidget_->setObjectName("IntroCanvas");

    mainLayout->addWidget(introWidget_, 1);
}

// применяет глобальный стиль окна и стартового экрана
void MainWindow::applyStyle()
{
    setStyleSheet(R"(
        QMainWindow {
            background-color: #121212;
            color: #f5f5f5;
        }

        #IntroCanvas {
            background-color: transparent;
        }
    )");
}

// подключает сигналы от стартового экрана к обработчикам окна
void MainWindow::setupConnections()
{
    if (!introWidget_) {
        return;
    }

    connect(introWidget_, &IntroWidget::startRequested,
            this, &MainWindow::handleStartRequested);

    connect(introWidget_, &IntroWidget::settingsRequested,
            this, &MainWindow::openSettingsDialog);

    connect(introWidget_, &IntroWidget::buildModeRequested,
            this, &MainWindow::handleBuildModeRequested);
}

// запускает симуляцию и переключает со стартового экрана на симуляцию
void MainWindow::handleStartRequested()
{
    if (!backend_) {
        QMessageBox::warning(this, tr("Симуляция"), tr("Бэкенд не инициализирован."));
        return;
    }

    if (simulationStarted_) {
        backend_->setPaused(false);
        return;
    }

    simulationStarted_ = true;
    backend_->setPaused(false);

    if (!simulationView_) {
        simulationView_ = new app::SimulationView(backend_, this);

        connect(simulationView_, &app::SimulationView::homeRequested,
                this, &MainWindow::handleSimulationHomeRequested);

        connect(simulationView_, &app::SimulationView::settingsRequested,
                this, &MainWindow::openSettingsDialog);
    }

    auto* layout = qobject_cast<QVBoxLayout*>(centralWidget()->layout());
    if (!layout) {
        return;
    }

    auto* effectOut = new QGraphicsOpacityEffect(introWidget_);
    introWidget_->setGraphicsEffect(effectOut);

    auto* fadeOut = new QPropertyAnimation(effectOut, "opacity", this);
    fadeOut->setDuration(600);
    fadeOut->setStartValue(1.0);
    fadeOut->setEndValue(0.0);
    fadeOut->setEasingCurve(QEasingCurve::InOutQuad);

    connect(fadeOut, &QPropertyAnimation::finished, this, [this, layout]() {
        if (introWidget_) {
            introWidget_->hide();
            layout->removeWidget(introWidget_);
            introWidget_->deleteLater();
            introWidget_ = nullptr;
        }

        if (!simulationView_->parent()) {
            simulationView_->setParent(centralWidget());
        }

        auto* effectIn = new QGraphicsOpacityEffect(simulationView_);
        simulationView_->setGraphicsEffect(effectIn);
        effectIn->setOpacity(0.0);

        layout->addWidget(simulationView_, 1);
        simulationView_->show();

        auto* fadeIn = new QPropertyAnimation(effectIn, "opacity", this);
        fadeIn->setDuration(600);
        fadeIn->setStartValue(0.0);
        fadeIn->setEndValue(1.0);
        fadeIn->setEasingCurve(QEasingCurve::InOutQuad);
        fadeIn->start(QAbstractAnimation::DeleteWhenStopped);
    });

    fadeOut->start(QAbstractAnimation::DeleteWhenStopped);
}

// создаёт и инициализирует бэк.
void MainWindow::initBackend()
{
    backend_ = new SimulationBackend(this);

    const bool ok = backend_->init();
    if (!ok) {
        QMessageBox::warning(
            this,
            tr("Ошибка БД"),
            tr("Не удалось инициализировать бэкенд.\n"
               "Проверьте наличие файла traffic.db и структуру таблиц.")
        );
    }
}

// создаёт диалог настроек и подключает применение параметров к бэкенду
void MainWindow::createSettingsDialog()
{
    settingsDialog_ = new QDialog(this);
    settingsDialog_->setWindowTitle(tr("Настройки симуляции"));
    settingsDialog_->setModal(true);
    settingsDialog_->resize(520, 680);
    settingsDialog_->setStyleSheet(R"(
        QDialog {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                stop:0 #0b0f20, stop:1 #071028);
            border-radius: 28px;
            border: 1px solid rgba(79, 140, 255, 0.3);
        }
    )");

    auto* container = new QWidget(settingsDialog_);
    container->setStyleSheet(R"(
        QWidget {
            background: rgba(11, 15, 32, 0.95);
            border-radius: 24px;
            border: 1px solid rgba(79, 140, 255, 0.2);
        }

        QLabel, QCheckBox {
            background: transparent;
            border: none;
            color: #f5f5f5;
            font-size: 16px;
        }

        QLabel#header {
            font-size: 28px;
            font-weight: bold;
            color: #4f8cff;
            background: transparent;
        }

        QDateTimeEdit {
            padding: 16px 48px 16px 16px;
            border-radius: 16px;
            background: rgba(255, 255, 255, 0.08);
            border: 1px solid rgba(79, 140, 255, 0.4);
            color: white;
            font-size: 15px;
            selection-background-color: #4f8cff;
        }

        QDateTimeEdit::drop-down {
            subcontrol-origin: padding;
            subcontrol-position: center right;
            width: 48px;
            border-left: 1px solid rgba(79, 140, 255, 0.4);
            border-top-right-radius: 16px;
            border-bottom-right-radius: 16px;
            background: rgba(79, 140, 255, 0.15);
        }

        QDateTimeEdit::down-arrow {
            width: 10px;
            height: 10px;
            border-radius: 5px;
            background: #4f8cff;
        }

        QDateTimeEdit::down-arrow:hover {
            background: #5f9dff;
        }

        QDateTimeEdit::down-arrow:pressed {
            background: #3d70cc;
        }

        QSlider::groove:horizontal {
            height: 12px;
            border-radius: 6px;
            background: rgba(79, 140, 255, 0.25);
        }

        QSlider::handle:horizontal {
            width: 32px;
            height: 32px;
            margin: -10px 0;
            border-radius: 16px;
            background: qradialgradient(cx:0.5, cy:0.5, radius:0.8,
                stop:0 #0affc2, stop:1 #4f8cff);
            border: 3px solid white;
        }

        QCheckBox {
            spacing: 16px;
            font-size: 16px;
            color: #f5f5f5;
        }

        QCheckBox::indicator {
            width: 26px;
            height: 26px;
            border-radius: 10px;
            border: 2px solid #4f8cff;
            background: transparent;
        }

        QCheckBox::indicator:checked {
            background: #4f8cff;
            image: url(qrc:/ui/assets/check.png);
        }

        QPushButton {
            padding: 16px 32px;
            border-radius: 16px;
            font-size: 16px;
            font-weight: bold;
            border: none;
        }

        QPushButton#ok {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                stop:0 #4f8cff, stop:1 #0affc2);
            color: white;
        }

        QPushButton#ok:hover {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                stop:0 #5f9dff, stop:1 #1affd4);
        }

        QPushButton#ok:pressed {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                stop:0 #3d70cc, stop:1 #00cc99);
        }

        QPushButton#cancel {
            background: transparent;
            color: #b4b7c8;
            border: 1px solid rgba(180, 183, 200, 0.3);
        }

        QPushButton#cancel:hover {
            background: rgba(180, 183, 200, 0.1);
        }
    )");

    auto* layout = new QVBoxLayout(container);
    layout->setContentsMargins(40, 40, 40, 40);
    layout->setSpacing(24);

    auto* header = new QLabel(tr("Настройки"));
    header->setObjectName("header");
    header->setAlignment(Qt::AlignCenter);
    layout->addWidget(header);

    {
        auto* title = new QLabel(tr("Дата и время симуляции"));
        layout->addWidget(title);

        timeEdit_ = new QDateTimeEdit;
        timeEdit_->setDisplayFormat("dd.MM.yyyy HH:mm");
        timeEdit_->setCalendarPopup(true);

        if (backend_) {
            const auto cond = backend_->world().conditions();
            QDate d(cond.time.year, static_cast<int>(cond.time.month), cond.time.day);
            QTime tm(cond.time.hour, cond.time.minute);
            timeEdit_->setDateTime(QDateTime(d, tm));
        } else {
            timeEdit_->setDateTime(QDateTime::currentDateTime());
        }

        layout->addWidget(timeEdit_);
    }

    {
        auto* title = new QLabel(tr("Целевое число машин"));
        layout->addWidget(title);

        auto* sliderRow = new QHBoxLayout;

        vehiclesSlider_ = new QSlider(Qt::Horizontal);
        vehiclesSlider_->setRange(0, 3000);
        vehiclesSlider_->setValue(0);

        vehiclesValueLabel_ = new QLabel(tr("Авто"));
        vehiclesValueLabel_->setStyleSheet("color: #0affc2; font-weight: bold; min-width: 80px;");

        sliderRow->addWidget(vehiclesSlider_);
        sliderRow->addWidget(vehiclesValueLabel_);
        layout->addLayout(sliderRow);

        connect(vehiclesSlider_, &QSlider::valueChanged, this, [this](int v) {
            vehiclesValueLabel_->setText(v == 0 ? tr("Авто") : QString::number(v));
        });
    }

    {
        auto* title = new QLabel(tr("Дополнительные условия"));
        layout->addWidget(title);

        accidentCheck_ = new QCheckBox(tr("Аварии"), container);
        roadWorkCheck_ = new QCheckBox(tr("Дорожные работы"), container);
        iceCheck_ = new QCheckBox(tr("Гололёд"), container);
        lightsCheck_ = new QCheckBox(tr("Светофоры"), container);

        accidentCheck_->setChecked(false);
        roadWorkCheck_->setChecked(false);
        iceCheck_->setChecked(false);
        lightsCheck_->setChecked(true);

        if (backend_) {
            const auto cond = backend_->world().conditions();
            accidentCheck_->setChecked(cond.accident);
            roadWorkCheck_->setChecked(cond.roadWork);
            iceCheck_->setChecked(cond.ice);
            lightsCheck_->setChecked(backend_->world().lightsEnabled());
        }

        layout->addWidget(accidentCheck_);
        layout->addWidget(roadWorkCheck_);
        layout->addWidget(iceCheck_);
        layout->addWidget(lightsCheck_);
    }

    layout->addStretch();

    auto* buttons = new QHBoxLayout;
    buttons->setSpacing(16);

    auto* okBtn = new QPushButton(tr("Применить"));
    okBtn->setObjectName("ok");
    okBtn->setCursor(Qt::PointingHandCursor);

    auto* cancelBtn = new QPushButton(tr("Отмена"));
    cancelBtn->setObjectName("cancel");
    cancelBtn->setCursor(Qt::PointingHandCursor);

    buttons->addWidget(okBtn);
    buttons->addWidget(cancelBtn);
    layout->addLayout(buttons);

    connect(okBtn, &QPushButton::clicked, settingsDialog_, &QDialog::accept);
    connect(cancelBtn, &QPushButton::clicked, settingsDialog_, &QDialog::reject);

    connect(settingsDialog_, &QDialog::accepted, this, [this]() {
        if (!backend_) {
            return;
        }

        const QDateTime dt = timeEdit_->dateTime();
        backend_->setSimulationDateTime(dt.date().year(),
                                        dt.date().month(),
                                        dt.date().day(),
                                        dt.time().hour(),
                                        dt.time().minute());

        backend_->setAccidentEnabled(accidentCheck_->isChecked());
        backend_->setRoadWorkEnabled(roadWorkCheck_->isChecked());
        backend_->setIceEnabled(iceCheck_->isChecked());
        backend_->setLightsEnabled(lightsCheck_->isChecked());

        backend_->setManualVehicleTarget(vehiclesSlider_->value());
    });

    auto* mainLayout = new QVBoxLayout(settingsDialog_);
    mainLayout->setContentsMargins(30, 30, 30, 30);
    mainLayout->addWidget(container, 0, Qt::AlignCenter);
}

// открывает диалог настроек и перед показом подтягивает актуальное время
void MainWindow::openSettingsDialog()
{
    if (!settingsDialog_) {
        createSettingsDialog();
    }

    if (backend_ && timeEdit_) {
        const auto cond = backend_->world().conditions();
        const auto& t = cond.time;

        QDate d(t.year, static_cast<int>(t.month), t.day);
        QTime tm(t.hour, t.minute);

        timeEdit_->setDateTime(QDateTime(d, tm));
    }

    settingsDialog_->exec();
}

// открывает диалог настроек времени для режима строительства и применяет новое время к миру
void MainWindow::openBuildModeSettingsDialog()
{
    if (!backend_) {
        return;
    }

    const auto& world = backend_->world();
    const auto& cond = world.conditions();
    const auto& t = cond.time;

    QDate date(t.year, static_cast<int>(t.month), t.day);
    QTime time(t.hour, t.minute);

    QDialog dlg(this);
    dlg.setWindowTitle(tr("Настройки времени"));

    auto* layout = new QVBoxLayout(&dlg);

    auto* dtLabel = new QLabel(tr("Текущее время в городе:"), &dlg);
    layout->addWidget(dtLabel);

    auto* dtEdit = new QDateTimeEdit(&dlg);
    dtEdit->setDisplayFormat("dd.MM.yyyy HH:mm");
    dtEdit->setDate(date);
    dtEdit->setTime(time);
    dtEdit->setCalendarPopup(true);
    layout->addWidget(dtEdit);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                                     Qt::Horizontal, &dlg);
    layout->addWidget(btns);

    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    if (dlg.exec() != QDialog::Accepted) {
        return;
    }

    QDateTime newDt = dtEdit->dateTime();

    traffic::SimulationTime newTime = t;
    newTime.year = newDt.date().year();
    newTime.month = static_cast<traffic::Month>(newDt.date().month());
    newTime.day = newDt.date().day();
    newTime.hour = newDt.time().hour();
    newTime.minute = newDt.time().minute();

    traffic::TrafficConditions newCond = cond;
    newCond.time = newTime;

    auto& mutableWorld = backend_->world();
    mutableWorld.setConditions(newCond);

    if (simulationView_) {
        simulationView_->update();
    }
    if (buildModeView_) {
        buildModeView_->update();
    }
}

// обрабатывает возврат “Домой” из симуляции и показывает стартовый экран
void MainWindow::handleSimulationHomeRequested()
{
    if (!backend_) {
        return;
    }

    backend_->setPaused(true);
    simulationStarted_ = false;

    auto* layout = qobject_cast<QVBoxLayout*>(centralWidget()->layout());
    if (!layout) {
        return;
    }

    if (simulationView_) {
        simulationView_->hide();
        layout->removeWidget(simulationView_);
    }

    if (!introWidget_) {
        introWidget_ = new IntroWidget(this);

        connect(introWidget_, &IntroWidget::startRequested,
                this, &MainWindow::handleStartRequested);
        connect(introWidget_, &IntroWidget::settingsRequested,
                this, &MainWindow::openSettingsDialog);
        connect(introWidget_, &IntroWidget::buildModeRequested,
                this, &MainWindow::handleBuildModeRequested);
    }

    layout->addWidget(introWidget_, 1);
    introWidget_->show();
}

// переключает приложение в режим строительства
void MainWindow::handleBuildModeRequested()
{
    auto* layout = qobject_cast<QVBoxLayout*>(centralWidget()->layout());
    if (!layout) {
        return;
    }

    if (backend_) {
        backend_->setPaused(true);
        simulationStarted_ = false;
    }

    if (introWidget_) {
        introWidget_->hide();
        layout->removeWidget(introWidget_);
    }
    if (simulationView_) {
        simulationView_->hide();
        layout->removeWidget(simulationView_);
    }

    if (!buildModeView_) {
        buildModeView_ = new app::BuildModeView(backend_, this);

        connect(buildModeView_, &app::BuildModeView::homeRequested,
                this, [this]() {
                    auto* l = qobject_cast<QVBoxLayout*>(centralWidget()->layout());
                    if (!l) {
                        return;
                    }

                    if (buildModeView_) {
                        buildModeView_->hide();
                        l->removeWidget(buildModeView_);
                    }

                    if (!introWidget_) {
                        introWidget_ = new IntroWidget(this);
                        setupConnections();
                    }

                    l->addWidget(introWidget_, 1);
                    introWidget_->show();
                });

        connect(buildModeView_, &app::BuildModeView::settingsRequested,
                this, &MainWindow::openBuildModeSettingsDialog);
    }

    layout->addWidget(buildModeView_, 1);
    buildModeView_->show();
}
