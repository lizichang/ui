#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDebug>
#include <QTimer>
#include <QLabel>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    // 添加状态栏标签
    batteryLabel = new QLabel("电池: 100%");
    signalLabel = new QLabel("信号: 良好");
    gpsLabel = new QLabel("GPS: 定位中");
    modeLabel = new QLabel("模式: 手动");
    
    ui->statusbar->addPermanentWidget(batteryLabel);
    ui->statusbar->addPermanentWidget(signalLabel);
    ui->statusbar->addPermanentWidget(gpsLabel);
    ui->statusbar->addPermanentWidget(modeLabel);
    
    // 连接信号槽
    connect(ui->video1SourceComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onVideo1SourceChanged);
    connect(ui->video2SourceComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onVideo2SourceChanged);
    
    // 连接按钮信号
    connect(ui->startButton, &QPushButton::clicked, this, &MainWindow::onStartButtonClicked);
    connect(ui->stopButton, &QPushButton::clicked, this, &MainWindow::onStopButtonClicked);
    connect(ui->emergencyButton, &QPushButton::clicked, this, &MainWindow::onEmergencyButtonClicked);
    connect(ui->calibrateButton, &QPushButton::clicked, this, &MainWindow::onCalibrateButtonClicked);
    connect(ui->settingsButton, &QPushButton::clicked, this, &MainWindow::onSettingsButtonClicked);
    connect(ui->connectButton, &QPushButton::clicked, this, &MainWindow::onConnectButtonClicked);
    
    // 模拟数据更新
    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::updateStatusBar);
    timer->start(1000); // 每秒更新一次
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onVideo1SourceChanged(int index)
{
    QString source = ui->video1SourceComboBox->currentText();
    qDebug() << "图传1源切换为:" << source;
    ui->video1Label->setText("图传1: " + source);
}

void MainWindow::onVideo2SourceChanged(int index)
{
    QString source = ui->video2SourceComboBox->currentText();
    qDebug() << "图传2源切换为:" << source;
    ui->video2Label->setText("图传2: " + source);
}

void MainWindow::onStartButtonClicked()
{
    qDebug() << "启动按钮点击";
    ui->statusbar->showMessage("机器人启动中...", 2000);
}

void MainWindow::onStopButtonClicked()
{
    qDebug() << "停止按钮点击";
    ui->statusbar->showMessage("机器人停止中...", 2000);
}

void MainWindow::onEmergencyButtonClicked()
{
    qDebug() << "紧急停止按钮点击";
    ui->statusbar->showMessage("紧急停止激活", 2000);
}

void MainWindow::onCalibrateButtonClicked()
{
    qDebug() << "校准按钮点击";
    ui->statusbar->showMessage("开始校准...", 2000);
}

void MainWindow::onSettingsButtonClicked()
{
    qDebug() << "设置按钮点击";
    ui->statusbar->showMessage("打开设置界面", 2000);
}

void MainWindow::onConnectButtonClicked()
{
    qDebug() << "连接按钮点击";
    ui->statusbar->showMessage("尝试连接机器人...", 2000);
}

void MainWindow::updateStatusBar()
{
    // 模拟数据更新
    static int battery = 100;
    static int signalStrength = 95;
    static bool gpsFixed = false;
    static QString modes[] = {"手动", "自动", "半自动"};
    static int modeIndex = 0;
    
    battery--;
    if (battery < 0) battery = 100;
    
    signalStrength = (signalStrength + 1) % 100;
    if (signalStrength < 30) signalStrength = 80;
    
    if (battery % 10 == 0) {
        gpsFixed = !gpsFixed;
        modeIndex = (modeIndex + 1) % 3;
    }
    
    batteryLabel->setText(QString("电池: %1%").arg(battery));
    signalLabel->setText(QString("信号: %1%").arg(signalStrength));
    gpsLabel->setText(gpsFixed ? "GPS: 已定位" : "GPS: 定位中");
    modeLabel->setText(QString("模式: %1").arg(modes[modeIndex]));
}
