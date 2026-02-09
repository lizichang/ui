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
    
    // 初始化控制状态栏
    controlStatusLabel = new QLabel("手柄操作状态: 就绪");
    ui->statusbar->addPermanentWidget(controlStatusLabel);
    
    // 连接信号槽
    // 位置姿态控制
    connect(ui->cartesianButton, &QPushButton::clicked, this, &MainWindow::onCartesianButtonClicked);
    connect(ui->ptpButton, &QPushButton::clicked, this, &MainWindow::onPtpButtonClicked);
    
    // 手柄控制
    connect(ui->joystickEnableCheckBox, &QCheckBox::clicked, this, &MainWindow::onJoystickEnableCheckBoxClicked);
    connect(ui->zForwardButton, &QPushButton::pressed, this, &MainWindow::onZForwardButtonPressed);
    connect(ui->zForwardButton, &QPushButton::released, this, &MainWindow::onZForwardButtonReleased);
    connect(ui->zBackwardButton, &QPushButton::pressed, this, &MainWindow::onZBackwardButtonPressed);
    connect(ui->zBackwardButton, &QPushButton::released, this, &MainWindow::onZBackwardButtonReleased);
    
    // 视觉识别
    connect(ui->alignButton, &QPushButton::clicked, this, &MainWindow::onAlignButtonClicked);
    
    // 模拟数据更新
    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::updateStatusBar);
    timer->start(1000); // 每秒更新一次
}

MainWindow::~MainWindow()
{
    delete ui;
}

// 位置姿态控制槽函数
void MainWindow::onCartesianButtonClicked()
{
    qDebug() << "直线运动 (Cartesian) 按钮点击";
    ui->statusbar->showMessage("执行笛卡尔直线路径规划...", 2000);
}

void MainWindow::onPtpButtonClicked()
{
    qDebug() << "关节规划 (PTP) 按钮点击";
    ui->statusbar->showMessage("执行关节空间路径规划...", 2000);
}

// 手柄控制槽函数
void MainWindow::onJoystickEnableCheckBoxClicked(bool checked)
{
    qDebug() << "手柄激活状态: " << checked;
    ui->statusbar->showMessage(checked ? "手柄已激活" : "手柄已禁用", 2000);
    ui->controlStatusLabel->setText(checked ? "手柄操作状态: 已激活" : "手柄操作状态: 就绪");
}

void MainWindow::onZForwardButtonPressed()
{
    qDebug() << "前进 (LT) 按钮按下";
    ui->statusbar->showMessage("Z轴前进...", 1000);
}

void MainWindow::onZForwardButtonReleased()
{
    qDebug() << "前进 (LT) 按钮释放";
    ui->statusbar->showMessage("Z轴停止", 1000);
}

void MainWindow::onZBackwardButtonPressed()
{
    qDebug() << "后退 (RT) 按钮按下";
    ui->statusbar->showMessage("Z轴后退...", 1000);
}

void MainWindow::onZBackwardButtonReleased()
{
    qDebug() << "后退 (RT) 按钮释放";
    ui->statusbar->showMessage("Z轴停止", 1000);
}

// 视觉识别槽函数
void MainWindow::onAlignButtonClicked()
{
    qDebug() << "执行对准按钮点击";
    ui->statusbar->showMessage("正在执行视觉对准...", 2000);
    ui->recognitionResultLabel->setText("正在执行对准...");
}

// 状态更新槽函数
void MainWindow::updateStatusBar()
{
    // 模拟控制状态更新
    static QString statuses[] = {"就绪", "运行中", "接近奇异点", "碰撞警告"};
    static int statusIndex = 0;
    
    if (ui->joystickEnableCheckBox->isChecked()) {
        statusIndex = (statusIndex + 1) % 4;
        ui->controlStatusLabel->setText(QString("手柄操作状态: %1").arg(statuses[statusIndex]));
    }
}
