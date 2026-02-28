#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDebug>
#include <QTimer>
#include <QMessageBox>
#include <QDoubleValidator>
#include <QIntValidator>

#include "RosWorker.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // 启动 ROS 后台线程
    rosWorker = new RosWorker();
    rosWorker->start();

    // 虚拟手柄定时器
    joyTimer = new QTimer(this);
    joyTimer->setInterval(20); // 20ms
    
    // 设置视觉识别显示区域的图片
    QPixmap pixmap(":/sample2.png");
    if (!pixmap.isNull()) {
        // 按比例缩放图片以适应label，保持原始比例，确保整个图片都能显示
        // 使用高质量抗锯齿缩放
        QPixmap scaledPixmap = pixmap.scaled(
            ui->visionLabel->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation
        );
        ui->visionLabel->setPixmap(scaledPixmap);
        
        // 设置高质量渲染
        ui->visionLabel->setAttribute(Qt::WA_TranslucentBackground);
    }
    
    // 应用深色主题样式表
    applyDarkTheme();
    
    // 添加工具提示
    setupTooltips();
    
    // 设置输入验证
    setupInputValidation();
    
    // 连接信号槽
    // 模式切换
    connect(ui->manualModeButton, &QPushButton::clicked, this, &MainWindow::onManualModeButtonClicked);
    connect(ui->autoModeButton, &QPushButton::clicked, this, &MainWindow::onAutoModeButtonClicked);
    connect(ui->semiautoModeButton, &QPushButton::clicked, this, &MainWindow::onSemiautoModeButtonClicked);
    
    // 位置姿态控制
    connect(ui->cartesianButton, &QPushButton::clicked, this, &MainWindow::onCartesianButtonClicked);
    connect(ui->ptpButton, &QPushButton::clicked, this, &MainWindow::onPtpButtonClicked);
    
    // 手柄控制
    connect(ui->joystickEnableCheckBox, &QCheckBox::clicked, this, &MainWindow::onJoystickEnableCheckBoxClicked);
    connect(ui->zForwardButton, &QPushButton::pressed, this, &MainWindow::onZForwardButtonPressed);
    connect(ui->zForwardButton, &QPushButton::released, this, &MainWindow::onZForwardButtonReleased);
    connect(ui->zBackwardButton, &QPushButton::pressed, this, &MainWindow::onZBackwardButtonPressed);
    connect(ui->zBackwardButton, &QPushButton::released, this, &MainWindow::onZBackwardButtonReleased);
    
    // 参数调节
    connect(ui->speedSlider, &QSlider::valueChanged, this, &MainWindow::onSpeedSliderValueChanged);
    connect(ui->sensitivitySlider, &QSlider::valueChanged, this, &MainWindow::onSensitivitySliderValueChanged);
    
    // 安全设置
    connect(ui->emergencyButton, &QPushButton::clicked, this, &MainWindow::onEmergencyButtonClicked);
    connect(ui->collisionCheckBox, &QCheckBox::clicked, this, &MainWindow::onCollisionCheckBoxClicked);
    connect(ui->workAreaLimitCheckBox, &QCheckBox::clicked, this, &MainWindow::onWorkAreaLimitCheckBoxClicked);
    connect(ui->autoReturnCheckBox, &QCheckBox::clicked, this, &MainWindow::onAutoReturnCheckBoxClicked);
    
    // 视觉识别
    connect(ui->nutRecognitionButton, &QPushButton::clicked, this, &MainWindow::onNutRecognitionButtonClicked);
    connect(ui->boltRecognitionButton, &QPushButton::clicked, this, &MainWindow::onBoltRecognitionButtonClicked);
    connect(ui->objectRecognitionButton, &QPushButton::clicked, this, &MainWindow::onObjectRecognitionButtonClicked);
    connect(ui->prevTargetButton, &QPushButton::clicked, this, &MainWindow::onPrevTargetButtonClicked);
    connect(ui->nextTargetButton, &QPushButton::clicked, this, &MainWindow::onNextTargetButtonClicked);
    connect(ui->alignButton, &QPushButton::clicked, this, &MainWindow::onAlignButtonClicked);
    connect(ui->captureButton, &QPushButton::clicked, this, &MainWindow::onCaptureButtonClicked);
    connect(ui->recordButton, &QPushButton::clicked, this, &MainWindow::onRecordButtonClicked);
    connect(ui->cameraSourceComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onCameraSourceComboBoxCurrentIndexChanged);
    
    // 连接 ROS 状态更新信号到槽函数
    connect(rosWorker, &RosWorker::statusUpdated, this, 
        [this](QString target, QString msg, int code) 
        {
            qDebug() << "Received status update from ROS:" << target << msg << code;
            
            QLabel* targetLabel = nullptr;

            // 路由到具体的 QLabel
            
            if (target == "VISION_STATUS") {   // 更新视觉识别结果文本框
                targetLabel = ui->recognitionResultLabel;
            } else if (target == "SYSTEM_STATUS") {   // 更新左下角系统状态
                targetLabel = ui->systemStatusLabel;
            } else if (target == "JOY_STATUS") {   // 更新手柄操作状态
                targetLabel = ui->joyStatusLabel;  
            } else if (target == "VISION_TARGET") {   // 更新目标指示器
                targetLabel = ui->targetIndexLabel;
            }
            
            QString colorName = "white"; // 默认颜色
            if (code == -1) {
                colorName = "#FF4C4C"; // 亮红色
            } else if (code == 2) {
                colorName = "#FFD700"; // 金黄色
            } else if (code == 1) {
                colorName = "#00FF00"; // 亮绿色
            }

            if (target == "PLAN_SERVER_FINISHED")  // 规划完成
            {   
                ui->joystickEnableCheckBox->setEnabled(true);  // 规划完成后恢复手柄控制
                ui->cartesianButton->setEnabled(true); // 规划完成后恢复按钮
                ui->ptpButton->setEnabled(true);
            } else {
                // 用 HTML 标签包裹文字
                QString richText = QString("<font color='%1'>%2</font>").arg(colorName).arg(msg);
                
                targetLabel->setText(richText);
            }
        });
    

    // 连接摇杆定时器信号到槽函数
    connect(joyTimer, &QTimer::timeout, this, [this]() {
        // 获取摇杆输入
        float left_x = ui->joystickXY->getOutX();
        float left_y = ui->joystickXY->getOutY();
        float right_x = ui->joystickYawPitch->getOutX();
        float right_y = ui->joystickYawPitch->getOutY();

        // 坐标映射
        QJsonObject data;
        data["lx"] = -left_y;         // 左摇杆上下 -> X
        data["ly"] = left_x;          // 左摇杆左右 -> Y
        data["lz"] = current_joy_z_;  // 扳机 -> Z
        
        data["ax"] = right_x;         // 右摇杆左右 -> 摇头 Roll
        data["ay"] = -right_y;        // 右摇杆上下 -> 点头 Pitch
        data["az"] = 0.0;             // 原代码中 Z轴旋转暂未映射

        rosWorker->sendCommand("joy", "DATA", data);
    });

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    
    // 重新调整视觉识别显示区域的图片大小
    QPixmap pixmap(":/sample2.png");
    if (!pixmap.isNull()) {
        // 使用高质量抗锯齿缩放
        QPixmap scaledPixmap = pixmap.scaled(
            ui->visionLabel->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation
        );
        ui->visionLabel->setPixmap(scaledPixmap);
    }
}

// 辅助函数：验证位置姿态输入
bool MainWindow::validateInput(double &x, double &y, double &z, double &roll, double &pitch, double &yaw)
{
    bool ok;
    x = ui->xLineEdit->text().toDouble(&ok);
    y = ui->yLineEdit->text().toDouble(&ok);
    z = ui->zLineEdit->text().toDouble(&ok);
    roll = ui->rollLineEdit->text().toDouble(&ok);
    pitch = ui->pitchLineEdit->text().toDouble(&ok);
    yaw = ui->yawLineEdit->text().toDouble(&ok);
    
    if (!ok) {
        QMessageBox::warning(this, "输入错误", "请输入有效的数值！");
        return false;
    }
    return true;
}

// 位置姿态控制槽函数
void MainWindow::onCartesianButtonClicked()
{
    double x, y, z, roll, pitch, yaw;
    if (!validateInput(x, y, z, roll, pitch, yaw))
        return;
    
    qDebug() << "直线运动 (Cartesian) 按钮点击";
    qDebug() << "目标位置: (" << x << "," << y << "," << z << ")";
    qDebug() << "目标姿态: (" << roll << "," << pitch << "," << yaw << ")";
    
    ui->statusbar->showMessage("执行笛卡尔直线路径规划...", 2000);
    ui->operationResultLabel->setStyleSheet("color: white;");
    ui->operationResultLabel->setText(QString("正在移动到 (%1, %2, %3)...").arg(x).arg(y).arg(z));

    ui->cartesianButton->setEnabled(false); // 规划过程中禁用按钮，防止重复点击
    ui->ptpButton->setEnabled(false);

    if (joyTimer->isActive()) {
        ui->joystickEnableCheckBox->setChecked(false); // 规划时禁用手柄控制
        ui->joystickEnableCheckBox->setEnabled(false);
        joyTimer->stop();
    }

    rosWorker->sendCommand("pose", "MOVE_LINEAR", {
        {"x", x},
        {"y", y},
        {"z", z},
        {"roll", roll},
        {"pitch", pitch},
        {"yaw", yaw}
    });
}

void MainWindow::onPtpButtonClicked()
{
    double x, y, z, roll, pitch, yaw;
    if (!validateInput(x, y, z, roll, pitch, yaw))
        return;
    
    qDebug() << "关节规划 (PTP) 按钮点击";
    qDebug() << "目标位置: (" << x << "," << y << "," << z << ")";
    qDebug() << "目标姿态: (" << roll << "," << pitch << "," << yaw << ")";
    
    ui->statusbar->showMessage("执行关节空间路径规划...", 2000);
    ui->operationResultLabel->setStyleSheet("color: white;");
    ui->operationResultLabel->setText(QString("正在PTP移动到 (%1, %2, %3, %4, %5, %6)...").arg(x).arg(y).arg(z).arg(roll).arg(pitch).arg(yaw));

    ui->cartesianButton->setEnabled(false); // 规划过程中禁用按钮，防止重复点击
    ui->ptpButton->setEnabled(false);

    if (joyTimer->isActive()) {
        ui->joystickEnableCheckBox->setChecked(false); // 规划时禁用手柄控制
        ui->joystickEnableCheckBox->setEnabled(false);
        joyTimer->stop();
    }

    rosWorker->sendCommand("pose", "MOVE_PTP", {
        {"x", x},
        {"y", y},
        {"z", z},
        {"roll", roll},
        {"pitch", pitch},
        {"yaw", yaw}
    });
}

// 手柄控制槽函数
void MainWindow::onJoystickEnableCheckBoxClicked(bool checked)
{
    qDebug() << "手柄激活状态: " << checked;
    ui->statusbar->showMessage(checked ? "手柄已激活" : "手柄已禁用", 2000);
    ui->joyStatusLabel->setText(checked ? "已激活" : "就绪");

    if (checked) {
        joyTimer->start();
        // 发送启动命令
        rosWorker->sendCommand("joy", "START_JOY", {});
    } else {
        joyTimer->stop();
    }
}

void MainWindow::onZForwardButtonPressed()
{
    qDebug() << "前进 (LT) 按钮按下";
    ui->statusbar->showMessage("Z轴前进...", 1000);
    current_joy_z_ = 1.0f; // 模拟 LT 扳机被按下
}

void MainWindow::onZForwardButtonReleased()
{
    qDebug() << "前进 (LT) 按钮释放";
    ui->statusbar->showMessage("Z轴停止", 1000);
    current_joy_z_ = 0.0f; // 模拟 LT 扳机被释放
}

void MainWindow::onZBackwardButtonPressed()
{
    qDebug() << "后退 (RT) 按钮按下";
    ui->statusbar->showMessage("Z轴后退...", 1000);
    current_joy_z_ = -1.0f; // 模拟 RT 扳机被按下
}

void MainWindow::onZBackwardButtonReleased()
{
    qDebug() << "后退 (RT) 按钮释放";
    ui->statusbar->showMessage("Z轴停止", 1000);
    current_joy_z_ = 0.0f; // 模拟 RT 扳机被释放
}

// 视觉识别槽函数
void MainWindow::onAlignButtonClicked()
{
    qDebug() << "执行对准按钮点击";
    ui->statusbar->showMessage("正在执行视觉对准...", 3000);
    
    // 模拟对准过程
    ui->recognitionResultLabel->setText("正在计算目标位置...");
    
    QTimer::singleShot(1000, [this]() {
        ui->recognitionResultLabel->setText("正在规划运动路径...");
    });
    
    QTimer::singleShot(2000, [this]() {
        ui->recognitionResultLabel->setText("正在移动到目标位置...");
    });
    
    QTimer::singleShot(3000, [this]() {
        ui->recognitionResultLabel->setText("✓ 对准完成！可以开始操作");
        ui->statusbar->showMessage("视觉对准完成", 2000);
    });
}

// 辅助函数：切换模式
void MainWindow::switchMode(QPushButton *activeButton, const QString &modeName)
{
    // 重置所有模式按钮
    ui->manualModeButton->setChecked(false);
    ui->autoModeButton->setChecked(false);
    ui->semiautoModeButton->setChecked(false);
    
    // 激活当前按钮
    activeButton->setChecked(true);
    
    qDebug() << "切换到" << modeName << "模式";
    ui->statusbar->showMessage("切换到" + modeName + "模式", 2000);
    ui->systemStatusLabel->setText("系统状态: " + modeName + "模式");
}

// 模式切换槽函数
void MainWindow::onManualModeButtonClicked()
{
    switchMode(ui->manualModeButton, "手动");
}

void MainWindow::onAutoModeButtonClicked()
{
    switchMode(ui->autoModeButton, "自动");
}

void MainWindow::onSemiautoModeButtonClicked()
{
    switchMode(ui->semiautoModeButton, "半自动");
}

// 参数调节槽函数
void MainWindow::onSpeedSliderValueChanged(int value)
{
    qDebug() << "速度限制调整为: " << value;
    ui->statusbar->showMessage(QString("速度限制: %1%").arg(value), 1000);

    rosWorker->sendCommand("joy_param", "SET_SPEED_LIMIT", {
        {"value", value}
    });
}

void MainWindow::onSensitivitySliderValueChanged(int value)
{
    qDebug() << "灵敏度调整为: " << value;
    ui->statusbar->showMessage(QString("灵敏度: %1%").arg(value), 1000);

    rosWorker->sendCommand("joy_param", "SET_SENSITIVITY", {
        {"value", value}
    });
}

// 安全设置槽函数
void MainWindow::onEmergencyButtonClicked()
{
    // 紧急停止确认对话框
    QMessageBox::StandardButton reply = QMessageBox::question(
        this,
        "紧急停止确认",
        "您确定要执行紧急停止吗？\n\n这将立即停止所有机械臂运动！",
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No
    );
    
    if (reply == QMessageBox::Yes) {
        qDebug() << "紧急停止按钮点击";
        ui->statusbar->showMessage("紧急停止已激活", 5000);
        ui->joyStatusLabel->setText("紧急停止");
        ui->systemStatusLabel->setText("紧急停止");
        
        // 禁用手柄控制
        ui->joystickEnableCheckBox->setChecked(false);
    }
}

void MainWindow::onCollisionCheckBoxClicked(bool checked)
{
    qDebug() << "碰撞检测状态: " << checked;
    ui->statusbar->showMessage(checked ? "碰撞检测已启用" : "碰撞检测已禁用", 2000);
}

void MainWindow::onWorkAreaLimitCheckBoxClicked(bool checked)
{
    qDebug() << "工作区域限制状态: " << checked;
    ui->statusbar->showMessage(checked ? "工作区域限制已启用" : "工作区域限制已禁用", 2000);
}

void MainWindow::onAutoReturnCheckBoxClicked(bool checked)
{
    qDebug() << "超出自动返回状态: " << checked;
    ui->statusbar->showMessage(checked ? "超出自动返回已启用" : "超出自动返回已禁用", 2000);
}

// 视觉识别槽函数
void MainWindow::onNutRecognitionButtonClicked()
{
    qDebug() << "螺母识别模式按钮点击";
    ui->statusbar->showMessage("切换到螺母识别模式", 2000);
    ui->recognitionResultLabel->setText("识别模式: 螺母识别");
}

void MainWindow::onBoltRecognitionButtonClicked()
{
    qDebug() << "螺栓识别模式按钮点击";
    ui->statusbar->showMessage("切换到螺栓识别模式", 2000);
    ui->recognitionResultLabel->setText("识别模式: 螺栓识别");
}

void MainWindow::onObjectRecognitionButtonClicked()
{
    qDebug() << "通用物体识别模式按钮点击";
    ui->statusbar->showMessage("切换到通用物体识别模式", 2000);
    ui->recognitionResultLabel->setText("识别模式: 通用物体识别");
}

void MainWindow::onPrevTargetButtonClicked()
{
    qDebug() << "上一个目标按钮点击";
    ui->statusbar->showMessage("选择上一个目标", 1000);
    ui->targetIndexLabel->setText("目标 1/3");
}

void MainWindow::onNextTargetButtonClicked()
{
    qDebug() << "下一个目标按钮点击";
    ui->statusbar->showMessage("选择下一个目标", 1000);
    ui->targetIndexLabel->setText("目标 2/3");
}

void MainWindow::onCaptureButtonClicked()
{
    qDebug() << "截图按钮点击";
    ui->statusbar->showMessage("正在截图...", 1000);
    ui->recognitionResultLabel->setText("截图已保存");
}

void MainWindow::onRecordButtonClicked()
{
    qDebug() << "录制按钮点击";
    ui->statusbar->showMessage("开始录制...", 1000);
    ui->recognitionResultLabel->setText("录制中...");
}

void MainWindow::onCameraSourceComboBoxCurrentIndexChanged(int index)
{
    QString source = ui->cameraSourceComboBox->currentText();
    qDebug() << "相机源切换为: " << source;
    ui->statusbar->showMessage(QString("相机源: %1").arg(source), 1000);
}


// 应用深色主题样式表
void MainWindow::applyDarkTheme()
{
    QString styleSheet = R"(
        QMainWindow {
            background-color: #1E1E1E;
        }
        
        QWidget {
            background-color: #1E1E1E;
            color: #FFFFFF;
            font-family: "Microsoft YaHei", "SimHei", Arial;
            font-size: 10pt;
        }
        
        QGroupBox {
            background-color: #2D2D2D;
            border: 2px solid #3D3D3D;
            border-radius: 8px;
            margin-top: 12px;
            padding-top: 12px;
            font-weight: bold;
        }
        
        QGroupBox::title {
            subcontrol-origin: margin;
            subcontrol-position: top left;
            left: 12px;
            padding: 0 8px;
            color: #4FC3F7;
        }
        
        QPushButton {
            background-color: #3D3D3D;
            border: 1px solid #555555;
            border-radius: 6px;
            padding: 8px 16px;
            min-height: 32px;
            font-weight: bold;
        }
        
        QPushButton:hover {
            background-color: #4D4D4D;
            border: 1px solid #007ACC;
        }
        
        QPushButton:pressed {
            background-color: #2D2D2D;
        }
        
        QPushButton:checked {
            background-color: #007ACC;
            border: 1px solid #0099FF;
        }
        
        QLineEdit {
            background-color: #252525;
            border: 1px solid #555555;
            border-radius: 4px;
            padding: 6px;
            min-height: 24px;
        }
        
        QLineEdit:focus {
            border: 2px solid #007ACC;
            background-color: #2D2D2D;
        }
        
        QLabel {
            color: #E0E0E0;
            background-color: transparent;
            border: none;
        }
        
        QSlider::groove:horizontal {
            height: 8px;
            background: #3D3D3D;
            border-radius: 4px;
        }
        
        QSlider::handle:horizontal {
            background: #007ACC;
            width: 18px;
            margin: -5px 0;
            border-radius: 9px;
        }
        
        QSlider::handle:horizontal:hover {
            background: #0099FF;
        }
        
        QCheckBox {
            spacing: 8px;
            background-color: transparent;
            border: none;
        }
        
        QCheckBox::indicator {
            width: 20px;
            height: 20px;
            border: 2px solid #555555;
            border-radius: 4px;
            background-color: #252525;
        }
        
        QCheckBox::indicator:checked {
            background-color: #007ACC;
            border-color: #007ACC;
            image: url(:/icons/check.png);
        }
        
        QCheckBox::indicator:hover {
            border-color: #007ACC;
        }
        
        QComboBox {
            background-color: #252525;
            border: 1px solid #555555;
            border-radius: 4px;
            padding: 6px;
            min-height: 24px;
        }
        
        QComboBox:hover {
            border: 1px solid #007ACC;
        }
        
        QComboBox::drop-down {
            border: none;
            width: 24px;
        }
        
        QComboBox QAbstractItemView {
            background-color: #2D2D2D;
            border: 1px solid #555555;
            selection-background-color: #007ACC;
        }
        
        QStatusBar {
            background-color: #252525;
            border-top: 1px solid #3D3D3D;
        }
        
        QMenuBar {
            background-color: #2D2D2D;
            border-bottom: 1px solid #3D3D3D;
        }
        
        QMenuBar::item {
            background-color: transparent;
            padding: 8px 12px;
        }
        
        QMenuBar::item:selected {
            background-color: #007ACC;
        }
        
        /* 紧急停止按钮特殊样式 */
        QPushButton#emergencyButton {
            background-color: #FF4444;
            color: white;
            border: 2px solid #FF6666;
            font-size: 12pt;
            font-weight: bold;
        }
        
        QPushButton#emergencyButton:hover {
            background-color: #FF6666;
            border: 2px solid #FF8888;
        }
        
        QPushButton#emergencyButton:pressed {
            background-color: #CC0000;
        }
        
        /* 视觉识别显示区域 */
        QLabel#visionLabel {
            background-color: #000000;
            border: 2px solid #3D3D3D;
            border-radius: 4px;
            color: #888888;
        }
        
        /* 手柄摇杆区域 */
        QWidget#joystickXY, QWidget#joystickYawPitch {
            background-color: #252525;
            border: 2px solid #3D3D3D;
            border-radius: 8px;
        }
    )";
    
    this->setStyleSheet(styleSheet);
}

// 设置工具提示
void MainWindow::setupTooltips()
{
    // 模式切换
    ui->manualModeButton->setToolTip("手动模式：完全由操作员控制机械臂运动");
    ui->autoModeButton->setToolTip("自动模式：机械臂按照预设程序自动执行任务");
    ui->semiautoModeButton->setToolTip("半自动模式：操作员与自动系统协同控制");
    
    // 位置姿态控制
    ui->xLineEdit->setToolTip("末端执行器X坐标（单位：毫米）");
    ui->yLineEdit->setToolTip("末端执行器Y坐标（单位：毫米）");
    ui->zLineEdit->setToolTip("末端执行器Z坐标（单位：毫米）");
    ui->rollLineEdit->setToolTip("末端执行器Roll角（单位：度）");
    ui->pitchLineEdit->setToolTip("末端执行器Pitch角（单位：度）");
    ui->yawLineEdit->setToolTip("末端执行器Yaw角（单位：度）");
    ui->cartesianButton->setToolTip("笛卡尔直线路径规划：末端执行器沿直线路径运动");
    ui->ptpButton->setToolTip("关节空间路径规划：各关节独立运动到目标位置");
    
    // 手柄控制
    ui->joystickEnableCheckBox->setToolTip("勾选后激活手柄控制，防止误触");
    ui->zForwardButton->setToolTip("Z轴前进（对应手柄LT扳机）");
    ui->zBackwardButton->setToolTip("Z轴后退（对应手柄RT扳机）");
    
    // 参数调节
    ui->speedSlider->setToolTip("调整机械臂运动速度限制（0-100%）");
    ui->sensitivitySlider->setToolTip("调整手柄控制灵敏度（0-100%）");
    
    // 安全设置
    ui->emergencyButton->setToolTip("紧急停止：立即停止所有运动（危险操作）");
    ui->collisionCheckBox->setToolTip("启用碰撞检测，防止机械臂与环境碰撞");
    
    // 视觉识别
    ui->nutRecognitionButton->setToolTip("切换到螺母识别模式");
    ui->boltRecognitionButton->setToolTip("切换到螺栓识别模式");
    ui->objectRecognitionButton->setToolTip("切换到通用物体识别模式");
    ui->prevTargetButton->setToolTip("选择上一个识别到的目标");
    ui->nextTargetButton->setToolTip("选择下一个识别到的目标");
    ui->alignButton->setToolTip("执行视觉对准，自动移动到目标位置");
    ui->captureButton->setToolTip("保存当前视觉画面截图");
    ui->recordButton->setToolTip("开始/停止视频录制");
    ui->cameraSourceComboBox->setToolTip("选择相机源：末端相机/全景相机/外部相机");
}

// 设置输入验证
void MainWindow::setupInputValidation()
{
    // 设置位置输入范围（-1000mm 到 1000mm）
    QDoubleValidator *posValidator = new QDoubleValidator(-1000.0, 1000.0, 2, this);
    posValidator->setNotation(QDoubleValidator::StandardNotation);
    
    ui->xLineEdit->setValidator(posValidator);
    ui->yLineEdit->setValidator(posValidator);
    ui->zLineEdit->setValidator(posValidator);
    
    // 设置角度输入范围（-180度 到 180度）
    QDoubleValidator *angleValidator = new QDoubleValidator(-180.0, 180.0, 2, this);
    angleValidator->setNotation(QDoubleValidator::StandardNotation);
    
    ui->rollLineEdit->setValidator(angleValidator);
    ui->pitchLineEdit->setValidator(angleValidator);
    ui->yawLineEdit->setValidator(angleValidator);
}
