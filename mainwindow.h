#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    // 模式切换
    void onManualModeButtonClicked();
    void onAutoModeButtonClicked();
    void onSemiautoModeButtonClicked();
    
    // 位置姿态控制
    void onCartesianButtonClicked();
    void onPtpButtonClicked();
    
    // 手柄控制
    void onJoystickEnableCheckBoxClicked(bool checked);
    void onZForwardButtonPressed();
    void onZForwardButtonReleased();
    void onZBackwardButtonPressed();
    void onZBackwardButtonReleased();
    
    // 参数调节
    void onSpeedSliderValueChanged(int value);
    void onSensitivitySliderValueChanged(int value);
    
    // 安全设置
    void onEmergencyButtonClicked();
    void onCollisionCheckBoxClicked(bool checked);
    void onWorkAreaLimitCheckBoxClicked(bool checked);
    void onAutoReturnCheckBoxClicked(bool checked);
    
    // 视觉识别
    void onNutRecognitionButtonClicked();
    void onBoltRecognitionButtonClicked();
    void onObjectRecognitionButtonClicked();
    void onPrevTargetButtonClicked();
    void onNextTargetButtonClicked();
    void onAlignButtonClicked();
    void onCaptureButtonClicked();
    void onRecordButtonClicked();
    void onCameraSourceComboBoxCurrentIndexChanged(int index);
    
    // 状态更新
    void updateStatusBar();

protected:
    void resizeEvent(QResizeEvent *event) override;

private:
    Ui::MainWindow *ui;
    
    // 辅助函数
    void applyDarkTheme();
    void setupTooltips();
    void setupInputValidation();
    bool validateInput(double &x, double &y, double &z, double &roll, double &pitch, double &yaw);
    void switchMode(QPushButton *activeButton, const QString &modeName);
};
#endif // MAINWINDOW_H
