#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>

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
    // 位置姿态控制
    void onCartesianButtonClicked();
    void onPtpButtonClicked();
    
    // 手柄控制
    void onJoystickEnableCheckBoxClicked(bool checked);
    void onZForwardButtonPressed();
    void onZForwardButtonReleased();
    void onZBackwardButtonPressed();
    void onZBackwardButtonReleased();
    
    // 视觉识别
    void onAlignButtonClicked();
    
    // 状态更新
    void updateStatusBar();

private:
    Ui::MainWindow *ui;
    QLabel *controlStatusLabel;
};
#endif // MAINWINDOW_H
