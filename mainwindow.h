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
    void onVideo1SourceChanged(int index);
    void onVideo2SourceChanged(int index);
    void onStartButtonClicked();
    void onStopButtonClicked();
    void onEmergencyButtonClicked();
    void onCalibrateButtonClicked();
    void onSettingsButtonClicked();
    void onConnectButtonClicked();
    void updateStatusBar();

private:
    Ui::MainWindow *ui;
    QLabel *batteryLabel;
    QLabel *signalLabel;
    QLabel *gpsLabel;
    QLabel *modeLabel;
};
#endif // MAINWINDOW_H
