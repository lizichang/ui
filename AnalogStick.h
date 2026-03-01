#ifndef ANALOGSTICK_H
#define ANALOGSTICK_H

#include <QWidget>
#include <QPointF>
#include <QString>

class AnalogStick : public QWidget
{
    Q_OBJECT // 必须包含宏以支持信号与槽

public:
    explicit AnalogStick(QWidget *parent = nullptr);

    // 获取当前的 XY 摇杆值 [-1.0, 1.0]
    float getOutX() const { return out_x; }
    float getOutY() const { return out_y; }

signals:
    // 当摇杆移动时发出信号，可以直接连到 RosWorker 的发送函数上
    void moved(float x, float y);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    void updateStickPos(const QPointF& mouse_pos);

    QString name;
    
    // 状态参数
    QPointF center;
    QPointF handle_pos;
    bool is_pressed;
    
    // 尺寸参数 (已按 120x120 比例缩小)
    float max_radius;
    float handle_radius;

    // 输出值
    float out_x;
    float out_y;
};

#endif // ANALOGSTICK_H