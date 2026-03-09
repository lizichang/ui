#include "AnalogStick.h"
#include <QPainter>
#include <QMouseEvent>
#include <QRadialGradient>
#include <cmath>

AnalogStick::AnalogStick(QWidget *parent)
    : QWidget(parent), name("Stick"), is_pressed(false), out_x(0.0f), out_y(0.0f)
{

    setMinimumSize(120, 120);
    
    // 比例系数 0.6 (120/200)
    max_radius = 48.0f;   // 原 80.0
    handle_radius = 21.0f; // 原 35.0

    center = QPointF(width() / 2.0, height() / 2.0);
    handle_pos = center;
}

void AnalogStick::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    float w = width();
    float h = height();
    center = QPointF(w / 2.0, h / 2.0);

    // 1. 画底座 (深色背景)
    QRadialGradient grad_bg(center, max_radius);
    grad_bg.setColorAt(0, QColor(40, 40, 40));
    grad_bg.setColorAt(1, QColor(20, 20, 20));
    painter.setBrush(QBrush(grad_bg));
    painter.setPen(QPen(QColor(60, 60, 60), 2));
    painter.drawEllipse(center, max_radius, max_radius);

    // 2. 十字刻度线
    painter.setPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
    painter.drawLine(QPointF(center.x() - max_radius, center.y()), 
                     QPointF(center.x() + max_radius, center.y()));
    painter.drawLine(QPointF(center.x(), center.y() - max_radius), 
                     QPointF(center.x(), center.y() + max_radius));

    // 3. 计算摇杆头的位置 (如果没有按下，则回中)
    if (!is_pressed) {
        handle_pos = center;
    }

    // 4. 画摇杆杆身 (连接线)
    // 线宽也等比例缩小为 6 (原 10)
    painter.setPen(QPen(QColor(100, 100, 100), 6, Qt::SolidLine, Qt::RoundCap));
    painter.drawLine(center, handle_pos);

    // 5. 画摇杆头 (Thumb Stick)
    QRadialGradient grad_handle(handle_pos, handle_radius);
    grad_handle.setColorAt(0, QColor(80, 80, 80));
    grad_handle.setColorAt(1, QColor(30, 30, 30));

    // 按下变色
    if (is_pressed) {
        painter.setBrush(QBrush(QColor(200, 50, 50))); // 红色高亮
    } else {
        painter.setBrush(QBrush(grad_handle));
    }

    painter.setPen(QPen(QColor(10, 10, 10), 2));
    painter.drawEllipse(handle_pos, handle_radius, handle_radius);
}

void AnalogStick::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        // 使用 std::hypot 计算欧氏距离
        float dist = std::hypot(event->pos().x() - center.x(), event->pos().y() - center.y());
        
        // 判定点击范围，容差也缩小为 12 (原 20)
        if (dist <= max_radius + 12.0f) { 
            is_pressed = true;
            updateStickPos(event->pos());
        }
    }
}

void AnalogStick::mouseMoveEvent(QMouseEvent *event)
{
    if (is_pressed) {
        updateStickPos(event->pos());
    }
}

void AnalogStick::mouseReleaseEvent(QMouseEvent *event)
{
    Q_UNUSED(event);
    is_pressed = false;
    out_x = 0.0f;
    out_y = 0.0f;
    emit moved(0.0f, 0.0f);
    update();
}

void AnalogStick::updateStickPos(const QPointF& mouse_pos)
{
    float dx = mouse_pos.x() - center.x();
    float dy = mouse_pos.y() - center.y();

    float dist = std::hypot(dx, dy);

    // 限制在圆内 (Clamping)
    if (dist > max_radius) {
        float ratio = max_radius / dist;
        dx *= ratio;
        dy *= ratio;
    }

    handle_pos = QPointF(center.x() + dx, center.y() + dy);

    // 归一化输出 [-1.0, 1.0]
    // 注意：屏幕坐标系 Y 向下是正，但通常手柄向上是正，这里取反
    out_x = dx / max_radius;
    out_y = -(dy / max_radius);

    emit moved(out_x, out_y);
    update();
}