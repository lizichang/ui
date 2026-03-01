#ifndef ROSWORKER_H
#define ROSWORKER_H

#include <QThread>
#include <QString>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class RosWorker : public QThread
{
    Q_OBJECT

public:
    explicit RosWorker(QObject *parent = nullptr);
    ~RosWorker() override;

    // UI 发送命令到 ROS
    void sendCommand(const QString& type, const QString& action = "", const QJsonObject& data = QJsonObject());

signals:
    // ROS 发送状态更新到 UI
    void statusUpdated(QString target, QString msg, int code);

protected:
    void run() override;

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;
};

#endif // ROSWORKER_H