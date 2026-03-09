#include "RosWorker.h"
#include <QDebug>

RosWorker::RosWorker(QObject *parent) : QThread(parent)
{
    // 初始化 ROS 2 节点
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("gui_ros_node");

    // 创建发布者 (向后端发指令)
    pub_cmd_ = node_->create_publisher<std_msgs::msg::String>("/gui/command", 10);
    // 创建订阅者 (接收后端状态更新)
    sub_status_ = node_->create_subscription<std_msgs::msg::String>(
        "/gui/status", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            QJsonParseError error;
            QJsonDocument doc = QJsonDocument::fromJson(QString::fromStdString(msg->data).toUtf8(), &error);
            
            if (error.error != QJsonParseError::NoError || !doc.isObject()) return;
            
            QJsonObject payload = doc.object();
            QString type = payload["type"].toString();

            // 发射 Qt 信号
            if (type == "status") {
                QString target = payload["target"].toString();
                QString msg = payload["msg"].toString();
                int code = payload["code"].toInt(0);
                
                emit statusUpdated(target, msg, code);
            }
        });
    // 创建订阅者 (接收后端图像更新)
    sub_image_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/vision/output_image", 10, 
        [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
            try {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
                QImage image(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, static_cast<int>(cv_ptr->image.step), QImage::Format_RGB888);
                
                emit imageUpdated(image.copy());
            } catch (cv_bridge::Exception& e) {
                qWarning() << "cv_bridge exception: " << e.what();
            }
        });

    qDebug() << "ROS Worker initialized";
}

RosWorker::~RosWorker()
{
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    qDebug() << "ROS Worker shutdown";
}

void RosWorker::sendCommand(const QString& type, const QString& action, const QJsonObject& data)
{
    if (!pub_cmd_) {
        qWarning() << "Publisher not initialized";
        return;
    }

    QJsonObject cmd;
    cmd["type"] = type;
    if (!action.isEmpty()) {
        cmd["action"] = action;
    }
    if (!data.isEmpty()) {
        cmd["data"] = data;
    }

    std_msgs::msg::String msg;
    msg.data = QJsonDocument(cmd).toJson(QJsonDocument::Compact).toStdString();
    pub_cmd_->publish(msg);

    qDebug() << "Sent command to ROS:" << QString::fromStdString(msg.data);
}

// ROS 线程主循环
void RosWorker::run()
{
    if (rclcpp::ok()) {
        qDebug() << "ROS Worker thread started";
        rclcpp::spin(node_);
    }
}