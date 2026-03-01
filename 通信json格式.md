这个界面的功能非常丰富且专业，涵盖了状态机（模式切换）、绝对控制（位姿）、相对控制（手柄）、视觉交互（目标选择）以及安全机制。

为了让 C++ (Qt) 和 Python (ROS 2) 的通信既能应对当前这么多功能，又能方便未来扩展，我们需要设计一套**“带有路由标签（Routing Tag）”的标准 JSON 协议**。

其核心思想是：**`type` 决定大分类，`action` 决定具体动作，`data` 携带可选参数。**

以下是为你量身定制的 JSON 协议字典。

---

### 一、 Command 协议：前端 ➡️ 后端 (`/gui/command`)

前端所有的操作（点击按钮、拉动滑块、操作摇杆）都封装成统一格式发给后端。

#### 1. 系统与安全机制 (System & Safety)

*对应 UI：模式切换、紧急停止、碰撞检测等。*

```json
// 紧急停止 (最高优先级)
{ "type": "system", "action": "ESTOP" }

// 模式切换
{ "type": "system", "action": "SET_MODE", "data": {"mode": "auto"} } // 取值: manual, auto, semi_auto

// 安全设置复选框
{ "type": "system", "action": "SET_SAFETY", "data": {"collision": true, "work_limit": false} }

```

#### 2. 位置姿态指令控制 (Pose Control)

*对应 UI：X/Y/Z/Roll/Pitch/Yaw 输入框与运动按钮。*

```json
// 直线运动
{
  "type": "pose",
  "action": "MOVE_LINEAR",
  "data": { "x": 0.0, "y": 0.0, "z": 150.5, "roll": 0.0, "pitch": 90.0, "yaw": 0.0 }
}

// 关节规划
{
  "type": "pose",
  "action": "MOVE_JOINT",
  "data": { "x": 0.0, "y": 0.0, "z": 150.5, "roll": 0.0, "pitch": 90.0, "yaw": 0.0 }
}

```

#### 3. 手柄与参数调节 (Joy & Params)

*对应 UI：虚拟双摇杆、LT/RT、速度灵敏度滑块。*

```json
// 虚拟手柄高频发送
{
  "type": "joy",
  "action": "DATA",
  "data": { 
    "lx": 0.5, "ly": 0.0, "lz": 1.0,  // lz 对应 LT/RT
    "ax": 0.0, "ay": 0.0, "az": -0.8 
  }
}

// 参数滑块 (拖动释放时发送)
{
  "type": "joy_param",
  "action": "SET_SPEED_LIMIT",
  "data": { "value": 80 } // 百分比 0-100
}

```

#### 4. 视觉识别与交互 (Vision Control)

*对应 UI：识别模式、目标选择、执行对准、截图录制。*

```json
// 切换识别模型
{ "type": "vision", "action": "SET_MODEL", "data": {"model": "nut"} } // 取值: nut, bolt, general

// 目标选择切换
{ "type": "vision", "action": "SELECT_NEXT" }
{ "type": "vision", "action": "SELECT_PREV" }

// 执行对准当前锁定的目标
{ "type": "vision", "action": "EXECUTE_ALIGN" }

// 视觉控制指令
{ "type": "vision", "action": "TAKE_SCREENSHOT" }

```

---

### 二、 Status 协议：后端 ➡️ 前端 (`/gui/status`)

后端发给前端的 JSON 主要用于**更新 UI 文本**、**改变指示灯颜色**，或者**动态刷新目标列表**。

格式标准：`{"type": "status", "target": "<UI组件ID>", "msg": "显示文本", "code": 状态码, "data": {附加数据}}`
*(其中 `code` 可用于前端判断颜色：0=正常，1=绿色/成功，2=警告/黄色，-1=错误/红色)*

#### 1. 纯文本状态更新

*对应 UI：左下角的“控制状态”、视觉面板的“识别结果”。*

```json
// 更新视觉识别结果文本框
{
  "type": "status",
  "target": "PLAN_STATUS",
  "msg": "正在移动到 (120, 30, 45)...",
  "code": 0
}

// 更新左下角系统状态
{
  "type": "status",
  "target": "SYSTEM_STATUS",
  "msg": "紧急停止已触发！",
  "code": -1
}

// 更新手柄操作状态
{
  "type": "status",
  "target": "JOY_STATUS",
  "msg": "接近奇异点",
  "code": 2
}

// 更新目标指示器
{
  "type": "status",
  "target": "VISION_TARGET",
  "msg": "目标 1/3",
  "code": 0
}
```

### 三、 代码层面的优雅实现建议

如果把所有的 JSON 解析都堆在一个回调函数里，代码很快就会变成一座几千行的 `if-else` 大山。建议使用**“路由分发（Dispatcher）”**模式。

#### Python 后端：建立分发字典

在 Python 中，利用字典映射函数，消灭 `if-else`。

```python
class BackendLogicNode(Node):
    def __init__(self):
        # ... 初始化 ...
        # 建立路由表
        self.command_router = {
            "system": self.handle_system_cmd,
            "pose": self.handle_pose_cmd,
            "joy": self.handle_joy_cmd,
            "vision": self.handle_vision_cmd,
            "param": self.handle_param_cmd
        }

    # 订阅 /gui/command 时统一用这个 callback 函数
    def gui_msg_callback(self, msg): 
        try:
            payload = json.loads(msg.data)
            cmd_type = payload.get("type")
            
            # 从路由表找对应的处理函数，找不到则报错
            handler = self.command_router.get(cmd_type)
            if handler:
                handler(payload.get("action"), payload.get("data", {}))
            else:
                self.get_logger().warning(f"未知的指令类型: {cmd_type}")
                
        except json.JSONDecodeError:
            pass

    def handle_system_cmd(self, action, data):
        if action == "ESTOP":
            self.trigger_emergency_stop()
        elif action == "SET_MODE":
            self.set_robot_mode(data.get("mode"))

```

#### C++ 前端：统一的发送包装器

在 `RosWorker` 中写一个接受类型、动作和数据的泛型函数。

```cpp
// RosWorker.h
void sendCommand(const QString& type, const QString& action, const QJsonObject& data = QJsonObject());

// RosWorker.cpp
void RosWorker::sendCommand(const QString& type, const QString& action, const QJsonObject& data) {
    QJsonObject payload;
    payload["type"] = type;
    payload["action"] = action;
    if (!data.isEmpty()) {
        payload["data"] = data;
    }
    
    QJsonDocument doc(payload);
    auto msg = std_msgs::msg::String();
    msg.data = doc.toJson(QJsonDocument::Compact).toStdString();
    pub_cmd_->publish(msg);
}

```

这样，UI 同事在给“直线运动”按钮绑定事件时，只需写一行极其清晰的代码：

```cpp
QJsonObject poseData;
poseData["x"] = ui->inputX->text().toDouble();
// ... 填充 y, z, r, p, y
rosWorker_->sendCommand("pose", "MOVE_LINEAR", poseData);

```

这套协议的延展性极强。不管以后界面上是加机械爪开合控制，还是加激光雷达点云开关，你只需要在字典里加一个新的 `type` 和 `action`，底层的 ROS 通信链路一行都不需要改。


<br></br>
<br></br>
<br></br>

---

这是一个非常成熟且易于扩展的架构设计。通过**路由分发（Dispatcher）**，你可以彻底消灭掉传统代码中那种长达几百行的 `if-else` 判断。

下面我将分别给出 **Python后端（接收指令并分发）** 和 **C++前端（封装打包并发送）** 的完整实现样例。

---

### 第一部分：Python 后端实现 (数据大脑 & 路由核心)

后端的关键是利用 Python **字典（Dictionary）可以直接映射函数** 的特性，把不同的 `type` 导向不同的处理函数。

```python
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class BackendLogicNode(Node):
    def __init__(self):
        super().__init__('backend_logic_node')
        
        # 1. 通信接口
        self.cmd_sub = self.create_subscription(String, '/gui/command_json', self.gui_msg_callback, 10)
        self.status_pub = self.create_publisher(String, '/gui/status_json', 10)
        
        # 2. 核心：建立路由分发字典 (Router)
        # 键是 JSON 中的 "type"，值是对应的处理函数引用
        self.command_router = {
            "system": self.handle_system_cmd,
            "pose":   self.handle_pose_cmd,
            "joy":    self.handle_joy_cmd,
            "vision": self.handle_vision_cmd,
            "param":  self.handle_param_cmd
        }
        self.get_logger().info("后端逻辑节点已启动，路由表加载完毕。")

    # ==========================================
    # 消息入口：主分发器 (Dispatcher)
    # ==========================================
    def gui_msg_callback(self, msg):
        try:
            # 1. 解析 JSON
            payload = json.loads(msg.data)
            cmd_type = payload.get("type", "")
            action = payload.get("action", "")
            data = payload.get("data", {})

            # 2. 路由分发
            handler = self.command_router.get(cmd_type)
            if handler:
                # 找到对应函数，把 action 和 data 传给它
                handler(action, data)
            else:
                self.get_logger().warning(f"丢弃未知的指令类型: {cmd_type}")

        except json.JSONDecodeError:
            self.get_logger().error(f"JSON 解析失败，非法格式: {msg.data}")

    # ==========================================
    # 子处理器：按模块处理具体业务
    # ==========================================
    def handle_system_cmd(self, action, data):
        if action == "ESTOP":
            self.get_logger().fatal("!!! 触发紧急停止 !!!")
            # 调用底层急停函数...
            self.send_status("SYSTEM_STATE", "急停已触发", code=-1)
            
        elif action == "SET_MODE":
            mode = data.get("mode", "manual")
            self.get_logger().info(f"切换系统模式至: {mode}")
            self.send_status("SYSTEM_STATE", f"当前模式: {mode}", code=0)

    def handle_pose_cmd(self, action, data):
        if action == "MOVE_LINEAR":
            x, y, z = data.get("x", 0.0), data.get("y", 0.0), data.get("z", 0.0)
            self.get_logger().info(f"执行直线运动至: X:{x}, Y:{y}, Z:{z}")
            self.send_status("ARM_STATE", "正在执行直线运动...")
            # 调用 MoveIt API ...

    def handle_joy_cmd(self, action, data):
        # 手柄由于高频发送，一般不需要 action，直接解包 data
        lx, az = data.get("lx", 0.0), data.get("az", 0.0)
        
        # 组装 ROS 原生 Twist 消息发布给底盘/机械臂
        twist = Twist()
        twist.linear.x = float(lx)
        twist.angular.z = float(az)
        # self.cmd_vel_pub.publish(twist)

    def handle_vision_cmd(self, action, data):
        if action == "EXECUTE_ALIGN":
            self.get_logger().info("视觉：开始执行对准目标")
            
    # ==========================================
    # 辅助工具：统一发送状态反馈
    # ==========================================
    def send_status(self, target, msg_text, code=0, data=None):
        payload = {
            "type": "status",
            "target": target,
            "msg": msg_text,
            "code": code,
            "data": data or {}
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

def main(args=None):
    rclpy.init(args=args)
    node = BackendLogicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

---

### 第二部分：C++ 前端实现 (封装打包 & 信号分发)

前端的核心是写一个**万能的发送函数**，并且在接收到后端状态时，**提前把 JSON 拆解好，再通过 Qt 信号发给对应的 UI 组件**。

#### 1. `RosWorker.h`

```cpp
#ifndef ROSWORKER_H
#define ROSWORKER_H

#include <QThread>
#include <QJsonObject>
#include <QJsonDocument>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class RosWorker : public QThread
{
    Q_OBJECT
public:
    explicit RosWorker(QObject *parent = nullptr);
    
    // 万能 JSON 发送接口 (参数带默认值，方便调用)
    void sendCommand(const QString& type, const QString& action = "", const QJsonObject& data = QJsonObject());

signals:
    // 将解析好的数据拆分为易用的信号，UI 层无需接触 JSON
    void statusUpdated(QString target, QString msg, int code);
    void targetIndicatorUpdated(int current, int total); // 专门针对某些复杂数据

protected:
    void run() override;

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;
};

#endif

```

#### 2. `RosWorker.cpp`

```cpp
#include "RosWorker.h"
#include <QJsonParseError>
#include <QDebug>

RosWorker::RosWorker(QObject *parent) : QThread(parent)
{
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("gui_ros_worker");

    pub_cmd_ = node_->create_publisher<std_msgs::msg::String>("/gui/command_json", 10);

    // 接收后端的 JSON 并进行解包分发
    sub_status_ = node_->create_subscription<std_msgs::msg::String>(
        "/gui/status_json", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            QJsonParseError error;
            QJsonDocument doc = QJsonDocument::fromJson(QString::fromStdString(msg->data).toUtf8(), &error);
            
            if (error.error != QJsonParseError::NoError || !doc.isObject()) return;
            
            QJsonObject payload = doc.object();
            QString type = payload["type"].toString();

            // 针对不同类型的反馈，发射不同的 Qt 信号
            if (type == "status") {
                QString target = payload["target"].toString();
                QString text = payload["msg"].toString();
                int code = payload["code"].toInt(0);
                
                emit statusUpdated(target, text, code);
                
                // 针对含有特殊业务逻辑的数据 (如目标数量更新)
                if (payload.contains("data")) {
                    QJsonObject dataObj = payload["data"].toObject();
                    if (dataObj.contains("current") && dataObj.contains("total")) {
                        emit targetIndicatorUpdated(dataObj["current"].toInt(), dataObj["total"].toInt());
                    }
                }
            }
        });
}

// ... run() ...

// 万能打包发送器
void RosWorker::sendCommand(const QString& type, const QString& action, const QJsonObject& data)
{
    if (!pub_cmd_) return;

    QJsonObject payload;
    payload["type"] = type;
    
    if (!action.isEmpty()) {
        payload["action"] = action;
    }
    
    if (!data.isEmpty()) {
        payload["data"] = data;
    }

    // 压缩为单行字符串
    QString jsonString = QJsonDocument(payload).toJson(QJsonDocument::Compact);
    
    auto msg = std_msgs::msg::String();
    msg.data = jsonString.toStdString();
    pub_cmd_->publish(msg);
}

```

---

### 第三部分：UI 层怎么调用？(极致清爽)

经过这样封装后，你的 UI 同事在 `MainWindow.cpp` 里面写代码就会变成一种享受：

```cpp
// 1. 发送：急停按钮被点击
connect(ui->btnEstop, &QPushButton::clicked, this, [this]() {
    rosWorker_->sendCommand("system", "ESTOP");
});

// 2. 发送：位置控制面板点击“直线运动”
connect(ui->btnMoveLinear, &QPushButton::clicked, this, [this]() {
    QJsonObject poseData;
    poseData["x"] = ui->inputX->text().toDouble();
    poseData["y"] = ui->inputY->text().toDouble();
    poseData["z"] = ui->inputZ->text().toDouble();
    
    rosWorker_->sendCommand("pose", "MOVE_LINEAR", poseData);
});

// 3. 接收：统一更新各种状态标签
connect(rosWorker_, &RosWorker::statusUpdated, this, [this](QString target, QString msg, int code) {
    QLabel* targetLabel = nullptr;
    
    // 路由到具体的 QLabel 组件
    if (target == "SYSTEM_STATE") targetLabel = ui->labelSystemState;
    else if (target == "ARM_STATE") targetLabel = ui->labelArmState;
    else if (target == "VISION_RESULT") targetLabel = ui->labelVisionResult;
    
    if (targetLabel) {
        targetLabel->setText(msg);
        // 根据 code 变色，例如 code==-1 时变红警示
        if (code == -1) targetLabel->setStyleSheet("color: red; font-weight: bold;");
        else targetLabel->setStyleSheet("color: white;");
    }
});

```

### 总结

这就是经典的 **MVC (Model-View-Controller)** 思想在跨语言机器人开发中的应用：

* **Model (Python 后端)**：只管处理业务逻辑和运算，不知道界面长啥样。
* **View (Qt UI)**：只管显示按钮和文本，不知道底层怎么通信。
* **Controller (RosWorker)**：在中间用 JSON 把两者黏合起来。未来加任何新功能，几乎都只需要改 Python 的逻辑和前端触发的代码，中间链路稳如磐石。