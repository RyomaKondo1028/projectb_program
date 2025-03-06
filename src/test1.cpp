#include <thread>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include "ros2_robotiqgripper/srv/robotiq_gripper.hpp" // グリッパーのサービス型をインクルード
// グリッパー操作サーバーのクラスを追加
class GripperServer : public rclcpp::Node {
public:
    GripperServer() : Node("gripper_server") {
        // サービスサーバーの作成
        service_ = this->create_service<ros2_robotiqgripper::srv::RobotiqGripper>(
            "/Robotiq_Gripper", std::bind(&GripperServer::handle_gripper_request,
            this, std::placeholders::_1, std::placeholders::_2));
    }
private:
    // グリッパーの実際の制御関数
    bool control_actual_gripper(const std::string& action, float position, float force) {
        RCLCPP_INFO(this->get_logger(), "グリッパーを%s します。位置=%.2f, 力=%.2f", action == "OPEN" ? "開き" : "閉じ", position, force);
        // ここで実際のハードウェアやシミュレーション API へのコマンドを送信する。
        // 以下は仮の動作として、成功を常に返します。
        if (action == "OPEN") {
            // グリッパーを開く処理（ハードウェアに応じて実装）
            RCLCPP_INFO(this->get_logger(), "グリッパーを開いています...");
        } else if (action == "CLOSE") {
            // グリッパーを閉じる処理（ハードウェアに応じて実装）
            RCLCPP_INFO(this->get_logger(), "グリッパーを閉じています...");
        } else {
            RCLCPP_ERROR(this->get_logger(), "不明なアクション: %s", action.c_str());
            return false;
        }
        // 仮の成功判定
        return true;
    }
    // グリッパー操作のリクエストを処理
    void handle_gripper_request(
        const std::shared_ptr<ros2_robotiqgripper::srv::RobotiqGripper::Request> request,
        std::shared_ptr<ros2_robotiqgripper::srv::RobotiqGripper::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Received request: action=%s, force=%.2f, position=%.2f", request->action.c_str(), request->force, request->position);
        // グリッパーの制御処理を呼び出し
        bool result = control_actual_gripper(request->action, request->position, request->force);
        // 成功したかどうかをレスポンスとして返す
        response->success = result;
        response->value = request->position; // 状態に応じた値（仮に 0 を返しています）
        response->average = request->force; // 状態に応じた平均値（仮に 0.0 を返しています）
        response->message = result ? "グリッパー操作が成功しました" : "グリッパー操作が失敗しました";
    }

    rclcpp::Service<ros2_robotiqgripper::srv::RobotiqGripper>::SharedPtr service_;
};
void control_gripper(const std::shared_ptr<rclcpp::Node>& node, const std::string& action, float position = 1.0f, float force = 1.0f) {
    auto const logger = rclcpp::get_logger("control_gripper");
    // グリッパー操作のためのサービスクライアントを作成
    auto client = node->create_client<ros2_robotiqgripper::srv::RobotiqGripper>("/Robotiq_Gripper");
    // サービスが利用可能になるまで待機
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger, "サービスが終了しました。");
            return;
        }
        RCLCPP_INFO(logger, "サービスの準備を待機中...");
    }
    // リクエストの作成（グリッパーの開閉）
    auto request = std::make_shared<ros2_robotiqgripper::srv::RobotiqGripper::Request>();
    request->action = action;
    request->position = position; // 開閉位置を設定（0.0-1.0）
    request->force = force; // 力加減を設定（0.0-1.0）
    // サービスにリクエストを送信
    auto future = client->async_send_request(request);
    // サービスの応答を待機
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(logger, "グリッパーが%s ました。", action == "OPEN" ? "開き" : "閉じ");
        } else {
            RCLCPP_ERROR(logger, "グリッパーを%s のに失敗しました。", action == "OPEN" ? "開く" : "閉じる");
        }
    } else {
        RCLCPP_ERROR(logger, "サービスコールに失敗しました。");
    }
}
int main(int argc, char * argv[]) {
    // ROS の初期化とノード作成
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    // GripperServer のインスタンスを作成してサーバーを動作させる
    auto gripper_server = std::make_shared<GripperServer>();
    // ロガーの作成
    auto logger = rclcpp::get_logger("hello_moveit");
    // MoveGroupInterface の作成
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group_interface(node, "ur_manipulator");
    /*
    // グリッパーを開く
    RCLCPP_INFO(logger, "グリッパーを開きます。");
    control_gripper(node, "OPEN",0.0f,1.0f);
    */
    // 初期位置の設定（各ジョイント角度を指定）
    std::vector<double> initial_position = {-1.57, -1.57, -1.57, 0.0, 1.57, 0.0};
    move_group_interface.setJointValueTarget(initial_position);
    // 初期位置への移動を実行
    auto move_to_initial_position = [&]() -> bool {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group_interface.plan(plan));
        if (success) {
            move_group_interface.execute(plan);
        }
        return success;
    };
    if (!move_to_initial_position()) {
        RCLCPP_ERROR(logger, "Failed to move to the initial position.");
        rclcpp::shutdown();
        return 1;
    }
    /*
    // グリッパーを開く（0.0 - 1.0 の範囲で指定）
    RCLCPP_INFO(logger, "グリッパーを開きます。");
    control_gripper(node, "OPEN", 0.0f, 0.5f); // 0.0 は開いた位置
    */
    // ウェイポイントリスト作成
    std::vector<geometry_msgs::msg::Pose> waypoints;
    // 最初のポーズを設定
    geometry_msgs::msg::Pose pose1;
    pose1.position.x = 0.28;
    pose1.position.y = 0.2;
    pose1.position.z = 0.4;
    tf2::Quaternion quaternion1;
    quaternion1.setRPY(M_PI, 0, 0); // エンドエフェクタを下向きに設定
    pose1.orientation.x = quaternion1.x();
    pose1.orientation.y = quaternion1.y();
    pose1.orientation.z = quaternion1.z();
    pose1.orientation.w = quaternion1.w();
    waypoints.push_back(pose1);

    geometry_msgs::msg::Pose pose2 = pose1;
    pose2.position.z -= 0.15;
    waypoints.push_back(pose2);
    moveit_msgs::msg::RobotTrajectory trajectory;
    move_group_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;
    move_group_interface.execute(cartesian_plan);
    /*
    // グリッパーのは場を42.5[mm]（0.0 - 1.0 の範囲で指定）
    RCLCPP_INFO(logger, "グリッパーの幅を42.5[mm]に変更。");
    control_gripper(node, "CLOSE", 0.5f, 0.1f);
    */
    std::vector<geometry_msgs::msg::Pose> waypoints2;
    // 新しいポーズ設定
    geometry_msgs::msg::Pose pose3 = pose2;
    pose3.position.z += 0.16;
    waypoints2.push_back(pose3);
    geometry_msgs::msg::Pose pose4 = pose3;
    pose4.position.x -= 0.56;
    pose4.position.y += 0.15;
    waypoints2.push_back(pose4);
    geometry_msgs::msg::Pose pose5 = pose4;
    pose5.position.z -= 0.195;
    waypoints2.push_back(pose5);
    move_group_interface.computeCartesianPath(waypoints2, 0.01, 0.0, trajectory);
    cartesian_plan.trajectory_ = trajectory;
    move_group_interface.execute(cartesian_plan);
    /*
    // グリッパーを開く
    RCLCPP_INFO(logger, "グリッパーを開きます。");
    control_gripper(node, "OPEN",0.0f,1.0f);       
    */
    // ROS の終了
    rclcpp::shutdown();
    return 0;
}








