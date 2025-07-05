// #include <memory>
// #include <chrono>
// #include <cmath>

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "tf2/LinearMath/Quaternion.h"

// #include "goal_msgs/srv/goal.hpp" // 커스텀 서비스

// #include "std_msgs/msg/string.hpp" // web 노드

// using namespace std::chrono_literals;

// class GoalTriggerNode : public rclcpp::Node
// {
// public:
//   using NavigateToPose = nav2_msgs::action::NavigateToPose;
//   using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
//   using GoalService = goal_msgs::srv::Goal;

//   GoalTriggerNode() : Node("goal_trigger_node")
//   {
//     // 액션 클라이언트 초기화
//     action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

//     // 커스텀 서비스 서버 생성
//     service_ = this->create_service<GoalService>(
//         "/trigger_goal",
//         std::bind(&GoalTriggerNode::service_callback, this, std::placeholders::_1, std::placeholders::_2));

//     // web통신 위한 퍼블리셔 생성
//     publisher_ = this->create_publisher<std_msgs::msg::String>("web", 10);
//     timer_ = this->create_wall_timer(
//         500ms, std::bind(&GoalTriggerNode::timer_callback, this));
//   }

// private:
//   // bool count_ = false;

//   rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

//   // 커스텀 서비스
//   rclcpp::Service<GoalService>::SharedPtr service_;

//   // web통신 퍼블리셔
//   rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

//   bool pub_ = false;

//   void timer_callback()
//   {
//     auto message = std_msgs::msg::String();
//     message.data = "도착!";
//     if (pub_ == true)
//     {
//       publisher_->publish(message);
//     }
//   }

//   void service_callback(
//       const std::shared_ptr<GoalService::Request> request,
//       std::shared_ptr<GoalService::Response> response)
//   {
//     RCLCPP_INFO(this->get_logger(), "📩 서비스 요청 수신: data = %ld", request->data);

//     if (request->data == 1)
//     {
//       // if (count_ == false)
//       // {
//         send_goal();
//         response->result = true;
//       // }
//     }
//     else
//     {
//       RCLCPP_WARN(this->get_logger(), "❌ 알 수 없는 명령: %ld", request->data);
//       response->result = false;
//     }
//   }

//   void send_goal()
//   {
//     if (!action_client_->wait_for_action_server(5s))
//     {
//       RCLCPP_ERROR(this->get_logger(), "action server 오류");
//       return;
//     }

//     auto goal_msg = NavigateToPose::Goal();
//     goal_msg.pose.header.frame_id = "map";
//     goal_msg.pose.header.stamp = now();

//     // ###############
//     // goal 위치 설정
//     // x, y, z 설정
//     goal_msg.pose.pose.position.x = 19.208;
//     goal_msg.pose.pose.position.y = 2.405;
//     goal_msg.pose.pose.position.z = 0;

//     //5공 지하 엘베: 19.208, 2.405, 180
//     //1공 1층 정수기 : 19.463, 16,490

//     // roll, pitch, yaw 설정
//     double roll = 0.0, pitch = 0.0, yaw_deg = 180.0;
//     double yaw = yaw_deg * M_PI / 180.0;

//     tf2::Quaternion q;
//     q.setRPY(roll, pitch, yaw);
//     goal_msg.pose.pose.orientation.x = q.x();
//     goal_msg.pose.pose.orientation.y = q.y();
//     goal_msg.pose.pose.orientation.z = q.z();
//     goal_msg.pose.pose.orientation.w = q.w();
//     // ###############

//     auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
//     send_goal_options.result_callback = std::bind(&GoalTriggerNode::result_callback, this, std::placeholders::_1);

//     action_client_->async_send_goal(goal_msg, send_goal_options);
//   }

//   void result_callback(const GoalHandleNavigateToPose::WrappedResult &result)
//   {
//     if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
//     {
//       // count_ = true;
//       // 도착 했을 때 여기

//       pub_ = true;
//       RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
//     }
//     else
//     {
//       RCLCPP_WARN(this->get_logger(), "⚠ Goal failed or was canceled.");
//     }
//   }
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<GoalTriggerNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }

#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int64.hpp" // 👉 토픽 수신용

using namespace std::chrono_literals;

class GoalTriggerNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  GoalTriggerNode() : Node("goal_trigger_node")
  {
    // 액션 클라이언트 초기화
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // 웹 통신 퍼블리셔 생성
    publisher_ = this->create_publisher<std_msgs::msg::String>("web", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&GoalTriggerNode::timer_callback, this));

    // goal trigger 수신 구독자 생성
    subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
        "trigger_goal", 10,
        std::bind(&GoalTriggerNode::trigger_callback, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

  // 웹 통신용
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_;

  bool pub_ = false;

  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "도착!";
    if (pub_)
    {
      publisher_->publish(message);
      pub_ = false;
    }
  }

  void trigger_callback(const std_msgs::msg::Int64::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "📩 토픽 수신: data = %ld", msg->data);
    // 0
    // 원점
    if (msg->data == 0)
    {
      auto goal_msg = NavigateToPose::Goal();
      goal_msg.pose.header.frame_id = "map";
      goal_msg.pose.header.stamp = now();
      // 목표 위치 설정
      goal_msg.pose.pose.position.x = 0.0;
      goal_msg.pose.pose.position.y = 0.0;
      goal_msg.pose.pose.position.z = 0.0;

      double yaw_deg = 0;
      double yaw = yaw_deg * M_PI / 180.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      goal_msg.pose.pose.orientation.x = q.x();
      goal_msg.pose.pose.orientation.y = q.y();
      goal_msg.pose.pose.orientation.z = q.z();
      goal_msg.pose.pose.orientation.w = q.w();

      auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
      send_goal_options.result_callback = std::bind(&GoalTriggerNode::result_callback, this, std::placeholders::_1);

      action_client_->async_send_goal(goal_msg, send_goal_options);
    }
    // 1
    // 엘베
    else if (msg->data == 1)
    {
      auto goal_msg = NavigateToPose::Goal();
      goal_msg.pose.header.frame_id = "map";
      goal_msg.pose.header.stamp = now();

      // 목표 위치 설정
      // 5공지하엘베
      // goal_msg.pose.pose.position.x = 19.208;
      // goal_msg.pose.pose.position.y = 2.405;
      // goal_msg.pose.pose.position.z = 0.0;
      //5공1층자판기
      // goal_msg.pose.pose.position.x = 9.798;
      // goal_msg.pose.pose.position.y = 36.389;
      // goal_msg.pose.pose.position.z = 0.0;

      
      // 시연 102호
        
      goal_msg.pose.pose.position.x = 5.000;
      goal_msg.pose.pose.position.y = -2.355;
      goal_msg.pose.pose.position.z = 0.0;
       

      double yaw_deg = 180;
      double yaw = yaw_deg * M_PI / 180.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      goal_msg.pose.pose.orientation.x = q.x();
      goal_msg.pose.pose.orientation.y = q.y();
      goal_msg.pose.pose.orientation.z = q.z();
      goal_msg.pose.pose.orientation.w = q.w();

      auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
      send_goal_options.result_callback = std::bind(&GoalTriggerNode::result_callback, this, std::placeholders::_1);

      action_client_->async_send_goal(goal_msg, send_goal_options);
    }
    // 2
    // 사물함
    else if (msg->data == 3)
    {
      auto goal_msg = NavigateToPose::Goal();
      goal_msg.pose.header.frame_id = "map";
      goal_msg.pose.header.stamp = now();

      // 목표 위치 설정
      // goal_msg.pose.pose.position.x = 43.528;
      // goal_msg.pose.pose.position.y = 2.757;
      // goal_msg.pose.pose.position.z = 0.0;

      goal_msg.pose.pose.position.x = 4.000;
      goal_msg.pose.pose.position.y =  0.243;
      goal_msg.pose.pose.position.z = 0;      
        

      double yaw_deg = -90;
      double yaw = yaw_deg * M_PI / 180.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      goal_msg.pose.pose.orientation.x = q.x();
      goal_msg.pose.pose.orientation.y = q.y();
      goal_msg.pose.pose.orientation.z = q.z();
      goal_msg.pose.pose.orientation.w = q.w();

      auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
      send_goal_options.result_callback = std::bind(&GoalTriggerNode::result_callback, this, std::placeholders::_1);

      action_client_->async_send_goal(goal_msg, send_goal_options);
    }
    // 3
    // 화장실
    else if (msg->data == 4)
    {
      auto goal_msg = NavigateToPose::Goal();
      goal_msg.pose.header.frame_id = "map";
      goal_msg.pose.header.stamp = now();

      // 목표 위치 설정
      // goal_msg.pose.pose.position.x = 12.674;
      // goal_msg.pose.pose.position.y = 4.797;
      // goal_msg.pose.pose.position.z = 0.0;
      goal_msg.pose.pose.position.x = 1.184;
      goal_msg.pose.pose.position.y =  -2.766;
      goal_msg.pose.pose.position.z = -1.543;

      double yaw_deg = 90;
      double yaw = yaw_deg * M_PI / 180.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      goal_msg.pose.pose.orientation.x = q.x();
      goal_msg.pose.pose.orientation.y = q.y();
      goal_msg.pose.pose.orientation.z = q.z();
      goal_msg.pose.pose.orientation.w = q.w();

      auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
      send_goal_options.result_callback = std::bind(&GoalTriggerNode::result_callback, this, std::placeholders::_1);

      action_client_->async_send_goal(goal_msg, send_goal_options);
    }
    // 4
    // 창동 앞
    else if (msg->data == 2)
    {
      auto goal_msg = NavigateToPose::Goal();
      goal_msg.pose.header.frame_id = "map";
      goal_msg.pose.header.stamp = now();

      // 목표 위치 설정
      goal_msg.pose.pose.position.x = 28.110;
      goal_msg.pose.pose.position.y = 4.797;
      goal_msg.pose.pose.position.z = 0.0;

      double yaw_deg = 0;
      double yaw = yaw_deg * M_PI / 180.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      goal_msg.pose.pose.orientation.x = q.x();
      goal_msg.pose.pose.orientation.y = q.y();
      goal_msg.pose.pose.orientation.z = q.z();
      goal_msg.pose.pose.orientation.w = q.w();

      auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
      send_goal_options.result_callback = std::bind(&GoalTriggerNode::result_callback, this, std::placeholders::_1);

      action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    else
    {
      RCLCPP_WARN(this->get_logger(), "❌ 알 수 없는 명령: %ld", msg->data);
    }
  }

  void result_callback(const GoalHandleNavigateToPose::WrappedResult &result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      pub_ = true;
      RCLCPP_INFO(this->get_logger(), "✅ Goal succeeded!");
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "⚠ Goal failed or was canceled.");
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalTriggerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
