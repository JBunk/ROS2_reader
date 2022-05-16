#include <inttypes.h>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "FibActionClient.hpp"
#include "custom_interfaces/action/fibonacci.hpp"

using namespace std::placeholders; // needed for the _1 and _2 in std::bind

FibActionClient::FibActionClient(const rclcpp::NodeOptions & node_options)
: Node("fib_action_client", node_options), goal_done_(false)
{
    this->client_ptr_ = rclcpp_action::create_client<FibAction>(
      this,
      "fibonacci");

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&FibActionClient::send_goal, this));
}

bool FibActionClient::is_goal_done() const
{
    return this->goal_done_;
}

void FibActionClient::send_goal()
{
    // cancel the timer.
    this->timer_->cancel();

    // the goal is not done.
    this->goal_done_ = false;

    // wait till the action server is also running.
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        this->goal_done_ = true;
        return;
    }

    // create a new goal message:
    auto goal_msg = FibAction::Goal();
    goal_msg.order = 10;

    // print in logger
    RCLCPP_INFO(this->get_logger(), "Sending goal");
    
    // setting the functions that handle the return-messages:
    auto send_goal_options = rclcpp_action::Client<FibAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&FibActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&FibActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&FibActionClient::result_callback, this, _1);

    // sending the goal:
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}



void FibActionClient::goal_response_callback(std::shared_future<GoalHandleFib::SharedPtr> future)
{
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void FibActionClient::feedback_callback(
    GoalHandleFib::SharedPtr,
    const std::shared_ptr<const FibAction::Feedback> feedback
    ){
    RCLCPP_INFO(
        this->get_logger(), 
        "Next number in sequence received: %" PRId32,
        feedback->partial_sequence.back()
    );
}

void FibActionClient::result_callback(const GoalHandleFib::WrappedResult & result)
{
    this->goal_done_ = true;
    // result.code contains information why the result is send.
    // It is possible to get an incomplete result, for example when the
    // goal is canceled.
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
    }   

    RCLCPP_INFO(this->get_logger(), "Result received");
    for (auto number : result.result->sequence) {
        RCLCPP_INFO(this->get_logger(), "%" PRId32, number);
    }
}
