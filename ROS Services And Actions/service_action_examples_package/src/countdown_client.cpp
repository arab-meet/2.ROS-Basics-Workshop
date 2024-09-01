#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <service_action_examples_package/CountdownAction.h>

// This callback function will be called when feedback is received from the action server
void feedbackCallback(const service_action_examples_package::CountdownFeedbackConstPtr& feedback) {
    ROS_INFO("Feedback: %d", feedback->current_num);
}

// This callback function will be called when the action becomes active
void activeCallback() {
    ROS_INFO("Goal just went active");
}

// This callback function will be called when the action is done
void doneCallback(const actionlib::SimpleClientGoalState& state,
                  const service_action_examples_package::CountdownResultConstPtr& result) {
    ROS_INFO("Action finished: %s", state.toString().c_str());
    ROS_INFO("Result: %d", result->is_finished);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "countdown_client_cpp");

    if (argc != 2) {
        ROS_INFO("Usage: countdown_client <starting_num>");
        return 1;
    }

    int starting_num = atoi(argv[1]);

    actionlib::SimpleActionClient<service_action_examples_package::CountdownAction> client("countdown", true);

    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();

    service_action_examples_package::CountdownGoal goal;
    goal.starting_num = starting_num;

    ROS_INFO("Sending goal: %d", starting_num);
    client.sendGoal(goal, &doneCallback, &activeCallback, &feedbackCallback);

    // Wait for the action to return
    bool finished_before_timeout = client.waitForResult(ros::Duration(30.0));

    if (!finished_before_timeout) {
        ROS_INFO("Action did not finish before the timeout.");
    }

    return 0;
}
