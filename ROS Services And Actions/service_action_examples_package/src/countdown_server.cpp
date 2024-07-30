#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <service_action_examples_package/CountdownAction.h>
#include <unistd.h>

void executeCallback(const service_action_examples_package::CountdownGoalConstPtr &goal, 
                     actionlib::SimpleActionServer<service_action_examples_package::CountdownAction> *as) {
    ros::Rate r(1);
    service_action_examples_package::CountdownFeedback feedback;
    feedback.current_num = goal->starting_num;

    while (feedback.current_num > 0) {
        // Publish feedback
        as->publishFeedback(feedback);

        // Print log messages
        ROS_INFO("Feedback: %d", feedback.current_num);

        // Decrement the feedback message's current_num
        feedback.current_num -= 1;

        // Wait a second before counting down to the next number
        r.sleep();
    }

    // Final feedback message
    as->publishFeedback(feedback);
    ROS_INFO("Feedback: %d", feedback.current_num);
    ROS_INFO("Done!");

    service_action_examples_package::CountdownResult result;
    result.is_finished = true;
    // Indicate that the goal was successful
    as->setSucceeded(result);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "countdown_server_cpp");
    ros::NodeHandle nh;

    actionlib::SimpleActionServer<service_action_examples_package::CountdownAction> server(nh, "countdown", 
            boost::bind(&executeCallback, _1, &server), false);

    server.start();
    ROS_INFO("Starting Action Server cpp");
    ros::spin();

    return 0;
}
