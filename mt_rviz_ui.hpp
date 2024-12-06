// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CUSTOM_INTERACTIVE_MARKERS__MT_RVIZ_UI_HPP_
#define CUSTOM_INTERACTIVE_MARKERS__MT_RVIZ_UI_HPP_

#include <memory>
#include <QLineEdit>                                         // For text input fields in the GUI.
#include <QPushButton>                                       // For buttons in the GUI.
#include <QVBoxLayout>                                       // For organizing layout.
#include <QDoubleSpinBox>                                    // For number input (radius, height).
#include <QMessageBox>                                       // For displaying error messages.
#include <rviz_common/panel.hpp>                             // Base class for creating a panel in RViz.
#include <tf2_ros/transform_broadcaster.h>                   // Sends transformation data in ROS.
#include <rclcpp/rclcpp.hpp>                                 // ROS2 node operation.
#include <visualization_msgs/msg/marker.hpp>                 // For creating marker types like cylinders.
#include <interactive_markers/interactive_marker_server.hpp> // For interactive markers in RViz

namespace custom_interactive_markers
{
    class MTRvizUI : public rviz_common::Panel
    {
        Q_OBJECT // A special macro that allows the class to use Qt's signal-slot system (for buttons and events).

            public : MTRvizUI(QWidget *parent = nullptr); // Constructor: sets up the panel.
        virtual void onInitialize() override;             // Called when the panel is initialized.

    private Q_SLOTS:
        void publishFrame(); // Publishes the transform for the entered frame name.

    private:
        // ROS 2 node for communication.
        rclcpp::Node::SharedPtr node_;
        // Transform broadcaster for publishing frames.
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        // Interactive marker server for adding markers.
        std::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
        rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr feedback_subscription_; // Declare the subscription

        // Text input fields for frame name and parent frame.
        QLineEdit *frame_name_input_;
        QLineEdit *parent_frame_input_;

        // Input fields for cylinder dimensions (radius and height).
        QDoubleSpinBox *radius_input_;
        QDoubleSpinBox *height_input_;

        // Publish button.
        QPushButton *publish_button_;

        // Function to create interactive markers.
        void createInteractiveMarkers();
        // Function to add cylinders as interactive markers.
        void addCylinders();
        // Function to create a cylinder marker.
        visualization_msgs::msg::Marker createCylinderMarker(double radius, double height);

        // Function to handle input validation.
        bool validateInputs();
        void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr feedback);
        // void processFeedback(std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> feedback);

        // new below delete later
        // bool isValidFrameName(const std::string &frame_name); // Declaration of the new function
        // Function to update or remove existing interactive markers.
        void updateInteractiveMarker();
        void removeInteractiveMarker();
    };
} // namespace custom_interactive_markers

#endif // CUSTOM_INTERACTIVE_MARKERS__MT_RVIZ_UI_HPP_
