
Copyright(c) 2023, Stogl Robotics Consulting UG(haftungsbeschränkt)

Licensed under the Apache License, Version 2.0(the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

#include <custom_interactive_markers/mt_rviz_ui.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

// headers related to the interactive markers used in RViz
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

// Qt libraries (GUI)
#include <QVBoxLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QMessageBox> // Include for error messages

namespace custom_interactive_markers
{
    MTRvizUI::MTRvizUI(QWidget *parent)
        : rviz_common::Panel(parent),
          node_(std::make_shared<rclcpp::Node>("mt_rviz_ui_node")),
          tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(node_)),
          //// create an interactive marker server
          interactive_marker_server_(std::make_shared<interactive_markers::InteractiveMarkerServer>("mt_rviz_ui_server", node_))
    {

        RCLCPP_DEBUG(node_->get_logger(), "Initializing MTRvizUI...");
// Layout to organize UI components
QVBoxLayout *layout = new QVBoxLayout;

// Frame name and parent frame UI
QLabel *frame_name_label = new QLabel("Frame Name:", this);
layout->addWidget(frame_name_label);
frame_name_input_ = new QLineEdit(this);
frame_name_input_->setPlaceholderText("Enter frame name");
layout->addWidget(frame_name_input_);

QLabel *parent_frame_label = new QLabel("Parent Frame:", this);
layout->addWidget(parent_frame_label);
parent_frame_input_ = new QLineEdit(this);
parent_frame_input_->setPlaceholderText("Enter parent frame name");
layout->addWidget(parent_frame_input_);

// Cylinder dimensions input between 0.1 to 10
QLabel *radius_label = new QLabel("Cylinder Radius:", this);
layout->addWidget(radius_label);
radius_input_ = new QDoubleSpinBox(this);
radius_input_->setRange(0.1, 10.0); // Minimum radius should be positive
layout->addWidget(radius_input_);

QLabel *height_label = new QLabel("Cylinder Height:", this);
layout->addWidget(height_label);
height_input_ = new QDoubleSpinBox(this);
height_input_->setRange(0.1, 10.0); // Minimum height should be positive
layout->addWidget(height_input_);

// Publish button
publish_button_ = new QPushButton("Publish Frame", this);
layout->addWidget(publish_button_);
connect(publish_button_, &QPushButton::clicked, this, &MTRvizUI::publishFrame);

// Set the layout for the panel
setLayout(layout);

// // Create the interactive markers grid
// createInteractiveMarkers();
}

void MTRvizUI::onInitialize()
{
    rviz_common::Panel::onInitialize();
    RCLCPP_INFO(node_->get_logger(), "MTRvizUI plugin initialized.");
    createInteractiveMarkers();
}
void MTRvizUI::createInteractiveMarkers()
{
    RCLCPP_DEBUG(node_->get_logger(), "Creating interactive markers...");
    for (int i = 0; i < 5; ++i)
    {
        for (int j = 0; j < 10; ++j)
        {
            // Unique marker name
            std::string marker_name = "box_" + std::to_string(i) + "_" + std::to_string(j);
            RCLCPP_DEBUG(node_->get_logger(), "Creating marker: %s", marker_name.c_str());

            // Interactive marker
            visualization_msgs::msg::InteractiveMarker int_marker;
            int_marker.header.frame_id = "world";
            int_marker.name = marker_name;
            int_marker.pose.position.x = i * 2.0; // Grid X spacing
            int_marker.pose.position.y = j * 2.0; // Grid Y spacing
            int_marker.pose.position.z = 0.5;     // Place boxes on the floor
            int_marker.scale = 1.0;

            // Control for small box
            visualization_msgs::msg::InteractiveMarkerControl control;
            control.name = "small_box";
            control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
            control.always_visible = true;

            // Create a small box marker
            visualization_msgs::msg::Marker box_marker;
            box_marker.type = visualization_msgs::msg::Marker::CUBE;
            box_marker.scale.x = 1.0; // Box size
            box_marker.scale.y = 1.0;
            box_marker.scale.z = 1.0;
            box_marker.color.r = 0.5f;
            box_marker.color.g = 0.5f;
            box_marker.color.b = 0.5f;
            box_marker.color.a = 1.0f;
            control.markers.push_back(box_marker);

            // Add control to the interactive marker
            int_marker.controls.push_back(control);

            // Create arrows to visualize direction (X and Y axes)
            visualization_msgs::msg::Marker arrow_x = createArrowMarker(1.0, 0.0, 0.0, "arrow_x_" + marker_name); // X direction
            visualization_msgs::msg::Marker arrow_y = createArrowMarker(0.0, 1.0, 0.0, "arrow_y_" + marker_name); // Y direction

            // Add arrows to controls
            visualization_msgs::msg::InteractiveMarkerControl arrow_control_x;
            arrow_control_x.name = "arrow_control_x";
            arrow_control_x.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
            arrow_control_x.always_visible = true;
            arrow_control_x.markers.push_back(arrow_x);
            int_marker.controls.push_back(arrow_control_x);

            visualization_msgs::msg::InteractiveMarkerControl arrow_control_y;
            arrow_control_y.name = "arrow_control_y";
            arrow_control_y.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
            arrow_control_y.always_visible = true;
            arrow_control_y.markers.push_back(arrow_y);
            int_marker.controls.push_back(arrow_control_y);

            // Insert the marker into the server
            RCLCPP_INFO(node_->get_logger(), "Inserting marker: %s", marker_name.c_str());
            interactive_marker_server_->insert(int_marker);
        }
    }

    // Apply changes to the interactive marker server
    RCLCPP_INFO(node_->get_logger(), "Marker inserted, applying changes...");
    interactive_marker_server_->applyChanges();
    RCLCPP_INFO(node_->get_logger(), "Interactive markers created and applied.");

    // Subscribe to the feedback topic
    feedback_subscription_ = node_->create_subscription<visualization_msgs::msg::InteractiveMarkerFeedback>(
        "/mt_rviz_ui_server/feedback", 10, std::bind(&MTRvizUI::processFeedback, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "Feedback subscription created: /mt_rviz_ui_server/feedback");
}

visualization_msgs::msg::Marker MTRvizUI::createArrowMarker(double x_scale, double y_scale, double z_scale, const std::string &marker_name)
{
    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.scale.x = x_scale;
    arrow_marker.scale.y = y_scale;
    arrow_marker.scale.z = z_scale;

    // Set color of the arrow
    arrow_marker.color.r = 1.0f; // Set to red or any other color
    arrow_marker.color.g = 0.0f;
    arrow_marker.color.b = 0.0f;
    arrow_marker.color.a = 1.0f;

    arrow_marker.ns = marker_name; // Assign the marker name
    return arrow_marker;
}

void MTRvizUI::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr feedback)
{
    RCLCPP_INFO(node_->get_logger(), "Received feedback for marker: %s", feedback->marker_name.c_str());

    // Print feedback details for debugging
    RCLCPP_INFO(node_->get_logger(), "Interaction type: %d", feedback->event_type);
    RCLCPP_INFO(node_->get_logger(), "Marker position: (%f, %f, %f)",
                feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);

    // Update arrows position to reflect the new marker position
    updateArrows(feedback->marker_name, feedback->pose.position.x, feedback->pose.position.y);
}
void MTRvizUI::updateArrows(const std::string &marker_name, double x_position, double y_position)
{
    // Create an arrow marker.
    visualization_msgs::msg::Marker arrow_marker = createArrowMarker(1.0, 0.2, 0.2, marker_name);

    // Create an interactive marker for the arrow.
    interactive_markers::InteractiveMarker arrow_marker_interactive;
    arrow_marker_interactive.header.frame_id = "base_link"; // Or the frame you want the arrow in
    arrow_marker_interactive.name = marker_name;
    arrow_marker_interactive.description = "Arrow Marker";

    // Create an InteractiveMarkerControl and add the arrow marker to it.
    interactive_markers::InteractiveMarkerControl control;
    control.orientation.w = 1.0;
    control.orientation.x = 0.0;
    control.orientation.y = 0.0;
    control.orientation.z = 0.0;
    control.name = "arrow_control";
    control.interaction_mode = interactive_markers::InteractiveMarkerControl::MOVE_ROTATE_3D;
    control.markers.push_back(arrow_marker);

    // Add the control to the interactive marker.
    arrow_marker_interactive.controls.push_back(control);

    // Insert the interactive marker into the interactive marker server.
    interactive_marker_server_->insert(arrow_marker_interactive);

    // You may want to call `applyChanges()` to update the server immediately.
    interactive_marker_server_->applyChanges();
}

void MTRvizUI::publishFrame()
{
    RCLCPP_INFO(node_->get_logger(), "Publish button pressed.");
    std::string frame_name = frame_name_input_->text().toStdString();
    std::string parent_frame = parent_frame_input_->text().toStdString();

    // Validate inputs
    if (frame_name.empty() || parent_frame.empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "Frame name or parent frame cannot be empty.");
        QMessageBox::warning(this, "Input Error", "Frame name or parent frame cannot be empty.");
        return;
    }
    RCLCPP_DEBUG(node_->get_logger(), "Publishing frame: %s under parent: %s", frame_name.c_str(), parent_frame.c_str());
    // Publish the transformation between the frames
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = node_->now();
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = frame_name;
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(transform);
    RCLCPP_INFO(node_->get_logger(), "Published frame \"%s\" under parent frame \"%s\".",
                frame_name.c_str(), parent_frame.c_str());

    // Add cylinders as interactive markers
    RCLCPP_INFO(node_->get_logger(), "Adding cylinders to interactive markers.");
    addCylinders();
}
void MTRvizUI::addCylinders()
{
    double radius = radius_input_->value();
    double height = height_input_->value();

    // Validate input
    if (radius <= 0.0 || height <= 0.0)
    {
        QMessageBox::warning(this, "Invalid Input", "Radius and Height must be positive values.");
        RCLCPP_ERROR(node_->get_logger(), "Invalid radius or height: radius=%.2f, height=%.2f", radius, height);
        return;
    }
    RCLCPP_INFO(node_->get_logger(), "Adding cylinders with radius=%.2f and height=%.2f", radius, height);

    // Loop through the grid and replace boxes with cylinders
    for (int i = 0; i < 5; ++i)
    {
        for (int j = 0; j < 10; ++j)
        {
            // Unique marker name
            std::string marker_name = "cylinder_" + std::to_string(i) + "_" + std::to_string(j);
            RCLCPP_DEBUG(node_->get_logger(), "Replacing marker: %s with cylinder.", marker_name.c_str());

            // Create a new interactive marker
            visualization_msgs::msg::InteractiveMarker int_marker;
            int_marker.header.frame_id = "world";
            int_marker.name = marker_name;
            int_marker.pose.position.x = i * 2.0;
            int_marker.pose.position.y = j * 2.0;
            int_marker.pose.position.z = 0.5;
            int_marker.scale = 1.0;

            // Create the cylinder marker
            visualization_msgs::msg::Marker cylinder_marker = createCylinderMarker(radius, height);

            // Add control for the cylinder
            visualization_msgs::msg::InteractiveMarkerControl control;
            control.name = "cylinder_control";
            control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
            control.always_visible = true;
            control.markers.push_back(cylinder_marker);

            // Attach the marker to the control
            int_marker.controls.push_back(control);

            // // Remove the old marker if it exists
            // RCLCPP_INFO(node_->get_logger(), "Inserting marker erased: %s", marker_name.c_str());
            // interactive_marker_server_->erase(marker_name);

            // Insert the updated interactive marker into the server
            RCLCPP_INFO(node_->get_logger(), "Inserting cylinder marker: %s", marker_name.c_str());
            interactive_marker_server_->insert(int_marker);
            RCLCPP_INFO(node_->get_logger(), "Inserted cylinder marker: %s", int_marker.name.c_str());

            // Broadcast a unique frame for the cylinder
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = node_->now();
            transform.header.frame_id = "world";
            transform.child_frame_id = marker_name;
            transform.transform.translation.x = i * 2.0;
            transform.transform.translation.y = j * 2.0;
            transform.transform.translation.z = height / 2.0;
            transform.transform.rotation.x = 0.0;
            transform.transform.rotation.y = 0.0;
            transform.transform.rotation.z = 0.0;
            transform.transform.rotation.w = 1.0;

            tf_broadcaster_->sendTransform(transform);
        }
    }
    // Apply changes to the interactive marker server
    interactive_marker_server_->applyChanges();
    RCLCPP_INFO(node_->get_logger(), "Cylinders added and server updated.");
}

visualization_msgs::msg::Marker MTRvizUI::createCylinderMarker(double radius, double height)
{
    visualization_msgs::msg::Marker cylinder_marker;
    cylinder_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    cylinder_marker.scale.x = radius * 2; // Diameter
    cylinder_marker.scale.y = radius * 2;
    cylinder_marker.scale.z = height;

    // Set color of the cylinder
    cylinder_marker.color.r = 1.0f;
    cylinder_marker.color.g = 0.0f;
    cylinder_marker.color.b = 0.0f;
    cylinder_marker.color.a = 1.0f;

    return cylinder_marker;
}
}
; // namespace custom_interactive_markers

// Register the RViz plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(custom_interactive_markers::MTRvizUI, rviz_common::Panel)