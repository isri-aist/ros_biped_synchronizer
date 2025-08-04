#include <cstddef>
#include <memory>
#include <chrono>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/synchronizer.h>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"


#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#define IMG_BUFFER_SIZE 45 // Size of the image buffer


class RosBipedSynchronizer : public rclcpp::Node
{
    public:
        RosBipedSynchronizer()
        : Node("ros_biped_synchronizer")
        {
            // Set QoS the same with the feet's force sensors for better synchronization later
            int qos_value = this->declare_parameter<int>("qos", 10);
            rclcpp::QoS qos(qos_value);
            // Get the force threshold from parameters, default is 270.0 (empirically found value for HRP5P)
            forceThreshold_ = this->declare_parameter<float>("force_threshold", 270.0f);
            float agePenalty = this->declare_parameter<float>("age_penalty", 0.01); // 0.01 seconds as the robot outputs at 200Hz
            maxFPS_ = this->declare_parameter<float>("max_fps", 30.0f); // Default max FPS is 30.0
            previousTrig_ = std::chrono::steady_clock::now();

            triggerPub_ = this->create_publisher<std_msgs::msg::Header>("/synchronizer/trigger", qos);

            #ifdef MANAGE_IMGS
            imagePub_ = this->create_publisher<sensor_msgs::msg::Image>("/synchronizer/image", qos);
            imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw", qos,
                std::bind(&RosBipedSynchronizer::imageCallback, this, std::placeholders::_1)
            );
            #endif

            leftFootWrenchSub_.subscribe(this, "/LeftFootForceSensor"); 
            rightFootWrenchSub_.subscribe(this, "/RightFootForceSensor");
            synchronizer_ = std::make_shared
                                <message_filters::Synchronizer
                                    <message_filters::sync_policies::ApproximateTime
                                        <geometry_msgs::msg::WrenchStamped, geometry_msgs::msg::WrenchStamped>
                                    >
                                >(10, leftFootWrenchSub_, rightFootWrenchSub_);

            synchronizer_->setAgePenalty(agePenalty);
            synchronizer_->registerCallback(std::bind(&RosBipedSynchronizer::syncCallback, this,
                                                      std::placeholders::_1, std::placeholders::_2));

            RCLCPP_INFO(this->get_logger(), "Synchronizer initialized with QoS %d and force threshold %.2f. Max FPS is set to %.0f ", qos_value, forceThreshold_, maxFPS_);
        }

    private:
        // Synchronization stuff
        message_filters::Subscriber<geometry_msgs::msg::WrenchStamped> leftFootWrenchSub_; // Subscribers for left foot force measurements
        message_filters::Subscriber<geometry_msgs::msg::WrenchStamped> rightFootWrenchSub_; // Subscribers for right foot force measurements

        std::shared_ptr
            <message_filters::Synchronizer
                <message_filters::sync_policies::ApproximateTime
                    <geometry_msgs::msg::WrenchStamped, geometry_msgs::msg::WrenchStamped>
                >
            > synchronizer_; // Synchronizer for the left and right foot force measurements

        #ifdef MANAGE_IMGS
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePub_;
        std::vector<sensor_msgs::msg::Image> imageBuffer_; // Buffer for storing images
        #endif

        rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr triggerPub_; // Publisher for the trigger message

        // Threshold for force synchronization, default is 270.0 (empirically found value for HRP5P).
        float forceThreshold_ = 270.0; 

        float maxFPS_ = 30.0;

        std::chrono::steady_clock::time_point previousTrig_ = std::chrono::steady_clock::now();
        
        
        /**
         * @brief Callback function to synchronize force measurements from both feet.
         *
         * This function is called with force data from the left and right foot sensors.
         * It extracts the force vectors from both messages and compares the z-axis force
         * components. If the absolute difference between the left and right z-axis forces
         * is below a predefined threshold (`forceThreshold_`), it publishes a trigger message
         * using the header from the left foot's message.
         *
         * @param leftWrench  Shared pointer to the left foot's WrenchStamped message.
         * @param rightWrench Shared pointer to the right foot's WrenchStamped message.
         */
        void syncCallback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr &leftWrench,
                          const geometry_msgs::msg::WrenchStamped::ConstSharedPtr &rightWrench)
        {
            // Extract force vectors from both leftWrench and rightWrench
            const geometry_msgs::msg::Vector3& leftForce = leftWrench->wrench.force;
            const geometry_msgs::msg::Vector3& rightForce = rightWrench->wrench.force;

            // if force on z-axis are equals on two feet (bellow a threshold), then publish the trigger
            if (std::abs(leftForce.z - rightForce.z) < forceThreshold_)
            {
                auto durationSinceLastTrig = std::chrono::steady_clock::now() - previousTrig_;
                auto durationInSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(durationSinceLastTrig).count() / 1000.0;
                if( durationInSeconds < (1.0 / maxFPS_))
                    return; // Skip if the time since the last trigger is less than the max FPS interval

                std_msgs::msg::Header header = leftWrench->header;
                
                #ifdef MANAGE_IMGS
                // Publish the latest image from the buffer
                if (!imageBuffer_.empty()) {
                    rclcpp::Time triggerTime = rclcpp::Time(header.stamp);
                    double closestTimeDiff = std::numeric_limits<double>::max();
                    size_t closestIndex = 0;
                    
                    for(size_t i = imageBuffer_.size() - 1; i > 0; --i) {
                        rclcpp::Time imageTime(imageBuffer_[i].header.stamp);
                        double timeDiff = std::abs((imageTime - triggerTime).seconds());
                        if (timeDiff < closestTimeDiff) {
                            closestTimeDiff = timeDiff;
                            closestIndex = i;
                        }
                    }

                    imagePub_->publish(imageBuffer_[closestIndex]); // Publish the closest image
                }
                #endif
                triggerPub_->publish(header);


                previousTrig_ = std::chrono::steady_clock::now(); // Update the time of the last trigger
            }
        }

        #ifdef MANAGE_IMGS
        void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
        {
            // Store the received image in the buffer
            imageBuffer_.push_back(*msg);

            // If the buffer exceeds a certain size, remove the oldest image
            if (imageBuffer_.size() > IMG_BUFFER_SIZE) {
                imageBuffer_.erase(imageBuffer_.begin());
            }
        }
        #endif
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RosBipedSynchronizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}