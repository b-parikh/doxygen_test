/**
 * @file creator.cpp
 * @brief Creates and stores RegularPolygon instances based on incoming msgs.
 * @author Biraj Parikh
 * @date April 24, 2020
 *
 * A node to create and store all RegularPolygon instances. Peridically, the node will broadcast
 * a list of all the polygons that it has already been asked to created.
 *
 * - PARAMETERS:
 *   - None
 * - PUBLISHES:
 *   - /builtPolygons
 * - SUBSCRIBES:
 *   - /polygonRequests
 * - SERVICES:
 *   - None
 * - INTERFACES:
 *   - None
 *
 * - LAUNCH FILES THIS NODE IS IN:
 *   - begin_test.launch
 */

#include <vector>
#include <string>
#include <signal.h>
#include <memory>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "doxygen_test/PolygonBuildData.h"  // custom msg
#include "doxygen_test/PolygonBuildDataArr.h" // custom msg

#include "../include/RegularPolygon.h"


#define NODE_NAME "Creator"  /**< Node name */
#define PUBLISH_RATE 0.5 /**< Rate of publishing in hertz */

std::vector<std::shared_ptr<RegularPolygon>> allPolygons;  /**< Stores all polygons that have been created already. */

/**
 * @brief Callback for /polygonRequests topic subscription
 *
 * When a polygon creation request comes in over /polygonRequests, the callback processes it. 
 * After parsing the request, a new polygon is created and stored in allPolygons.
 *
 * @param[in] msg PolygonBuildData instance containing data about what type of polygon to build
 */
void createNewPolygonCb(const doxygen_test::PolygonBuildData::ConstPtr& msg)
{
    RegularPolygon::COLOR c;
    std::string color = msg->color;
    if(color == "blue")
        c = RegularPolygon::COLOR::BLUE;
    else if(color == "red")
        c = RegularPolygon::COLOR::RED;
    else if(color == "green")
        c = RegularPolygon::COLOR::GREEN;
    else if(color == "black")
        c = RegularPolygon::COLOR::BLACK;
    else {
        ROS_INFO_STREAM("Invalid color " << color << ". Aborting polygon creation.");
        return;
    }

    if(msg->sideLength <= 0)
        ROS_INFO_STREAM("Invalid side length " << msg->sideLength << ". Aborting polygon creation.");

    std::shared_ptr<RegularPolygon> newPolygon;
    std::string type = msg->type;
    if(type == "triangle") {
        newPolygon = std::make_shared<Triangle>(msg->sideLength, c);
    } else if(type == "square") {
        newPolygon = std::make_shared<Square>(msg->sideLength, c);
    } else {
        ROS_INFO_STREAM("Invalid type " << type << ". Aborting polygon creation.");
        return;
    }

    ROS_INFO_STREAM("Created " << type << " of color " << color << " and side length " << msg->sideLength << ".");

    allPolygons.push_back(newPolygon);
}

/**
 * @brief Creates PolygonBuildDataArr msg from allPolygons
 *
 * Builds a PolygonBuildDataArr msg consisting of every previously created polygon's info.
 * The previously created polygons are stored in allPolygons.
 * This message is published onto /builtPolygons topic.
 *
 * @return doxygen_test::PolygonBuildDataArr with all built polygon info.
 */
doxygen_test::PolygonBuildDataArr createBuiltPolygonsMessage() {
    doxygen_test::PolygonBuildDataArr msg;
    for(int i = 0; i < allPolygons.size(); ++i) {
        doxygen_test::PolygonBuildData currPolygon;
        currPolygon.type = allPolygons[i]->getType();
        currPolygon.sideLength = allPolygons[i]->getSideLength();

        std::string c;
        RegularPolygon::COLOR color = allPolygons[i]->getColor();
        if(color == RegularPolygon::COLOR::BLUE)
            c = "blue";
        else if(color == RegularPolygon::COLOR::RED)
            c = "red";
        else if(color == RegularPolygon::COLOR::GREEN)
            c = "green";
        else if(color == RegularPolygon::COLOR::BLACK)
            c = "black";
        currPolygon.color = c;

        msg.polygons.push_back(currPolygon);
    }

    return msg;
}

void sigIntHandler(int sig) {
    ROS_INFO("Node shut down");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh;

    signal(SIGINT, sigIntHandler);

    ros::Subscriber sub = nh.subscribe("/polygonRequests", 10, createNewPolygonCb);
    ros::Publisher builtPolygonsPub = nh.advertise<doxygen_test::PolygonBuildDataArr>("/builtPolygons", 10);

    ros::Rate r(PUBLISH_RATE);
    while(ros::ok) {
        doxygen_test::PolygonBuildDataArr msg = createBuiltPolygonsMessage();
        builtPolygonsPub.publish(msg);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
