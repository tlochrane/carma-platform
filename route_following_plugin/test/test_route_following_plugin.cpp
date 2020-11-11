/*
 * Copyright (C) 2019-2020 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include "route_following_plugin.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>
#include <chrono>
#include<carma_wm/CARMAWorldModel.h>
#include<carma_wm/WMTestLibForGuidance.h>

namespace route_following_plugin
{

    TEST(RouteFollowingPluginTest, testFindLaneletIndexFromPath)
    {
        RouteFollowingPlugin rfp;
        lanelet::ConstLanelets lls;
        lanelet::Lanelet ll;
        ll.setId(15);
        lls.push_back(ll);
        lanelet::routing::LaneletPath path(lls);
        EXPECT_EQ(0, rfp.findLaneletIndexFromPath(15, path));
        EXPECT_EQ(-1, rfp.findLaneletIndexFromPath(5, path));
    }

    TEST(RouteFollowingPluginTest, testComposeManeuverMessage)
    {
        RouteFollowingPlugin rfp;
        auto msg = rfp.composeManeuverMessage(1.0, 10.0, 0.9, RouteFollowingPlugin::TWENTY_FIVE_MPH_IN_MS, 2, ros::Time(0, 0));
        EXPECT_EQ(cav_msgs::Maneuver::LANE_FOLLOWING, msg.type);
        EXPECT_EQ(cav_msgs::ManeuverParameters::NO_NEGOTIATION, msg.lane_following_maneuver.parameters.neogition_type);
        EXPECT_EQ(cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN, msg.lane_following_maneuver.parameters.presence_vector);
        EXPECT_EQ("InLaneCruisingPlugin", msg.lane_following_maneuver.parameters.planning_tactical_plugin);
        EXPECT_EQ("RouteFollowingPlugin", msg.lane_following_maneuver.parameters.planning_strategic_plugin);
        EXPECT_NEAR(1.0, msg.lane_following_maneuver.start_dist, 0.01);
        EXPECT_NEAR(0.9, msg.lane_following_maneuver.start_speed, 0.01);
        EXPECT_EQ(ros::Time(0), msg.lane_following_maneuver.start_time);
        EXPECT_NEAR(10.0, msg.lane_following_maneuver.end_dist, 0.01);
        EXPECT_NEAR(25 / 2.237, msg.lane_following_maneuver.end_speed, 0.01);
        EXPECT_TRUE(msg.lane_following_maneuver.end_time - ros::Time(1.49) < ros::Duration(0.01));
        EXPECT_EQ("2", msg.lane_following_maneuver.lane_id);
    }

    TEST(RouteFollowingPluginTest, testIdentifyLaneChange)
    {
        RouteFollowingPlugin rfp;
        auto relations = lanelet::routing::LaneletRelations();
        EXPECT_FALSE(rfp.identifyLaneChange(relations, 1));
        lanelet::routing::LaneletRelation relation;
        relation.relationType = lanelet::routing::RelationType::Successor;
        relations.push_back(relation);
        EXPECT_TRUE(rfp.identifyLaneChange(relations, 0));
    }

    TEST(RouteFollowingPluginTest, testGetSpeedLimit){
        RouteFollowingPlugin worker;
        //pre callback variables
        //define pose
        worker.pose_msg_.pose.position.x=5.55;
        worker.pose_msg_.pose.position.y=12.5;
        worker.pose_msg_.pose.position.z=0.0;
        
        worker.pose_msg_.pose.orientation.x=0.0;
        worker.pose_msg_.pose.orientation.y=0.0;
        worker.pose_msg_.pose.orientation.z=0.0;
        worker.pose_msg_.pose.orientation.w=0.0;
        //define twist
        worker.current_speed_=10.0;

        //Define World Model
        carma_wm::test::MapOptions options;
        options.lane_length_=25;
        options.lane_width_=3.7;
        options.speed_limit_=carma_wm::test::MapOptions::SpeedLimit::DEFAULT;
        options.obstacle_=carma_wm::test::MapOptions::Obstacle::NONE;


        //Create Map
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw=std::make_shared<carma_wm::CARMAWorldModel>();
        //create the Semantic Map
        lanelet::LaneletMapPtr map=carma_wm::test::buildGuidanceTestMap(options.lane_width_,options.lane_length_);
        //set the map with default routingGraph
        cmw->carma_wm::CARMAWorldModel::setMap(map);
        worker.wm_=cmw;
        carma_wm::test::setRouteByIds({1210,1213},cmw);
        lanelet::Velocity speed_limit=30_mph;
        carma_wm::test::setSpeedLimit(speed_limit,cmw);
 
        //Define plan for request and response
        //PlanManeuversRequest
        cav_srvs::PlanManeuvers plan;
        cav_srvs::PlanManeuversRequest pplan;
        
        cav_msgs::ManeuverPlan plan_req1;
        plan_req1.header;
        plan_req1.maneuver_plan_id;
        plan_req1.planning_start_time;
        plan_req1.planning_completion_time;
        //cav_msgs::Maneuver RouteFollowingPlugin::composeManeuverMessage(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time current_time)
        plan_req1.maneuvers.push_back(worker.composeManeuverMessage(0,0,0,0,0,ros::Time(0)));
        pplan.prior_plan=plan_req1;
        plan.request=pplan;
        //PlanManeuversResponse 
        cav_srvs::PlanManeuversResponse newplan;
        for(auto i=0;i<plan_req1.maneuvers.size();i++) newplan.new_plan.maneuvers.push_back(plan_req1.maneuvers[i]);

        plan.response=newplan;
        
        //RouteFollowing plan maneuver callback
        ros::Time::init();  //initializing ros time to use ros::Time::now()
        if(worker.plan_maneuver_cb(plan.request,plan.response)){    //boolean
            //to check whenever speed limit exists in Map, target speed is changed
            //check target speeds in updated response
            for(auto i=0;i<plan.response.new_plan.maneuvers.size();i++){
                std::cout<<"Response end speed:"<< "maneuver[" << i <<"]:"<< plan.response.new_plan.maneuvers[i].lane_following_maneuver.end_speed<<std::endl;
            }
        }
        else{
            std::cout<<"No maneuvers planned"<<std::endl;
        }
        EXPECT_TRUE(true);

    }

}

// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();
    return res;
}


