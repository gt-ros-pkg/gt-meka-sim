/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Daehyung Park (Dr. Charles C. Kemp's HRL, GIT).
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Daehyung Park nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *  
 * Daehyung Park is with Dr. Charles C. Kemp's the Healthcare Robotics Lab, 
 * Institute for Robotics & Intelligent Machines, Georgia Institute of 
 * Technology. (Contact: deric.park@gatech.edu)
 *
 * We gratefully acknowledge support from DARPA Maximum Mobility
 * and Manipulation (M3) Contract W911NF-11-1-603.
 *********************************************************************/

#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/World.hh>
#include <physics/physics.hh>
#include <sensors/sensors.hh>
#include <sensors/ContactSensor.hh>
#include <math/Pose.hh>
#include <math/Quaternion.hh>
#include <math/Vector3.hh>
#include <common/common.hh>
#include <stdio.h>
#include <boost/thread/mutex.hpp>

#include "ros/ros.h"
#include "hrl_haptic_manipulation_in_clutter_msgs/TaxelArray.h"

#include "KMlocal.h"

namespace gazebo
{   
  class ROSTactilePlugin : public SensorPlugin
  {

    public: ROSTactilePlugin()
    {
      // KM variable initialization
      // execution parameters (see KMterm.h and KMlocal.h)
      this->KM_term = new KMterm(50, 0, 0, 0, 0.1, 0.1, 3, 0.5, 10, 0.95);
      // number of centers
      this->KM_k   = 5;
      // dimension
      this->KM_dim = 3;
      // init data pts object
      this->KM_dataPts = new KMdata(this->KM_dim, 3); //hard code the default nPts
    }
    public: ~ROSTactilePlugin()
    {
      this->KM_term->~KMterm(); // does this work?
      this->KM_dataPts->~KMdata();
     
      delete this->KM_term;
      delete this->KM_dataPts;
    }

    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {     
      // Start up ROS
      if (!ros::isInitialized()){
        std::string name = "ros_tactile_plugin_node";
        int argc = 0;
        ros::init(argc, NULL, name);
      }
      else{
        ROS_WARN("ROS Tactile Plugin>> Something other than this ros_world plugin started ros::init(...), clock may not be published properly.");
      }


      // Get the parent sensor.
      this->parentSensor =
        boost::shared_dynamic_cast<sensors::ContactSensor>(_sensor);

      // Make sure the parent sensor is valid.
      if (!this->parentSensor)
        { 
          gzerr << "ROSTactilePlugin requires a ContactSensor.\n";
          return;
        } 

      // Get the parent world.
      std::string worldName = this->parentSensor->GetWorldName();
      this->parentWorld = physics::get_world(worldName);

      // ROS Nodehandle
      this->node = new ros::NodeHandle("ROSTactileNode");

      // ROS Topic Base Name
      std::string topicName = "/";
      topicName.append(this->parentSensor->GetName());
      topicName.append("/taxels/forces");

      // ROS Publisher
      this->pub = this->node->advertise<hrl_haptic_manipulation_in_clutter_msgs::TaxelArray>(topicName, 1);

      // Frame Initialize
      // get the contact pose reference frame name
      this->frame_name_ = "torso_lift_link";

      // preset myFrame to NULL, will search for the body with matching name in UpdateChild()
      // since most bodies are constructed on the fly
      //this->refFrame = NULL;

      // ROS Taxel Array
      this->frame_id.append(this->parentSensor->GetParentName());
      this->frame_id = this->frame_id.substr(this->frame_id.rfind(":")+1,this->frame_id.length());
      this->taxel.header.frame_id = "torso_lift_link";//"this->frame_id; //this->parentSensor->GetParentName();
      this->taxel.sensor_type     = "force";

      // Connect to the sensor update event.
      this->updateConnection = this->parentSensor->ConnectUpdated(boost::bind(&ROSTactilePlugin::OnUpdate, this));

      // Make sure the parent sensor is active.
      this->parentSensor->SetActive(true);      
    }


    // Called by the world update start event
    public: void OnUpdate()
    {
      // mutex
      boost::mutex::scoped_lock lock(this->mutex);

      // Get Torso Lift Link Frame
      /// if frameName specified is "world", "/map" or "map" report back inertial values in the gazebo world
      if (this->frame_name_ != "world" && this->frame_name_ != "/map" && this->frame_name_ != "map" ){        
        
        // look through all models in the world, search for body name that matches frameName
        std::vector<physics::ModelPtr> all_models = this->parentWorld->GetModels();
        for (std::vector<physics::ModelPtr>::iterator iter = all_models.begin(); iter != all_models.end(); iter++){
          if (*iter) this->refFrame = (*iter)->GetLink(this->frame_name_);
          if (this->refFrame) break;
        }
        
        // not found
        if (this->refFrame == NULL){
          ROS_DEBUG("ros_tactile_plugin: frameName: %s does not exist yet, will not publish\n",this->frame_name_.c_str());
          return;
        }
      }

      // Get all the contacts.
      msgs::Contacts contacts;
      contacts = this->parentSensor->GetContacts();
            
      // Set simulation time to taxel msg header
      this->taxel.header.stamp = ros::Time(this->parentWorld->GetSimTime().Double());
      
      // Set data points for KM
      this->KM_nPts    = contacts.contact_size();
      // Number of Available Contacts Index
      int KM_Avlb_nPts = 0;

      // Plugin's relative speed with respect to simulator
      double plugin_speed = 0.01;
      
      // Only if there is at least one contact,
      if (this->KM_nPts > 0){
        // Init pre_clustered contacts vectors
        math::Vector3 pre_contacts_position[this->KM_nPts];
        math::Vector3 pre_contacts_normal[this->KM_nPts];
        math::Vector3 pre_contacts_force[this->KM_nPts];
        
        // Init clustered contacts vectors
        math::Vector3 contacts_position[this->KM_nPts];
        math::Vector3 contacts_normal[this->KM_nPts];
        math::Vector3 contacts_force[this->KM_nPts];
        
        // get reference frame (body(link)) pose and subtract from it to get 
        // relative force, torque, position and normal vectors
        math::Pose frame_pose;
        math::Quaternion frame_rot;
        math::Vector3 frame_pos;
        
        //bool body1_flag = false;
        
        // Get reference frame (torso)
        if (this->refFrame){
            frame_pose = this->refFrame->GetWorldPose();//-this->myBody->GetCoMPose();
            frame_pos = frame_pose.pos;
            frame_rot = frame_pose.rot;
        }
        else{
            gzdbg << "No reference frame!!!\n";
            // no specific frames specified, use identity pose, keeping 
            // relative frame at inertial origin
            frame_pos = math::Vector3(0,0,0);
            frame_rot = math::Quaternion(1,0,0,0); // gazebo u,x,y,z == identity
            frame_pose = math::Pose(frame_pos, frame_rot);
        }

        // Main loop
        for (unsigned int i = 0; i < this->KM_nPts; ++i){          
         
          // // Get frame name
          // std::string collisionFrame1 = contacts.contact(i).collision1();
          // collisionFrame1 = collisionFrame1.substr(collisionFrame1.rfind(":"),collisionFrame1.length());

          // std::string collisionFrame2 = contacts.contact(i).collision2();
          // collisionFrame2 = collisionFrame2.substr(collisionFrame2.rfind(":"),collisionFrame2.length());
          
          // // Find collision1 which has specified frame
          // if (collisionFrame1.find("hand") != std::string::npos || collisionFrame2.find("hand") != std::string::npos){
          //   std::cout << "Collision between[" << contacts.contact(i).collision1()
          //             << "] and [" << contacts.contact(i).collision2() << "]\n";            
          // }  
                    
          // Find collision1 which has specified frame
          // if (collisionFrame.find(this->frame_id) != std::string::npos || true)
          //   {
          
          // if (collisionFrame.find("upper") != std::string::npos && i==0)
          //   {
          //     gzdbg << "--------- " << i << " == " << contacts.contact_size() << "-----------\n";
          //   }
          // gzdbg << collisionFrame << " " << this->frame_id << "\n";
                    
          // Init average contact vectors
          math::Vector3 contact_position_avg = math::Vector3(0,0,0);
          math::Vector3 contact_normal_avg   = math::Vector3(0,0,0);     
          math::Vector3 contact_force_sum    = math::Vector3(0,0,0);
          
          int nPts = 0;
          
          // Sub loop to get contact information from one contact object
          for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j){

            // Force is not normal-force. Thus, it should be projected to normal vector, since 
            // real robot only can sense normal-forces.
            // RVIZ : only handle normal force
            // MPC  : automatically project force to normal vector

            // The order of force pair is not consistent. 
            // So, I used absolute force with normal vector after this. 
            std::string body_name = contacts.contact(i).wrench(j).body_1_name();
            math::Vector3 contact_force;

            if (body_name.find(this->frame_id) != std::string::npos){                        
              contact_force = msgs::Convert(contacts.contact(i).wrench(j).body_2_force());
            }
            else{
              contact_force = msgs::Convert(contacts.contact(i).wrench(j).body_1_force());
            }

            // Count only available forces
            // HACK: exclude too huge force by Daehyung 20130729
            if (contact_force.GetSquaredLength() > 0.0001 && contact_force.GetSquaredLength() < 1000.){
                contact_force_sum += frame_rot.RotateVectorReverse(contact_force);
                nPts++;
            }
            else{
                continue;
            }

            // Contact position
            math::Vector3 contact_position = msgs::Convert(contacts.contact(i).position(j));
            contact_position = contact_position - frame_pos;
            contact_position_avg += frame_rot.RotateVectorReverse(contact_position);

            // rotate normal into user specified frame. 
            // frame_rot is identity if world is used.
            math::Vector3 contact_normal = msgs::Convert(contacts.contact(i).normal(j));
            std::string collisionFrame1 = contacts.contact(i).collision1();
            collisionFrame1 = collisionFrame1.substr(collisionFrame1.rfind(":"),collisionFrame1.length());
            if (collisionFrame1.find(this->frame_id) != std::string::npos){
              contact_normal_avg -= frame_rot.RotateVectorReverse(contact_normal);             
            }
            else{
              contact_normal_avg += frame_rot.RotateVectorReverse(contact_normal);             
            }
          }
          
          if (nPts > 0){
            contact_position_avg /= (double)nPts;
            contact_normal_avg.Normalize();
            //contact_force_avg /= (double)nPts;

            // TODO!!
            // Why force_sum is normal direction? there is no need to update force_sum~~!!
            double contact_normal_force_mag = contact_normal_avg.Dot(contact_force_sum); // projection
            contact_force_sum = contact_normal_avg*fabs(contact_normal_force_mag); // normal_force
            // contact_normal_force_sum = contact_normal_avg*fabs(contact_force_sum); // normal_force vector
            
            // Store contacts data which can be clustered after this.
            pre_contacts_position[KM_Avlb_nPts].x = contact_position_avg.x;
            pre_contacts_position[KM_Avlb_nPts].y = contact_position_avg.y;
            pre_contacts_position[KM_Avlb_nPts].z = contact_position_avg.z;
            
            pre_contacts_normal[KM_Avlb_nPts].x = contact_normal_avg.x;
            pre_contacts_normal[KM_Avlb_nPts].y = contact_normal_avg.y;
            pre_contacts_normal[KM_Avlb_nPts].z = contact_normal_avg.z;
            
            pre_contacts_force[KM_Avlb_nPts].x = contact_force_sum.x;
            pre_contacts_force[KM_Avlb_nPts].y = contact_force_sum.y;
            pre_contacts_force[KM_Avlb_nPts].z = contact_force_sum.z;

            KM_Avlb_nPts++;
          }
        }
    
        // K Means Clutering
        if (KM_Avlb_nPts > this->KM_k){

          // Init variable
          this->KM_dataPts->resize(this->KM_dim, KM_Avlb_nPts);
          KMdataArray const& KM_dataArray = this->KM_dataPts->getPts();
          
          // Get K means clustering data
          for (unsigned int i = 0; i < KM_Avlb_nPts; i++){
            KM_dataArray[i][0] = pre_contacts_position[i].x;
            KM_dataArray[i][1] = pre_contacts_position[i].y;
            KM_dataArray[i][2] = pre_contacts_position[i].z;
          }
          
          // build filtering structure
          this->KM_dataPts->buildKcTree();
          // allocate centers
          KMfilterCenters KM_ctrs(this->KM_k, *this->KM_dataPts);
          // repeated Lloyd’s
          KMlocalLloyds KM_kmAlg(KM_ctrs, *this->KM_term);
          // execute
          KM_ctrs = KM_kmAlg.execute();
          
          // Clustering by distance
          KMctrIdxArray KM_closeCtr = new KMctrIdx[this->KM_dataPts->getNPts()];
          double* KM_sqDist = new double[this->KM_dataPts->getNPts()];
          KM_ctrs.getAssignments(KM_closeCtr, KM_sqDist);
          
          // Classify
          for (unsigned int i = 0; i < KM_k; i++){
            //
            math::Vector3 contact_position_avg = math::Vector3(0,0,0);
            math::Vector3 contact_normal_avg   = math::Vector3(0,0,0);      
            math::Vector3 contact_force_sum    = math::Vector3(0,0,0);
            int nPts = 0;
            
            for (unsigned int j = 0; j < KM_Avlb_nPts; j++){              
              // Sum up the same group info
              if (KM_closeCtr[j] == i){
                nPts++;
                
                contact_position_avg.x += pre_contacts_position[j].x;
                contact_position_avg.y += pre_contacts_position[j].y;
                contact_position_avg.z += pre_contacts_position[j].z;
                
                contact_normal_avg.x += pre_contacts_normal[j].x;
                contact_normal_avg.y += pre_contacts_normal[j].y;
                contact_normal_avg.z += pre_contacts_normal[j].z;
                
                contact_force_sum.x += pre_contacts_force[j].x;
                contact_force_sum.y += pre_contacts_force[j].y;
                contact_force_sum.z += pre_contacts_force[j].z;
              }
            }
            
            // normalization and projection
            if (nPts > 0){
              contact_position_avg /= (double)nPts;
              contact_normal_avg.Normalize();
              //contact_force_avg /= (double)nPts;
              
              // projection
              double contact_normal_force_mag = contact_normal_avg.Dot(contact_force_sum); 
              // normal_force
              contact_force_sum = contact_normal_avg*contact_normal_force_mag; 
            }            
            
            contacts_position[i].x = contact_position_avg.x;
            contacts_position[i].y = contact_position_avg.y;
            contacts_position[i].z = contact_position_avg.z;
            
            contacts_normal[i].x = contact_normal_avg.x;
            contacts_normal[i].y = contact_normal_avg.y;
            contacts_normal[i].z = contact_normal_avg.z;
            
            contacts_force[i].x = contact_force_sum.x;
            contacts_force[i].y = contact_force_sum.y;
            contacts_force[i].z = contact_force_sum.z;          
          }
          
          delete [] KM_closeCtr;
          delete [] KM_sqDist;
        }
        else{
          for (unsigned int i = 0; i < KM_Avlb_nPts; i++){
            contacts_position[i] = pre_contacts_position[i];
            contacts_normal[i]   = pre_contacts_normal[i];
            contacts_force[i]    = pre_contacts_force[i];               
          }
        }
            
        // TODO : check distance and re-clustering one more
        // int nPts = KM_Avlb_nPts; //(KM_Avlb_nPts <= this->KM_k)? KM_Avlb_nPts: this->KM_k;
        int nPts = (KM_Avlb_nPts > this->KM_k)? this->KM_k : KM_Avlb_nPts;
        
        if (nPts > 1){
          int nGroupInd[nPts];
          bool bIndCheck[nPts];
          int nGroupPts = 0;
          double dist;

          memset(nGroupInd, 0, sizeof(nGroupInd));
          memset(bIndCheck, 0, sizeof(bIndCheck));

          for (unsigned int i = 0; i < nPts; i++){

            // Pass checked points
            if (bIndCheck[i] == true) continue;
            else{
              bIndCheck[i] = true;           
              nGroupInd[i] = nGroupPts;
              nGroupPts++;
            }

            for (unsigned int j = 1; j < nPts; j++){

              // Pass checked points
              if (bIndCheck[j] == true) continue;

              // Get distance between points
              dist = contacts_position[i].Distance(contacts_position[j]);

              // 
              if (dist < 0.01){
                nGroupInd[j] = nGroupInd[i];
                bIndCheck[j] = true;
              }
            }
          }

          // Grouping
          for (unsigned int i = 0; i < nGroupPts; i++){              

            // number of point in group
            int n = 0;

            math::Vector3 contact_position_avg = math::Vector3(0,0,0);
            math::Vector3 contact_normal_avg   = math::Vector3(0,0,0);      
            math::Vector3 contact_force_sum    = math::Vector3(0,0,0);

            for (unsigned int j = 0; j < nPts; j++){              
              // Sum up the same group info
              if (nGroupInd[j] == i){
                n++;
                
                contact_position_avg.x += contacts_position[j].x;
                contact_position_avg.y += contacts_position[j].y;
                contact_position_avg.z += contacts_position[j].z;
                
                contact_normal_avg.x += contacts_normal[j].x;
                contact_normal_avg.y += contacts_normal[j].y;
                contact_normal_avg.z += contacts_normal[j].z;
                
                contact_force_sum.x += contacts_force[j].x;
                contact_force_sum.y += contacts_force[j].y;
                contact_force_sum.z += contacts_force[j].z;
              }
            }

            // normalization and projection
            if (n > 0){
              contact_position_avg /= (double)n;
              contact_normal_avg.Normalize();
              //contact_force_avg /= (double)n;
              
              // Only take projected force
              double contact_normal_force_mag = contact_normal_avg.Dot(contact_force_sum); 
              // normal_force
              contact_force_sum = contact_normal_avg*contact_normal_force_mag; 
            }            
            
            contacts_position[i].x = contact_position_avg.x;
            contacts_position[i].y = contact_position_avg.y;
            contacts_position[i].z = contact_position_avg.z;
            
            contacts_normal[i].x = contact_normal_avg.x;
            contacts_normal[i].y = contact_normal_avg.y;
            contacts_normal[i].z = contact_normal_avg.z;
            
            contacts_force[i].x = contact_force_sum.x;
            contacts_force[i].y = contact_force_sum.y;
            contacts_force[i].z = contact_force_sum.z;                     
          }          

          nPts = nGroupPts;
        }

        // set taxel array message
        for (unsigned int i = 0; i < nPts; i++){

          if (fabs(contacts_force[i].x) + fabs(contacts_force[i].y) + fabs(contacts_force[i].z) > 1e-3){

            // set taxel array message
            this->taxel.centers_x.push_back(contacts_position[i].x);
            this->taxel.centers_y.push_back(contacts_position[i].y);
            this->taxel.centers_z.push_back(contacts_position[i].z);
            
            this->taxel.normals_x.push_back(contacts_normal[i].x);
            this->taxel.normals_y.push_back(contacts_normal[i].y);
            this->taxel.normals_z.push_back(contacts_normal[i].z);
            
            this->taxel.values_x.push_back(contacts_force[i].x * plugin_speed);
            this->taxel.values_y.push_back(contacts_force[i].y * plugin_speed);
            this->taxel.values_z.push_back(contacts_force[i].z * plugin_speed);
            this->taxel.link_names.push_back(this->frame_id);              
          }
        }
      }
        
      this->pub.publish(taxel);
      
      // Clear taxel array
      this->taxel.centers_x.clear();
      this->taxel.centers_y.clear();
      this->taxel.centers_z.clear();
      this->taxel.normals_x.clear();
      this->taxel.normals_y.clear();
      this->taxel.normals_z.clear();
      this->taxel.values_x.clear();
      this->taxel.values_y.clear();
      this->taxel.values_z.clear();
      this->taxel.link_names.clear();
      
      ros::spinOnce();
    }

    /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS Nodehandle
    private: ros::NodeHandle* node;

    // ROS Publisher
    ros::Publisher pub;

    // ROS TaxelArray
    hrl_haptic_manipulation_in_clutter_msgs::TaxelArray taxel;

    // Frame conversion
    private: std::string frame_id;
    private: std::string frame_name_;
    
    // Sensor reference frame
    private: physics::LinkPtr refFrame;

    // World
    private: physics::WorldPtr parentWorld;

    /// Mutex to protect updates.
    private: boost::mutex mutex;

    // K-means clustring
    private: KMterm *KM_term;
    private: int KM_k; 
    private: int KM_dim;
    private: int KM_nPts;
    private: KMdata *KM_dataPts;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(ROSTactilePlugin)
}



