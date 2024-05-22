#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <cnoid/EigenTypes>
#include <cnoid/YAMLReader>
#include <visualization_msgs/MarkerArray.h>
#include <tactile_sensor_msgs/WrenchStampedArray.h>

namespace tactile_sensor_visualizer {
  class tactile_sensor_visualizer
  {
  public:
    tactile_sensor_visualizer() {
      ros::NodeHandle nh, pnh("~");
      tactileSensorSub_ = pnh.subscribe("input", 1, &tactile_sensor_visualizer::tactileSensorCallback, this);
      this->marker_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("marker", 1, true /* latch */);
      this->wrench_pub_ = pnh.advertise<tactile_sensor_msgs::WrenchStampedArray>("wrench", 1);

      // load tactile_sensor_file
      {
        std::string fileName;
        pnh.getParam("filename", fileName);
        ROS_INFO_STREAM("lading " << fileName);
        cnoid::YAMLReader reader;
        cnoid::MappingPtr node;
        try {
          node = reader.loadDocument(fileName)->toMapping();
        } catch(const cnoid::ValueNode::Exception& ex) {
          ROS_ERROR_STREAM(ex.message());
        }
        // load
        auto& tactileSensorList = *node->findListing("tactile_sensor");
        if (!tactileSensorList.isValid()) {
          ROS_ERROR_STREAM("cannot load config file");
          return;
        } else {
          for (int i=0; i< tactileSensorList.size(); i++) {
            cnoid::Mapping* info = tactileSensorList[i].toMapping();
            TactileSensor sensor;
            // link
            info->extract("name", sensor.name);
            this->tactileSensorList_.push_back(sensor);
          }
        }
      }

      this->marker_.markers.resize(this->tactileSensorList_.size());
      for(int i=0;i<this->tactileSensorList_.size();i++){
        this->marker_.markers[i].header.frame_id = this->tactileSensorList_[i].name;
        this->marker_.markers[i].header.stamp = ros::Time(0);
        if(ros::this_node::getNamespace() == ""){
          this->marker_.markers[i].ns = ros::this_node::getName();
        }else{
          this->marker_.markers[i].ns = ros::this_node::getNamespace() + "/" + ros::this_node::getName();
        }
        this->marker_.markers[i].id = i;
        this->marker_.markers[i].lifetime = ros::Duration();
        this->marker_.markers[i].type = visualization_msgs::Marker::LINE_LIST;
        this->marker_.markers[i].action = visualization_msgs::Marker::ADD;
        this->marker_.markers[i].pose.orientation.w = 1.0; // rvizにUninitialized quaternion, assuming identity ワーニングが出る
        this->marker_.markers[i].scale.x = 0.002; // only scale.x is used and it controls the width of the line segments.
        this->marker_.markers[i].scale.y = 0.0;
        this->marker_.markers[i].scale.z = 0.0;
        //  It will draw a line between each pair of points, so 0-1, 2-3, 4-5, ...
        this->marker_.markers[i].points.resize(6);
        this->marker_.markers[i].colors.resize(6);
        this->marker_.markers[i].points[0].x = 0;
        this->marker_.markers[i].points[0].y = 0;
        this->marker_.markers[i].points[0].z = 0;
        this->marker_.markers[i].colors[0].r = 1.0f;
        this->marker_.markers[i].colors[0].g = 0.0f;
        this->marker_.markers[i].colors[0].b = 0.0f;
        this->marker_.markers[i].colors[0].a = 1.0f;
        this->marker_.markers[i].points[1].x = 0.005;
        this->marker_.markers[i].points[1].y = 0.0;
        this->marker_.markers[i].points[1].z = 0.0;
        this->marker_.markers[i].colors[1].r = 1.0f;
        this->marker_.markers[i].colors[1].g = 0.0f;
        this->marker_.markers[i].colors[1].b = 0.0f;
        this->marker_.markers[i].colors[1].a = 1.0f;
        this->marker_.markers[i].points[2].x = 0;
        this->marker_.markers[i].points[2].y = 0;
        this->marker_.markers[i].points[2].z = 0;
        this->marker_.markers[i].colors[2].r = 0.0f;
        this->marker_.markers[i].colors[2].g = 1.0f;
        this->marker_.markers[i].colors[2].b = 0.0f;
        this->marker_.markers[i].colors[2].a = 1.0f;
        this->marker_.markers[i].points[3].x = 0.0;
        this->marker_.markers[i].points[3].y = 0.005;
        this->marker_.markers[i].points[3].z = 0.0;
        this->marker_.markers[i].colors[3].r = 0.0f;
        this->marker_.markers[i].colors[3].g = 1.0f;
        this->marker_.markers[i].colors[3].b = 0.0f;
        this->marker_.markers[i].colors[3].a = 1.0f;
        this->marker_.markers[i].points[4].x = 0;
        this->marker_.markers[i].points[4].y = 0;
        this->marker_.markers[i].points[4].z = 0;
        this->marker_.markers[i].colors[4].r = 0.0f;
        this->marker_.markers[i].colors[4].g = 0.0f;
        this->marker_.markers[i].colors[4].b = 1.0f;
        this->marker_.markers[i].colors[4].a = 1.0f;
        this->marker_.markers[i].points[5].x = 0.0;
        this->marker_.markers[i].points[5].y = 0.0;
        this->marker_.markers[i].points[5].z = 0.005;
        this->marker_.markers[i].colors[5].r = 0.0f;
        this->marker_.markers[i].colors[5].g = 0.0f;
        this->marker_.markers[i].colors[5].b = 1.0f;
        this->marker_.markers[i].colors[5].a = 1.0f;
      }
      this->marker_pub_.publish(this->marker_);
    }
    void tactileSensorCallback(const std_msgs::Float32MultiArray& msg)
    {
      if(msg.data.size() != this->tactileSensorList_.size()*3) {
        ROS_ERROR_STREAM("data length is different. port data length : " << msg.data.size() << " config file sensor length : " << this->tactileSensorList_.size()*3);
        return;
      }

      this->wrench_.header.stamp = ros::Time::now();
      this->wrench_.header.frame_id = (this->tactileSensorList_.size()!=0) ? this->tactileSensorList_[0].name : std::string(""); // rviz内のmessagefilterがこのframe_idを使う
      int wrench_idx = 0;
      for (int i=0; i < this->tactileSensorList_.size(); i++) {
        if(msg.data[i*3+0] == 0 &&
           msg.data[i*3+1] == 0 &&
           msg.data[i*3+2] == 0) continue;
        if(this->wrench_.wrenchstampeds.size() <= wrench_idx) this->wrench_.wrenchstampeds.resize(wrench_idx+1);
        this->wrench_.wrenchstampeds[wrench_idx].header.stamp = ros::Time::now();
        this->wrench_.wrenchstampeds[wrench_idx].header.frame_id = this->tactileSensorList_[i].name;
        this->wrench_.wrenchstampeds[wrench_idx].wrench.force.x = msg.data[i*3+0];
        this->wrench_.wrenchstampeds[wrench_idx].wrench.force.y = msg.data[i*3+1];
        this->wrench_.wrenchstampeds[wrench_idx].wrench.force.z = msg.data[i*3+2];
        this->wrench_.wrenchstampeds[wrench_idx].wrench.torque.x = 0.0;
        this->wrench_.wrenchstampeds[wrench_idx].wrench.torque.y = 0.0;
        this->wrench_.wrenchstampeds[wrench_idx].wrench.torque.z = 0.0;
        wrench_idx++;
      }
      if(this->wrench_.wrenchstampeds.size() > wrench_idx) this->wrench_.wrenchstampeds.resize(wrench_idx);
      this->wrench_pub_.publish(this->wrench_);

    }
  protected:
    class TactileSensor
    {
    public:
      std::string name;
    };
    ros::Subscriber tactileSensorSub_;
    std::vector<TactileSensor> tactileSensorList_;
    visualization_msgs::MarkerArray marker_;
    ros::Publisher marker_pub_;
    // センサの個数が多いので、毎周期全てのセンサのデータを描画することは困難. 接触しているセンサのみ描画したい. visualization_msgs::MarkerArrayは、接触しているセンサが減った場合にDelete トピックを送る必要があるが、高周期でデータを送る場合Delete トピックを取りこぼす恐れがある.
    tactile_sensor_msgs::WrenchStampedArray wrench_;
    ros::Publisher wrench_pub_;
  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tactile_sensor_visualizer");
  ros::NodeHandle nh;
  tactile_sensor_visualizer::tactile_sensor_visualizer t;
  ros::spin();
}
