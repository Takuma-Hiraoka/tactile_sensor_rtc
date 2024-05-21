#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <cnoid/EigenTypes>
#include <cnoid/YAMLReader>
#include <visualization_msgs/MarkerArray.h>

namespace tactile_sensor_visualizer {
  class tactile_sensor_visualizer
  {
  public:
    tactile_sensor_visualizer() {
      ros::NodeHandle nh, pnh("~");
      tactileSensorSub_ = pnh.subscribe("input", 1, &tactile_sensor_visualizer::tactileSensorCallback, this);
      this->marker_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("marker", 1);

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
        this->marker_.markers[i].type = visualization_msgs::Marker::ARROW;
        this->marker_.markers[i].action = visualization_msgs::Marker::ADD;
        this->marker_.markers[i].pose.orientation.w = 1.0; // rvizにUninitialized quaternion, assuming identity ワーニングが出る
        this->marker_.markers[i].scale.x = 0.002; //  shaft diameter, and scale.y is the head diameter.
        this->marker_.markers[i].scale.y = 0.004; // head diameter
        this->marker_.markers[i].scale.z = 0.0; // If scale.z is not zero, it specifies the head length.
        //start end
        this->marker_.markers[i].points.resize(2);
        this->marker_.markers[i].points[0].x = 0;
        this->marker_.markers[i].points[0].y = 0;
        this->marker_.markers[i].points[0].z = 0;
        this->marker_.markers[i].points[1].x = 0;
        this->marker_.markers[i].points[1].y = 0;
        this->marker_.markers[i].points[1].z = 0;
        //color
        this->marker_.markers[i].color.r = 0.0f;
        this->marker_.markers[i].color.g = 1.0f;
        this->marker_.markers[i].color.b = 0.0f;
        this->marker_.markers[i].color.a = 1.0f;
      }
    }
    void tactileSensorCallback(const std_msgs::Float32MultiArray& msg)
    {
      if(msg.data.size() != this->tactileSensorList_.size()*3) {
        ROS_ERROR_STREAM("data length is different. port data length : " << msg.data.size() << " config file sensor length : " << this->tactileSensorList_.size()*3);
        return;
      }
      for (int i=0; i < this->tactileSensorList_.size(); i++) {
        this->marker_.markers[i].header.stamp = ros::Time(0);
        this->marker_.markers[i].points[1].x = msg.data[i*3+0] * 0.002;
        this->marker_.markers[i].points[1].y = msg.data[i*3+1] * 0.002;
        this->marker_.markers[i].points[1].z = msg.data[i*3+2] * 0.002;
      }
      this->marker_pub_.publish(this->marker_);
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
  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tactile_sensor_visualizer");
  ros::NodeHandle nh;
  tactile_sensor_visualizer::tactile_sensor_visualizer t;
  ros::spin();
}
