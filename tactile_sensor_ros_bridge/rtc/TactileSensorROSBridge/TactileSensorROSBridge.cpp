#include "TactileSensorROSBridge.h"
#include <cnoid/YAMLReader>
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/Point.h>

TactileSensorROSBridge::TactileSensorROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_tactileSensorArrayIn_("tactileSensorArrayIn", m_tactileSensorArray_)
{
}

RTC::ReturnCode_t TactileSensorROSBridge::onInitialize(){
  addInPort("tactileSensorArrayIn", this->m_tactileSensorArrayIn_);
  ros::NodeHandle pnh("~");

  this->tactile_sensor_array_pub = nh.advertise<visualization_msgs::MarkerArray>("tactile_sensor", 1);
  this->arrow.x = 0.006;
  this->arrow.y = 0.01;
  this->arrow.z = 0.01;

  // load tactile_sensor_file
  {
    std::string fileName;
    if(this->getProperties().hasKey("tactile_sensor_file")) fileName = std::string(this->getProperties()["tactile_sensor_file"]);
    else fileName = std::string(this->m_pManager->getConfig()["tactile_sensor_file"]); // 引数 -o で与えたプロパティを捕捉
    std::cerr << "[" << this->m_profile.instance_name << "] tactile_sensor_file: " << fileName <<std::endl;
    cnoid::YAMLReader reader;
    cnoid::MappingPtr node;
    try {
      node = reader.loadDocument(fileName)->toMapping();
    } catch(const cnoid::ValueNode::Exception& ex) {
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << ex.message() << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }
    // load
    auto& tactileSensorList = *node->findListing("tactile_sensor");
    if (!tactileSensorList.isValid()) {
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "cannot load config file" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    } else {
      for (int i=0; i< tactileSensorList.size(); i++) {
        cnoid::Mapping* info = tactileSensorList[i].toMapping();
        TactileSensor sensor;
        // link
        std::string linkName;
        info->extract("link", linkName);
        sensor.linkName = linkName;
        // type
        std::string type;
        info->extract("type", type);
        if (type == "rectangle") {
          auto point_ = info->extract("point");
          auto& pointTmp = *point_->toListing();
          cnoid::Vector3 point = cnoid::Vector3(pointTmp[0].toDouble(), pointTmp[1].toDouble(), pointTmp[2].toDouble());
          auto direction1_ = info->extract("direction1");
          auto& direction1Tmp = *direction1_->toListing();
          cnoid::Vector3 direction1 = cnoid::Vector3(direction1Tmp[0].toDouble(), direction1Tmp[1].toDouble(), direction1Tmp[2].toDouble());
          auto direction2_ = info->extract("direction2");
          auto& direction2Tmp = *direction2_->toListing();
          cnoid::Vector3 direction2 = cnoid::Vector3(direction2Tmp[0].toDouble(), direction2Tmp[1].toDouble(), direction2Tmp[2].toDouble());
          int num_dir1 = info->extract("num_dir1")->toInt();
          int num_dir2 = info->extract("num_dir2")->toInt();
          this->num_sensor += num_dir1 * num_dir2;
          for (int j=0; j < num_dir1; j++) {
            for (int k=0; k < num_dir2; k++) {
              sensor.positions.push_back(point + direction1 * j / num_dir1 + direction2 * k / num_dir2);
            }
          }
        }
        this->tactileSensorList.push_back(sensor);
      }
    }
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t TactileSensorROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();

  if(this->m_tactileSensorArrayIn_.isNew()){
    this->m_tactileSensorArrayIn_.read();
    if(this->m_tactileSensorArray_.data.length() != this->num_sensor*3) {
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "data length is different" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(this->num_sensor);
    int sensor_id = 0;
    for (int i=0; i < this->tactileSensorList.size(); i++) {
      for (int j=0; j < this->tactileSensorList[i].positions.size(); j++) {
        //marker array arrow
        marker_array.markers[sensor_id].header.frame_id = "/map";//this->tactileSensorList[i].linkName;
        marker_array.markers[sensor_id].header.stamp = ros::Time::now();
        marker_array.markers[sensor_id].ns = "tactile_sensor_arrow";
        marker_array.markers[sensor_id].id = sensor_id;
        marker_array.markers[sensor_id].lifetime = ros::Duration();
        marker_array.markers[sensor_id].type = visualization_msgs::Marker::ARROW;
        marker_array.markers[sensor_id].action = visualization_msgs::Marker::ADD;
        marker_array.markers[sensor_id].scale = this->arrow;

        marker_array.markers[sensor_id].points.resize(2);
        // start
        geometry_msgs::Point v_start;
        v_start.x = this->tactileSensorList[i].positions[j][0];
        v_start.y = this->tactileSensorList[i].positions[j][1];
        v_start.z = this->tactileSensorList[i].positions[j][2];
        // end
        geometry_msgs::Point v_end;
        v_end.x = v_start.x + this->m_tactileSensorArray_.data[3*sensor_id + 0] * 100;
        v_end.y = v_start.y + this->m_tactileSensorArray_.data[3*sensor_id + 1] * 100;
        v_end.z = v_start.z + this->m_tactileSensorArray_.data[3*sensor_id + 2] * 100;
        marker_array.markers[sensor_id].points[0] = v_start;
        marker_array.markers[sensor_id].points[1] = v_end;
        //color
        marker_array.markers[sensor_id].color.r = 1.0f;
        marker_array.markers[sensor_id].color.g = 0.0f;
        marker_array.markers[sensor_id].color.b = 0.0f;
        marker_array.markers[sensor_id].color.a = 1.0f;
        sensor_id++;
      }
    }
    this->tactile_sensor_array_pub.publish(marker_array);
  }
  return RTC::RTC_OK;
}


static const char* TactileSensorROSBridge_spec[] = {
  "implementation_id", "TactileSensorROSBridge",
  "type_name",         "TactileSensorROSBridge",
  "description",       "TactileSensorROSBridge component",
  "version",           "0.0",
  "vendor",            "Takuma-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

extern "C"{
    void TactileSensorROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(TactileSensorROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<TactileSensorROSBridge>, RTC::Delete<TactileSensorROSBridge>);
    }
};
