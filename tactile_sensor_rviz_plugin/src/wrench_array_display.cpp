#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/parse_color.h>
#include <rviz/validate_floats.h>
#include <rviz/default_plugin/wrench_visual.h>

#include <boost/foreach.hpp>
#include "wrench_array_display.h"

namespace tactile_sensor_rviz_plugin
{

WrenchStampedArrayDisplay::WrenchStampedArrayDisplay()
{
    force_color_property_ =
            new rviz::ColorProperty( "Force Color", QColor( 204, 51, 51 ),
                                     "Color to draw the force arrows.",
                                     this, SLOT( updateColorAndAlpha() ));

    torque_color_property_ =
            new rviz::ColorProperty( "Torque Color", QColor( 204, 204, 51),
                                     "Color to draw the torque arrows.",
                                     this, SLOT( updateColorAndAlpha() ));

    alpha_property_ =
            new rviz::FloatProperty( "Alpha", 1.0,
                                     "0 is fully transparent, 1.0 is fully opaque.",
                                     this, SLOT( updateColorAndAlpha() ));

    force_scale_property_ =
            new rviz::FloatProperty( "Force Arrow Scale", 2.0,
                                     "force arrow scale",
                                     this, SLOT( updateColorAndAlpha() ));

    torque_scale_property_ =
            new rviz::FloatProperty( "Torque Arrow Scale", 2.0,
                                     "torque arrow scale",
                                     this, SLOT( updateColorAndAlpha() ));

    width_property_ =
            new rviz::FloatProperty( "Arrow Width", 0.5,
                                     "arrow width",
                                     this, SLOT( updateColorAndAlpha() ));


    history_length_property_ =
            new rviz::IntProperty( "History Length", 1,
                                   "Number of prior measurements to display.",
                                   this, SLOT( updateHistoryLength() ));

    hide_small_values_property_ =
            new rviz::BoolProperty("Hide Small Values", true, "Hide small values",
                                   this, SLOT( updateColorAndAlpha() ));

    history_length_property_->setMin( 1 );
    history_length_property_->setMax( 100000 );
}

void WrenchStampedArrayDisplay::onInitialize()
{
    MFDClass::onInitialize();
    updateHistoryLength( );
}

WrenchStampedArrayDisplay::~WrenchStampedArrayDisplay()
{
}

// Override rviz::Display's reset() function to add a call to clear().
void WrenchStampedArrayDisplay::reset()
{
    MFDClass::reset();
    visuals_.clear();
}

void WrenchStampedArrayDisplay::updateColorAndAlpha()
{
    float alpha = alpha_property_->getFloat();
    float force_scale = force_scale_property_->getFloat();
    float torque_scale = torque_scale_property_->getFloat();
    float width = width_property_->getFloat();
    bool hide_small_values = hide_small_values_property_->getBool();
    Ogre::ColourValue force_color = force_color_property_->getOgreColor();
    Ogre::ColourValue torque_color = torque_color_property_->getOgreColor();

    for( size_t i = 0; i < visuals_.size(); i++ )
    {
      for(size_t j = 0; j< visuals_[i]->second.size(); j++){
        (*visuals_[i]).second[j]->setForceColor( force_color.r, force_color.g, force_color.b, alpha );
        (*visuals_[i]).second[j]->setTorqueColor( torque_color.r, torque_color.g, torque_color.b, alpha );
        (*visuals_[i]).second[j]->setForceScale( force_scale );
        (*visuals_[i]).second[j]->setTorqueScale( torque_scale );
        (*visuals_[i]).second[j]->setWidth( width );
        (*visuals_[i]).second[j]->setHideSmallValues( hide_small_values );
      }
    }
}

// Set the number of past visuals to show.
void WrenchStampedArrayDisplay::updateHistoryLength()
{
  //visuals_.rset_capacity(history_length_property_->getInt());
  if (visuals_.size()>history_length_property_->getInt()){
    for(int i=0;i<history_length_property_->getInt()-visuals_.size();i++){
      visuals_.pop_front();
    }
  }
}

inline bool validateFloats( const geometry_msgs::WrenchStamped& msg )
{
    return rviz::validateFloats(msg.wrench.force) && rviz::validateFloats(msg.wrench.torque) ;
}

// This is our callback to handle an incoming message.
void WrenchStampedArrayDisplay::processMessage( const tactile_sensor_msgs::WrenchStampedArray::ConstPtr& msg )
{
  boost::shared_ptr<std::pair<std::mutex, std::vector<boost::shared_ptr<rviz::WrenchVisual> > > > visuals;
  std::unique_lock<std::mutex> lock;
  {
    std::lock_guard<std::mutex> guard(this->mutex_);
    if( visuals_.size()>=history_length_property_->getInt() ) {
      visuals = visuals_.front();
      lock = std::unique_lock<std::mutex>(visuals->first, std::try_to_lock);
      if(!lock.owns_lock()) return; // 複数threadでprocessMessageを実行しているケース
      visuals_.pop_front();
      visuals_.push_back(visuals);
    }else {
      visuals.reset(new std::pair<std::mutex, std::vector<boost::shared_ptr<rviz::WrenchVisual> > >{});
      lock = std::unique_lock<std::mutex>(visuals->first);
      visuals_.push_back(visuals);
    }
  }
  int idx = 0;
  for (int i=0;i<msg->wrenchstampeds.size();i++){
    if( !validateFloats( msg->wrenchstampeds[i] ))
      {
        setStatus( rviz::StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
        continue;
      }

    // Here we call the rviz::FrameManager to get the transform from the
    // fixed frame to the frame in the header of this Imu message.  If
    // it fails, we can't do anything else so we return.
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if( !context_->getFrameManager()->getTransform( msg->wrenchstampeds[i].header.frame_id,
                                                    msg->wrenchstampeds[i].header.stamp,
                                                    position, orientation ))
      {
        //ROS_ERROR( "Error transforming from frame '%s' to frame '%s'",
        ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
                   msg->wrenchstampeds[i].header.frame_id.c_str(), qPrintable( fixed_frame_ ));
        continue;
      }

    if ( position.isNaN() )
      {
        ROS_ERROR_THROTTLE(1.0, "Wrench position contains NaNs. Skipping render as long as the position is invalid");
        continue;
    }

    if(visuals->second.size() <= idx) {
      visuals->second.push_back(boost::shared_ptr<rviz::WrenchVisual>(new rviz::WrenchVisual{context_->getSceneManager(), scene_node_ }));
    }
    // Now set or update the contents of the chosen visual.
    visuals->second[idx]->setVisible(true); // setVisible()を呼ぶとサイズの情報などが初期化されることに注意.
    visuals->second[idx]->setWrench( msg->wrenchstampeds[i].wrench );
    visuals->second[idx]->setFramePosition( position );
    visuals->second[idx]->setFrameOrientation( orientation );
    float alpha = alpha_property_->getFloat();
    float force_scale = force_scale_property_->getFloat();
    float torque_scale = torque_scale_property_->getFloat();
    float width = width_property_->getFloat();
    bool hide_small_values = hide_small_values_property_->getBool();
    Ogre::ColourValue force_color = force_color_property_->getOgreColor();
    Ogre::ColourValue torque_color = torque_color_property_->getOgreColor();
    visuals->second[idx]->setForceColor( force_color.r, force_color.g, force_color.b, alpha );
    visuals->second[idx]->setTorqueColor( torque_color.r, torque_color.g, torque_color.b, alpha );
    visuals->second[idx]->setForceScale( force_scale );
    visuals->second[idx]->setTorqueScale( torque_scale );
    visuals->second[idx]->setWidth( width );
    visuals->second[idx]->setHideSmallValues( hide_small_values );
    idx++;
  }

  // 毎周期rviz::WrenchVisualの生成/解体を繰り返すと画面がちらつく
  for(int i = idx; i<visuals->second.size();i++){
    visuals->second[i]->setVisible(false);
  }
}

} // end namespace tactile_sensor_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( tactile_sensor_rviz_plugin::WrenchStampedArrayDisplay, rviz::Display )

