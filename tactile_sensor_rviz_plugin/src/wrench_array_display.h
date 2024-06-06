#ifndef TACTILE_SENSOR_RVIZ_PLUGIN_WRENCHSTAMPEDARRAY_DISPLAY_H
#define TACTILE_SENSOR_RVIZ_PLUGIN_WRENCHSTAMPEDARRAY_DISPLAY_H

#include <deque>
#include <rviz/message_filter_display.h>
#include <thread>
#include <mutex>
#include <tactile_sensor_msgs/WrenchStampedArray.h>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class ROSTopicStringProperty;
class FloatProperty;
class IntProperty;
}

namespace rviz
{
  class WrenchVisual;
}

namespace tactile_sensor_rviz_plugin
{

class WrenchStampedArrayDisplay: public rviz::MessageFilterDisplay<tactile_sensor_msgs::WrenchStampedArray>
{
    Q_OBJECT
public:
    // Constructor.  pluginlib::ClassLoader creates instances by calling
    // the default constructor, so make sure you have one.
    WrenchStampedArrayDisplay();
    virtual ~WrenchStampedArrayDisplay();

protected:
    // Overrides of public virtual functions from the Display class.
    virtual void onInitialize();
    virtual void reset();

private Q_SLOTS:
    // Helper function to apply color and alpha to all visuals.
    void updateColorAndAlpha();
    void updateHistoryLength();

private:
  // Function to handle an incoming ROS message.
  void processMessage( const tactile_sensor_msgs::WrenchStampedArray::ConstPtr& msg );

  // Storage for the list of visuals par each joint intem
  // Storage for the list of visuals.  It is a circular buffer where
  // data gets popped from the front (oldest) and pushed to the back (newest)
  //注意!! rviz::WrenchVisualはshared_prtの状態で扱うこと。解体する際に、rviz側でまだ利用中の場合に、突然プログラムが落ちる。
  std::deque<boost::shared_ptr<std::pair<std::mutex, std::vector<boost::shared_ptr<rviz::WrenchVisual> > > > > visuals_;
  std::mutex mutex_;

  // Property objects for user-editable properties.
  rviz::ColorProperty *force_color_property_, *torque_color_property_;
  rviz::FloatProperty *alpha_property_, *force_scale_property_, *torque_scale_property_, *width_property_;
  rviz::IntProperty *history_length_property_;
  rviz::BoolProperty* hide_small_values_property_;
};
} // end namespace rviz_plugin_tutorials

#endif

