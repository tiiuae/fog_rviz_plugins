// clang: MatousFormat

#ifndef BUMPER_DISPLAY_H
#define BUMPER_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>

#include <rviz_common/message_filter_display.hpp>
#include <fog_msgs/msg/obstacle_sectors.hpp>
#endif

namespace Ogre
{
  class SceneNode;
}

namespace rviz_common
{
  namespace properties
  {
    class ColorProperty;
    class EnumProperty;
    class FloatProperty;
    class IntProperty;
    class BoolProperty;
  }  // namespace properties
}  // namespace rviz_common

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace fog_rviz_plugins
{

  namespace bumper
  {

    class Visual;

    class Display : public rviz_common::MessageFilterDisplay<fog_msgs::msg::ObstacleSectors>
    {
      Q_OBJECT
    public:
      // Constructor.  pluginlib::ClassLoader creates instances by calling
      // the default constructor, so make sure you have one.
      Display();
      virtual ~Display();

      // Overrides of protected virtual functions from Display.  As much
      // as possible, when Displays are not enabled, they should not be
      // subscribed to incoming data and should not show anything in the
      // 3D view.  These functions are where these connections are made
      // and broken.
    protected:
      virtual void onInitialize();

      // A helper to clear this display back to the initial state.
      virtual void reset();

      // These Qt slots get connected to signals indicating changes in the user-editable properties.
    private Q_SLOTS:
      void updateColorAndAlpha();
      void updateHistoryLength();
      void updateDisplayMode();
      void updateShowUndetected();
      void updateShowNoData();
      void updateCollisions();

      // Function to handle an incoming ROS message.
    private:
      void processMessage(fog_msgs::msg::ObstacleSectors::ConstSharedPtr msg);

      // Storage for the list of visuals.  It is a circular buffer where
      // data gets popped from the front (oldest) and pushed to the back (newest)
      boost::circular_buffer<boost::shared_ptr<Visual>> visuals_;

      // User-editable property variables.
      rviz_common::properties::ColorProperty* color_property_;
      rviz_common::properties::FloatProperty* alpha_property_;
      rviz_common::properties::BoolProperty* collision_colorize_property_;
      rviz_common::properties::FloatProperty* horizontal_collision_threshold_property_;
      rviz_common::properties::FloatProperty* vertical_collision_threshold_property_;
      rviz_common::properties::FloatProperty* collision_alpha_property_;
      rviz_common::properties::ColorProperty* collision_color_property_;
      rviz_common::properties::IntProperty* history_length_property_;
      rviz_common::properties::EnumProperty* display_mode_property_;
      rviz_common::properties::BoolProperty* show_undetected_property_;
      rviz_common::properties::BoolProperty* show_no_data_property_;
    };

  }  // namespace bumper

}  // end namespace fog_rviz_plugins

#endif
