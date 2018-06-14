#ifndef SEARCH_POSITION_H
#define SEARCH_POSITION_H

#include <rqt_gui_cpp/plugin.h>
#include <rqt_search_position/ui_search_position.h>
#include <QWidget>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <QString>
#include <kiro_gui_msgs/PositionArray.h>

namespace rqt_search_position
{

class SearchPositionPlugin
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  SearchPositionPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();

private:
  Ui::SearchPositionPluginWidget ui_;
  QWidget* widget_;

  ros::NodeHandle ros_node_handle;
  ros::Subscriber position_sub;
  ros::Publisher position_array_pub;
  kiro_gui_msgs::PositionArray positions_msgs;
  void position_data_callback(const geometry_msgs::PoseStamped message);
  void setWarringMsg(QString str);
  void setNormalMsg(QString str);
  QString convertPositionToQString(float pos_x,float pos_y, float pos_z, float ori_x, float ori_y, float ori_z, float ori_w );

Q_SIGNALS:
  void clicked();
  void setText(const QString str);
  void setStyleSheet(const QString str);

private Q_SLOTS:
    void sendMsg();
    void initList();

};
}  // namespace rqt_search_position

#endif // SEARCH_POSITION_H
