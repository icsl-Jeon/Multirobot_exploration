#include "search_position.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <ros/ros.h>

#include <sstream>

namespace rqt_search_position
{

SearchPositionPlugin::SearchPositionPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  setObjectName("SearchPositionPlugin");
}

void SearchPositionPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  position_sub = ros_node_handle.subscribe("search_position", 1, &SearchPositionPlugin::position_data_callback, this);
  position_array_pub = ros_node_handle.advertise<kiro_gui_msgs::PositionArray>("search_position_array", 1000);

  QObject::connect( ui_.sendButton, SIGNAL(clicked()), this, SLOT(sendMsg()));
  QObject::connect( ui_.initButton, SIGNAL(clicked()), this, SLOT(initList()));
  QObject::connect( this, SIGNAL(setText(const QString)), ui_.Statuslabel, SLOT(setText(const QString)));
  QObject::connect( this, SIGNAL(setStyleSheet(const QString)), ui_.Statuslabel, SLOT(setStyleSheet(const QString)));

}

void SearchPositionPlugin::shutdownPlugin()
{
  // unregister all publishers here
}

void SearchPositionPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void SearchPositionPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  // v = instance_settings.value(k)
}


void SearchPositionPlugin::position_data_callback(const geometry_msgs::PoseStamped message) {

   int count = ui_.listWidget->count();


   QString str_pos_ori;
   str_pos_ori = QString::number(count+1);
   str_pos_ori += ": ";
   str_pos_ori += convertPositionToQString(message.pose.position.x,message.pose.position.y,message.pose.position.z,
                             message.pose.orientation.x,message.pose.orientation.y,message.pose.orientation.z,message.pose.orientation.w);


   ui_.listWidget->insertItem(count, str_pos_ori);

   positions_msgs.positions.push_back(message);

}
QString SearchPositionPlugin::convertPositionToQString(float pos_x, float pos_y, float pos_z, float ori_x, float ori_y, float ori_z, float ori_w){
  setNormalMsg("Getting Position");

  QString result;
  result = "Position: ";
  result += QString::number(pos_x);
  result += ", ";
  result += QString::number(pos_y);
  result += ", ";
  result += QString::number(pos_z);
  result += " | Orientation: ";
  result += QString::number(ori_x);
  result += ", ";
  result += QString::number(ori_y);
  result += ", ";
  result += QString::number(ori_z);
  result += ", ";
  result += QString::number(ori_w);

  return result;


}

void SearchPositionPlugin::sendMsg(){
  if(positions_msgs.positions.empty()!=0){
    setWarringMsg("No Positions");

  }else{
     setNormalMsg("Publishing Positions");
     position_array_pub.publish(positions_msgs);
  }
}

void SearchPositionPlugin::setWarringMsg(QString str){
  emit setText(str);
  emit setStyleSheet("color: black; background-color: red;");
}

void SearchPositionPlugin::setNormalMsg(QString str){
  emit setText(str);
  emit setStyleSheet("color: black; background-color: green;");
}
void SearchPositionPlugin::initList(){
    setNormalMsg("Init Positions");
    ui_.listWidget->clear();
    positions_msgs.positions.clear();
}

}

PLUGINLIB_DECLARE_CLASS(rqt_search_position, SearchPositionPlugin, rqt_search_position::SearchPositionPlugin, rqt_gui_cpp::Plugin)


