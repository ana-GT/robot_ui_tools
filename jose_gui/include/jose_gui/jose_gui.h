#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <rviz/panel.h>

#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include "ui_jose_gui.h"

namespace Ui
{
  class JoseUi;
}

namespace jose
{

class JoseGui : public rviz::Panel
{
  Q_OBJECT

public:
  JoseGui(QWidget* parent = 0);
  ~JoseGui();

  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;
  
public Q_SLOTS:
  void go();
  
protected:
  
  void init();
  void setupWidgets();
 
  ros::NodeHandle nh_;
  Ui::Jose* ui_;
  
  QVBoxLayout* layout_;
};
 
} // namespace jose
