/**
 * @file jose_gui.cpp
 */
#include <jose_gui/jose_gui.h>
#include <filesystem>

namespace jose
{

JoseGui::JoseGui(QWidget* parent)
  : rviz::Panel(parent),
    ui_(new Ui::Jose)
{
  init();
}

JoseGui::~JoseGui()
{
  delete ui_;
}

void JoseGui::init()
{
  ui_->setupUi(this);
  this->setEnabled(true);

  setupWidgets();
}

void JoseGui::setupWidgets()
{
  QObject::connect(ui_->ik_button, SIGNAL(clicked()), this,
		   SLOT(go()), Qt::UniqueConnection);
}

void JoseGui::go()
{
  printf("IK..... \n");
}

void JoseGui::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}


void JoseGui::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}
  

  
} // end jose

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jose::JoseGui, rviz::Panel)
