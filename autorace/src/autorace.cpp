#include <pluginlib/class_list_macros.h>
#include "autorace/nodelet_autorace.h"
#include "autorace/nodelet_pcam.h"
#include "autorace/nodelet_usbcam.h"
#include "autorace/nodelet_signal.h"

PLUGINLIB_EXPORT_CLASS(autorace::NodeletAutorace, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(autorace::NodeletPcam, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(autorace::NodeletUsbcam, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(autorace::NodeletSignal, nodelet::Nodelet)