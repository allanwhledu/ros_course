#include "ros/ros.h"
#include "line_control.h"


int main(int argc, char **argv)
{
  /**
   * Инициализация системы сообщений ros
   * Регистрация node с определенным именем (третий аргумент функции)
   * Эта функция должна быть вызвана в первую очередь
   */
  ros::init(argc, argv, "control_node");

  /*
   * Создание объекта LineControl который выполняет всю работу
   * */
  LineControl Control;
  ROS_INFO_STREAM("go to spin");
  ros::spin();
}

