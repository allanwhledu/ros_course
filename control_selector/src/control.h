#ifndef CONTROL_H
#define CONTROL_H
#include <vector>

//абстрактный класс - интерфейс системы управления
class Control
{
protected:
    //в абстрактном классе нет данных
public:
    //абстрактные функции интерфейса

    //установка данных лазера
    virtual void setLaserData(const std::vector<float>& data) = 0;

    //установка текущей позиции робота
    virtual void setRobotPose(double x, double y, double theta) = 0;

    //получение управления
    virtual void getControl(double& v, double& w)  = 0;

    //виртуальный деструктор
    virtual ~Control() {}

};

#endif // CONTROL_H
