#ifndef KINEMATICS_THREAD_H
#define KINEMATICS_THREAD_H

#include <QThread>

template <typename Class>
class kinematics_thread : public QThread
{

public:

    kinematics_thread(Class& Obj, void (Class::*Func)())
        : Obj(Obj), Func(Func)
    {}

    void run()
    {
        (Obj.*Func)();
    }

private:

    Class& Obj;
    void (Class::*Func)();

signals:

};

#endif // KINEMATICS_THREAD_H
