#ifndef LULUWINZ_HPP
#define LULUWINZ_HPP

#include <rws/RobWorkStudioPlugin.hpp>

#include "ui_LuluWinz.h"

class LuluWinz: public rws::RobWorkStudioPlugin, private Ui::LuluWinz
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
public:
    LuluWinz();
    virtual ~LuluWinz();

    // functions inherited from RobworkStudioPlugin, are typically used but can be optional
    virtual void open(rw::models::WorkCell* workcell);
    virtual void close();
    virtual void initialize();

private slots:
    void sick1Event();
    void sick2Event();

    void stateChangedListener(const rw::kinematics::State& state);
private:
    QPushButton* _btn0,*_btn1;
};

#endif /*LULUWINZ_HPP*/
