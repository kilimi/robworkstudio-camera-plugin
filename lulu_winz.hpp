#ifndef LULUWINZ_HPP
#define LULUWINZ_HPP

#include <string.h>

#include <rws/RobWorkStudioPlugin.hpp>

#include "ui_LuluWinz.h"

using namespace rw::kinematics;

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
    //QPushButton* _btn0,*_btn1;
    void createAndSavePCD(Frame *, std::string);

};

#endif /*LULUWINZ_HPP*/
