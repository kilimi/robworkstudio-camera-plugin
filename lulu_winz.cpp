
#include "lulu_winz.hpp"

#include <RobWorkStudio.hpp>

#include "ui_LuluWinz.h"

USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rws;

LuluWinz::LuluWinz() : RobWorkStudioPlugin("LuluWinz", QIcon(":/pa_icon.png"))
{
    /*QWidget* base = new QWidget(this);
    QGridLayout* pLayout = new QGridLayout(base);
    base->setLayout(pLayout);
    this->setWidget(base);

    int row = 0;

    _btn0 = new QPushButton("Sick 1");
    pLayout->addWidget(_btn0, row++, 0);
    connect(_btn0, SIGNAL(clicked()), this, SLOT(sick1Event()));

    _btn1 = new QPushButton("Sick 2");
    pLayout->addWidget(_btn1, row++, 0);
    connect(_btn1, SIGNAL(clicked()), this, SLOT(sick2Event()));

    pLayout->setRowStretch(row,1); */ 	


    setupUi(this);
    connect(myButton, SIGNAL(clicked()), this, SLOT(sick1Event()));
}

LuluWinz::~LuluWinz(){ /* deallocate used memory */ }
void LuluWinz::open(WorkCell* workcell){ /* do something when workcell is openned */}
void LuluWinz::close() { /* do something when the workcell is closed */}

void LuluWinz::initialize() {
    /* do something when plugin is initialized */
    getRobWorkStudio()->stateChangedEvent().add(
            boost::bind(&LuluWinz::stateChangedListener, this, _1), this);
}

void LuluWinz::stateChangedListener(const State& state) {
    log().info() << "State changed!";
}

void LuluWinz::sick1Event() {
    QObject *obj = sender();
    log().info() << "This is my sick button listener!\n";
}

void LuluWinz::sick2Event() {
    QObject *obj = sender();
    log().info() << "This is an other sick listener!\n";
}

Q_EXPORT_PLUGIN2(LuluWinz, LuluWinz);
