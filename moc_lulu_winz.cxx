/****************************************************************************
** Meta object code from reading C++ file 'lulu_winz.hpp'
**
** Created: Mon Oct 7 20:33:55 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "lulu_winz.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'lulu_winz.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_LuluWinz[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      10,    9,    9,    9, 0x08,
      23,    9,    9,    9, 0x08,
      42,   36,    9,    9, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_LuluWinz[] = {
    "LuluWinz\0\0sick1Event()\0sick2Event()\0"
    "state\0stateChangedListener(rw::kinematics::State)\0"
};

void LuluWinz::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        LuluWinz *_t = static_cast<LuluWinz *>(_o);
        switch (_id) {
        case 0: _t->sick1Event(); break;
        case 1: _t->sick2Event(); break;
        case 2: _t->stateChangedListener((*reinterpret_cast< const rw::kinematics::State(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData LuluWinz::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject LuluWinz::staticMetaObject = {
    { &rws::RobWorkStudioPlugin::staticMetaObject, qt_meta_stringdata_LuluWinz,
      qt_meta_data_LuluWinz, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &LuluWinz::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *LuluWinz::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *LuluWinz::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_LuluWinz))
        return static_cast<void*>(const_cast< LuluWinz*>(this));
    if (!strcmp(_clname, "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1"))
        return static_cast< rws::RobWorkStudioPlugin*>(const_cast< LuluWinz*>(this));
    typedef rws::RobWorkStudioPlugin QMocSuperClass;
    return QMocSuperClass::qt_metacast(_clname);
}

int LuluWinz::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    typedef rws::RobWorkStudioPlugin QMocSuperClass;
    _id = QMocSuperClass::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
