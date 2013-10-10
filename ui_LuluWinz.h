/********************************************************************************
** Form generated from reading UI file 'LuluWinz.ui'
**
** Created: Tue Oct 8 15:10:31 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LULUWINZ_H
#define UI_LULUWINZ_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDockWidget>
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_LuluWinz
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QPushButton *myButton;
    QPushButton *run;
    QCheckBox *_checkBox;

    void setupUi(QDockWidget *LuluWinz)
    {
        if (LuluWinz->objectName().isEmpty())
            LuluWinz->setObjectName(QString::fromUtf8("LuluWinz"));
        LuluWinz->resize(468, 183);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        myButton = new QPushButton(dockWidgetContents);
        myButton->setObjectName(QString::fromUtf8("myButton"));

        verticalLayout->addWidget(myButton);

        run = new QPushButton(dockWidgetContents);
        run->setObjectName(QString::fromUtf8("run"));

        verticalLayout->addWidget(run);

        _checkBox = new QCheckBox(dockWidgetContents);
        _checkBox->setObjectName(QString::fromUtf8("_checkBox"));

        verticalLayout->addWidget(_checkBox);


        verticalLayout_2->addLayout(verticalLayout);

        LuluWinz->setWidget(dockWidgetContents);

        retranslateUi(LuluWinz);

        QMetaObject::connectSlotsByName(LuluWinz);
    } // setupUi

    void retranslateUi(QDockWidget *LuluWinz)
    {
        LuluWinz->setWindowTitle(QApplication::translate("LuluWinz", "DockWidget", 0, QApplication::UnicodeUTF8));
        myButton->setText(QApplication::translate("LuluWinz", "Load Scene", 0, QApplication::UnicodeUTF8));
        run->setText(QApplication::translate("LuluWinz", "Run", 0, QApplication::UnicodeUTF8));
        _checkBox->setText(QApplication::translate("LuluWinz", "CheckBox", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class LuluWinz: public Ui_LuluWinz {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LULUWINZ_H
