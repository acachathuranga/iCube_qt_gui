#include "icube_gui/ui/settings_screen.h"
#include "ui_settings_screen.h"

SettingsScreen::SettingsScreen(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SettingsScreen)
{
    ui->setupUi(this);
}

SettingsScreen::~SettingsScreen()
{
    delete ui;
}

void SettingsScreen::showEvent(QShowEvent *event)
{
    QWidget::showEvent(event);
}

void SettingsScreen::hideEvent(QHideEvent *event)
{
    QWidget::hideEvent(event);
}
