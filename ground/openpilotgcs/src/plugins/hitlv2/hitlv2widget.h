/**
 ******************************************************************************
 *
 * @file       hitlv2widget.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010-2012.
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup HITLPlugin HITLv2 Plugin
 * @{
 * @brief The Hardware In The Loop plugin version 2
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef HITLV2WIDGET_H
#define HITLV2WIDGET_H

#include <QtGui/QWidget>
#include <QProcess>
#include "simulatorv2.h"

class Ui_HITLWidget;

class HITLWidget : public QWidget
{
    Q_OBJECT

public:
    HITLWidget(QWidget *parent = 0);
    ~HITLWidget();

    void setSettingParameters(const SimulatorSettings& params) { settings = params; }

signals:
    void deleteSimulator();

private slots:
    void startButtonClicked();
    void stopButtonClicked();
    void buttonClearLogClicked();
    void onProcessOutput(QString text);
    void onAutopilotConnect();
    void onAutopilotDisconnect();
    void onSimulatorConnect();
    void onSimulatorDisconnect();

private:
    Ui_HITLWidget* widget;
    Simulator* simulator;
    SimulatorSettings settings;

    QString strAutopilotDisconnected;
    QString strSimulatorDisconnected;
    QString strAutopilotConnected;
//    QString strSimulatorConnected;
    QString strStyleEnable;
    QString strStyleDisable;
};
#endif /* HITLV2WIDGET_H */
