/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the Qt Charts module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:GPL$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 3 or (at your option) any later version
** approved by the KDE Free Qt Foundation. The licenses are as published by
** the Free Software Foundation and appearing in the file LICENSE.GPL3
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef VIEW_H
#define VIEW_H
#include <QtWidgets/QGraphicsView>
#include <QtCharts/QChartGlobal>
#include <QMap>

QT_BEGIN_NAMESPACE
class QGraphicsScene;
class QMouseEvent;
class QResizeEvent;
QT_END_NAMESPACE

QT_CHARTS_BEGIN_NAMESPACE
class QChart;
class QLineSeries;
class QScatterSeries;
QT_CHARTS_END_NAMESPACE

class Callout;

QT_CHARTS_USE_NAMESPACE


class TrajectoryView: public QGraphicsView
{
    Q_OBJECT

public:
    TrajectoryView(QWidget *parent = 0);

    void setXRange(double min, double max);
    void setYRange(double min, double max);

    void appendData(qreal ts, qreal pos, qreal velocity, qreal accel, qreal jerk);

    void setMaxVelocityAccel(qreal ts, qreal velocity, qreal accel);

    void clear();
    void setJerkVisible(bool visible);

protected:
    void resizeEvent(QResizeEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;

    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *pEvent) override;

public slots:
    void tooltip(QPointF point, bool state);

private:

    void updateCalloutGeometry();

private:
    QGraphicsSimpleTextItem *m_coordX               = nullptr;
    QGraphicsSimpleTextItem *m_coordY               = nullptr;

    QChart                  *m_chart                = nullptr;

    Callout                 *m_callout              = nullptr;
    QList<Callout *>        m_callouts;


    bool                    m_mousePressed          = false;
    QPoint                  m_dragStartPos          = {0, 0};

    QLineSeries             * m_posSeries           = nullptr;
    QLineSeries             * m_velocitySeries      = nullptr;
    QLineSeries             * m_accelSeries         = nullptr;
    QLineSeries             * m_jerkSeries          = nullptr;

    QScatterSeries             * m_maxVelocitySeries   = nullptr;
    QScatterSeries             * m_maxAccelSeries      = nullptr;

    qreal                   m_interval              = 0.01;
};

#endif
