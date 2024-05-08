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

#include "TrajectoryView.h"
#include <QtGui/QResizeEvent>
#include <QtWidgets/QGraphicsScene>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QSplineSeries>
#include <QtWidgets/QGraphicsTextItem>
#include "callout.h"
#include <QtGui/QMouseEvent>
#include <iostream>
#include <QtCharts/QScatterSeries>
#include <QDebug>

namespace  {
    bool findSuitablePoint(QList<QPointF> points, qreal x, QPointF & output)
    {
        int last = 0;
        for (int index = 0; index < points.size(); ++index)
        {
            const auto point = points.at(index);
            const auto lastPoint = points.at(last);
            if ( x == point.x() || (lastPoint.x() < x && x < point.x()) ) {
                output = points.at(index);
                return true;
            }
            last = index;
        }
        return false;
    }
}

TrajectoryView::TrajectoryView(QWidget *parent)
    : QGraphicsView(new QGraphicsScene, parent)
{
    setDragMode(QGraphicsView::NoDrag);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    // chart
    m_chart = new QChart;
    m_chart->setMinimumSize(640, 480);
    m_chart->setTitle("Hover the line to show callout. Click the line to make it stay");
    m_chart->legend();

    m_posSeries = new QLineSeries;
    m_posSeries->setName("Position");
    m_chart->addSeries(m_posSeries);

    m_velocitySeries = new QLineSeries;
    m_velocitySeries->setName("Velocity");
    m_chart->addSeries(m_velocitySeries);

    m_accelSeries = new QLineSeries;
    m_accelSeries->setName("Acceleration");
    m_chart->addSeries(m_accelSeries);

    m_jerkSeries = new QLineSeries;
    m_jerkSeries->setName("Jerk");
    m_chart->addSeries(m_jerkSeries);


    {
        m_maxVelocitySeries = new QScatterSeries;
        m_maxVelocitySeries->setName("Max Abs Velocity");
        m_maxVelocitySeries->setMarkerSize(7.0);

        auto velocityColor = m_velocitySeries->color();
        m_maxVelocitySeries->setColor(velocityColor);
        m_maxVelocitySeries->setMarkerShape(QScatterSeries::MarkerShapeCircle);
        m_chart->addSeries(m_maxVelocitySeries);
    }
    {
        m_maxAccelSeries = new QScatterSeries;
        m_maxAccelSeries->setName("Max Abs Acceleration");
        m_maxAccelSeries->setMarkerSize(5.0);

        auto accelColor = m_accelSeries->color();
        m_maxAccelSeries->setPen(QPen(accelColor));

        accelColor.setAlpha(100);
        m_maxAccelSeries->setColor(accelColor);
        m_maxAccelSeries->setMarkerShape(QScatterSeries::MarkerShapeRectangle);

        m_chart->addSeries(m_maxAccelSeries);
    }


    const auto seriesList = m_chart->series();
    for (int i = 0; i < seriesList.size(); ++i) {
        auto s = dynamic_cast<QXYSeries*>(seriesList[i]);
//        connect(s, &QXYSeries::clicked, this, &TrajectoryView::keepCallout);
        connect(s, &QXYSeries::hovered, this, &TrajectoryView::tooltip);
    }

    m_chart->createDefaultAxes();
    m_chart->setAcceptHoverEvents(true);

    m_chart->axes(Qt::Vertical).first()->setRange(-400, 400);
    m_chart->axes(Qt::Horizontal).first()->setRange(0, 4);

    setRenderHint(QPainter::Antialiasing);
    scene()->addItem(m_chart);

    m_coordX = new QGraphicsSimpleTextItem(m_chart);
    m_coordX->setPos(m_chart->size().width()/2 - 50, m_chart->size().height());
    m_coordX->setText("X: ");
    m_coordY = new QGraphicsSimpleTextItem(m_chart);
    m_coordY->setPos(m_chart->size().width()/2 + 50, m_chart->size().height());
    m_coordY->setText("Y: ");

    this->setMouseTracking(true);
}

void TrajectoryView::setXRange(double min, double max)
{
    m_chart->axes(Qt::Horizontal).first()->setRange(min, max);

    const auto range = max - min;

    m_interval = range / 100.0;

    qDebug() << "interval = " << m_interval;

}

void TrajectoryView::setYRange(double min, double max)
{
    m_chart->axes(Qt::Vertical).first()->setRange(min, max);
}

void TrajectoryView::appendData(qreal ts, qreal pos, qreal velocity, qreal accel, qreal jerk)
{

    if (m_posSeries->count() > 0) {
        const auto point = m_posSeries->at(m_posSeries->count()-1);
        if (ts - point.x() < m_interval) {
            return;
        }
    }

    m_posSeries->append(ts, pos);
    m_velocitySeries->append(ts, velocity);
    m_accelSeries->append(ts, accel);
    m_jerkSeries->append(ts, jerk);

}

void TrajectoryView::setMaxVelocityAccel(qreal ts, qreal velocity, qreal accel)
{
    if (m_maxVelocitySeries->count() > 0) {
        const auto point = m_maxVelocitySeries->at(m_maxVelocitySeries->count()-1);
        if (ts - point.x() < m_interval) {
            return;
        }
    }

    m_maxVelocitySeries->append(ts, velocity);
    m_maxAccelSeries->append(ts, accel);
}

void TrajectoryView::clear()
{
    for (auto series :  m_chart->series()) {
        auto lineSeries = dynamic_cast<QXYSeries*>(series);
        lineSeries->clear();
    }

    for (auto callout : m_callouts) {
        delete callout;
    }

    m_callouts.clear();
}

void TrajectoryView::updateCalloutGeometry()
{
    const auto callouts = m_callouts;
    for (Callout *callout : callouts)
        callout->updateGeometry();
}
void TrajectoryView::resizeEvent(QResizeEvent *event)
{
    if (scene()) {
        scene()->setSceneRect(QRect(QPoint(0, 0), event->size()));
         m_chart->resize(event->size());
         m_coordX->setPos(m_chart->size().width()/2 - 50, m_chart->size().height() - 20);
         m_coordY->setPos(m_chart->size().width()/2 + 50, m_chart->size().height() - 20);
         updateCalloutGeometry();
    }
    QGraphicsView::resizeEvent(event);
}

void TrajectoryView::mousePressEvent(QMouseEvent *event)
{
//    std::cout << "TrajectoryView::mousePressEvent" << std::endl;

    if (m_callout && m_callout->isVisible()) {
        m_callouts.append(m_callout);
        m_callout = new Callout(m_chart);
        m_callout->hide();
        std::cout << "callout size=" << m_callouts.size() << std::endl;
    }

    if (event->button() == (Qt::RightButton))
    {
        m_mousePressed = true;
        m_dragStartPos = event->pos();
        this->setCursor(Qt::OpenHandCursor);
    }

    QGraphicsView::mousePressEvent(event);
}

void TrajectoryView::mouseReleaseEvent(QMouseEvent *event)
{
//    std::cout << "TrajectoryView::mouseReleaseEvent" << std::endl;

    if (event->button() == Qt::RightButton )
    {
        m_mousePressed = false;
        this->setCursor(Qt::ArrowCursor);

        updateCalloutGeometry();
    }

    QGraphicsView::mouseMoveEvent(event);
}

void TrajectoryView::mouseMoveEvent(QMouseEvent *event)
{
//    std::cout << "TrajectoryView::mouseMoveEvent" << std::endl;

    m_coordX->setText(QString("X: %1").arg(m_chart->mapToValue(event->pos()).x()));
    m_coordY->setText(QString("Y: %1").arg(m_chart->mapToValue(event->pos()).y()));

    if (m_mousePressed)
    {
        QPoint deltaPos = event->pos() - m_dragStartPos;
        m_chart->scroll(-deltaPos.x(), deltaPos.y());
        m_dragStartPos = event->pos();

        updateCalloutGeometry();
    }

    QGraphicsView::mouseMoveEvent(event);
}

void TrajectoryView::wheelEvent(QWheelEvent *event)
{
    qreal rVal = std::pow(0.999, event->delta()); // 设置比例
    // 1. 读取视图基本信息
    QRectF oPlotAreaRect = m_chart->plotArea();
    QPointF oCenterPoint = oPlotAreaRect.center();
    // 2. 水平调整
    oPlotAreaRect.setWidth(oPlotAreaRect.width() * rVal);
    // 3. 竖直调整
    oPlotAreaRect.setHeight(oPlotAreaRect.height() * rVal);
    // 4.1 计算视点，视点不变，围绕中心缩放
    //QPointF oNewCenterPoint(oCenterPoint);
    // 4.2 计算视点，让鼠标点击的位置移动到窗口中心
    //QPointF oNewCenterPoint(event->pos());
    // 4.3 计算视点，让鼠标点击的位置尽量保持不动(等比换算，存在一点误差)
    QPointF oNewCenterPoint(2 * oCenterPoint - event->pos() - (oCenterPoint - event->pos()) / rVal);
    // 5. 设置视点
    oPlotAreaRect.moveCenter(oNewCenterPoint);
    // 6. 提交缩放调整
    m_chart->zoomIn(oPlotAreaRect);
    QGraphicsView::wheelEvent(event);

    updateCalloutGeometry();
}

void TrajectoryView::tooltip(QPointF point, bool state)
{
    if (m_callout == nullptr)
        m_callout = new Callout(m_chart);

    if (state) {

        auto* series = dynamic_cast<QXYSeries*>(sender());
        QPointF output;
        if ( findSuitablePoint(series->points(), point.x(), output) )
        {
            m_callout->setText(QString("%3:\nX: %1, Y: %2 ").arg(output.x()).arg(output.y()).arg(series->name()));
            m_callout->setAnchor(point);
            m_callout->setZValue(11);
            m_callout->updateGeometry();
            m_callout->show();
        }

    } else {
        m_callout->hide();
    }
}

void TrajectoryView::setJerkVisible(bool visible)
{
    if (visible) {
        m_jerkSeries->show();
    } else {
        m_jerkSeries->hide();
    }

    std::cout << "count:" << m_jerkSeries->count() << std::endl;
}
