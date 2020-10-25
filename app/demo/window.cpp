
#include "window.h"

#include <Params.hpp>

#include <QCheckBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QMenu>
#include <QPushButton>
#include <QRadioButton>
#include <QDoubleSpinBox>
#include <QtCharts>

Window::Window(QWidget *parent)
  : QWidget(parent)
{
  QGridLayout *grid = new QGridLayout;
  grid->addWidget(createParametersGroup(), 0, 0);
  grid->addWidget(createControllerOperationsGroup(), 1, 0);
  grid->addWidget(createSpeedPlotGroup(), 0, 1);
  grid->addWidget(createHeadingPlotGroup(), 1, 1);
  setLayout(grid);

  setWindowTitle(tr("Group Boxes"));
  resize(480, 320);
}

QGroupBox *Window::createParametersGroup()
{
  // @TODO create a signal to have ANY changes in this group restart the system
  //  some can be asynchronous though...
  ackermann::Params params(0.45, 45.0, 1.0, 1.0);

  QGroupBox *groupBox = new QGroupBox(tr("Contoller Parameters"));

  // variables that require a system reset / restart
  QDoubleSpinBox *controlFrequency = new QDoubleSpinBox();
  controlFrequency->setValue(params.control_frequency);
  QDoubleSpinBox *wheelBase = new QDoubleSpinBox();
  wheelBase->setValue(params.wheel_base);
  QDoubleSpinBox *maxSteeringAngle = new QDoubleSpinBox();
  maxSteeringAngle->setValue(params.max_steering_angle);

  // variables that can / should be set on the fly
  QDoubleSpinBox *kpHeading = new QDoubleSpinBox();
  kpHeading->setValue(params.kp_heading);
  QDoubleSpinBox *kiHeading = new QDoubleSpinBox();
  kiHeading->setValue(params.ki_heading);
  QDoubleSpinBox *kdHeading = new QDoubleSpinBox();
  kdHeading->setValue(params.kd_heading);
  QDoubleSpinBox *kpSpeed = new QDoubleSpinBox();
  kpSpeed->setValue(params.kp_speed);
  QDoubleSpinBox *kiSpeed = new QDoubleSpinBox();
  kiSpeed->setValue(params.ki_speed);
  QDoubleSpinBox *kdSpeed = new QDoubleSpinBox();
  kdSpeed->setValue(params.kd_speed);

  QVBoxLayout *vbox = new QVBoxLayout;
  vbox->addWidget(controlFrequency);
  vbox->addWidget(wheelBase);
  vbox->addWidget(maxSteeringAngle);
  vbox->addWidget(kpHeading);
  vbox->addWidget(kiHeading);
  vbox->addWidget(kdHeading);
  vbox->addWidget(kpSpeed);
  vbox->addWidget(kiSpeed);
  vbox->addWidget(kdSpeed);

  vbox->addStretch(1);
  groupBox->setLayout(vbox);

  return groupBox;
}

QGroupBox *Window::createControllerOperationsGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("&Controller Operations"));

  QPushButton *startButton = new QPushButton(tr("&start"));
  QPushButton *stopButton = new QPushButton(tr("&stop"));
  QPushButton *resetButton = new QPushButton(tr("&reset"));

  QVBoxLayout *vbox = new QVBoxLayout;
  vbox->addWidget(startButton);
  vbox->addWidget(stopButton);
  vbox->addWidget(resetButton);
  vbox->addStretch(1);
  groupBox->setLayout(vbox);

  return groupBox;
}

QGroupBox *Window::createSpeedPlotGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("Speed Plots"));

  QLineSeries* series = new QLineSeries();
  series->append(0, 6);
  series->append(2, 4);

  // add chart instance
  QChart *chart = new QChart();
  chart->legend()->hide();
  chart->addSeries(series);
  chart->createDefaultAxes();
  chart->setTitle("Simple line chart example");

  QChartView *chartView = new QChartView(chart);
  chartView->setRenderHint(QPainter::Antialiasing);

  QVBoxLayout *vbox = new QVBoxLayout;
  vbox->addWidget(chartView);
  vbox->addStretch(1);
  groupBox->setLayout(vbox);

  return groupBox;
}

QGroupBox *Window::createHeadingPlotGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("Heading Plots"));

  QVBoxLayout *vbox = new QVBoxLayout;
  vbox->addStretch(1);
  groupBox->setLayout(vbox);

  return groupBox;
}
