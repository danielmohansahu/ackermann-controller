
#include "window.h"

#include <QCheckBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QMenu>
#include <QPushButton>
#include <QRadioButton>
#include <QDoubleSpinBox>
#include <QtCharts>

Window::Window(const std::shared_ptr<ackermann::Params> params)
  : QWidget(nullptr),
    params_(params)
{
  this->init();
}

Window::Window(QWidget *parent)
  : QWidget(parent)
{
  this->init();
}

void Window::init()
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

  // controller frequency
  QLabel *controlFrequencyLabel = new QLabel(tr("Desired controller loop rate:"));
  QDoubleSpinBox *controlFrequency = new QDoubleSpinBox();
  controlFrequency->setValue(params.control_frequency);
  controlFrequency->setSuffix(tr(" (hz)"));

  // wheel base
  QLabel *wheelBaseLabel = new QLabel(tr("Vehicle wheel base:"));
  QDoubleSpinBox *wheelBase = new QDoubleSpinBox();
  wheelBase->setValue(params.wheel_base);
  wheelBase->setSuffix(tr(" (m)"));

  // max steering angle
  QLabel *maxSteeringAngleLabel = new QLabel(tr("Vehicle maximum steering angle:"));
  QDoubleSpinBox *maxSteeringAngle = new QDoubleSpinBox();
  maxSteeringAngle->setValue(params.max_steering_angle);
  maxSteeringAngle->setSuffix(tr(" (rad)"));

  // heading PID params (KP, KI, KD)
  QLabel *headingPIDLabel = new QLabel(tr("Heading PID Parameters:"));
  QDoubleSpinBox *kpHeading = new QDoubleSpinBox();
  kpHeading->setValue(params.kp_heading);
  kpHeading->setPrefix(tr("kp: "));
  QDoubleSpinBox *kiHeading = new QDoubleSpinBox();
  kiHeading->setValue(params.ki_heading);
  kiHeading->setPrefix(tr("ki: "));
  QDoubleSpinBox *kdHeading = new QDoubleSpinBox();
  kdHeading->setValue(params.kd_heading);
  kdHeading->setPrefix(tr("kd: "));

  // speed PID params (KP, KI, KD)
  QLabel *speedPIDLabel = new QLabel(tr("Speed PID Parameters:"));
  QDoubleSpinBox *kpSpeed = new QDoubleSpinBox();
  kpSpeed->setValue(params.kp_speed);
  kpSpeed->setPrefix(tr("kp: "));
  QDoubleSpinBox *kiSpeed = new QDoubleSpinBox();
  kiSpeed->setValue(params.ki_speed);
  kiSpeed->setPrefix(tr("ki: "));
  QDoubleSpinBox *kdSpeed = new QDoubleSpinBox();
  kdSpeed->setValue(params.kd_speed);
  kdSpeed->setPrefix(tr("kd: "));

  // add all parameters to box
  QVBoxLayout *vbox = new QVBoxLayout;
  vbox->addWidget(controlFrequencyLabel);
  vbox->addWidget(controlFrequency);
  vbox->addWidget(wheelBaseLabel);
  vbox->addWidget(wheelBase);
  vbox->addWidget(maxSteeringAngleLabel);
  vbox->addWidget(maxSteeringAngle);
  vbox->addWidget(headingPIDLabel);
  vbox->addWidget(kpHeading);
  vbox->addWidget(kiHeading);
  vbox->addWidget(kdHeading);
  vbox->addWidget(speedPIDLabel);
  vbox->addWidget(kpSpeed);
  vbox->addWidget(kiSpeed);
  vbox->addWidget(kdSpeed);

  vbox->addStretch(1);
  groupBox->setLayout(vbox);

  return groupBox;
}

QGroupBox *Window::createControllerOperationsGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("Controller Operations"));

  QPushButton *startButton = new QPushButton(tr("start"));
  QPushButton *stopButton = new QPushButton(tr("stop"));
  QPushButton *resetButton = new QPushButton(tr("reset"));

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

  // add desired setpoint series
  QLineSeries* setpointSeries = new QLineSeries();
  setpointSeries->append(0, 6);
  setpointSeries->append(2, 4);

  // add achieved values series
  QLineSeries* achievedSeries = new QLineSeries();
  achievedSeries->append(0, 5);
  achievedSeries->append(2, 3);

  // add chart instance
  QChart *chart = new QChart();
  // chart->legend()->hide();
  chart->addSeries(setpointSeries);
  chart->addSeries(achievedSeries);
  chart->createDefaultAxes();
  chart->setTitle("Vehicle Speed");

  // add ChartView instance (to actually display the chart)
  QChartView *chartView = new QChartView(chart);
  chartView->setRenderHint(QPainter::Antialiasing);

  // add chart to visual box
  QVBoxLayout *vbox = new QVBoxLayout;
  vbox->addWidget(chartView);
  groupBox->setLayout(vbox);

  return groupBox;
}

QGroupBox *Window::createHeadingPlotGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("Heading Plots"));

  // add desired setpoint series
  QLineSeries* setpointSeries = new QLineSeries();
  setpointSeries->append(0, 16);
  setpointSeries->append(2, 4);

  // add achieved values series
  QLineSeries* achievedSeries = new QLineSeries();
  achievedSeries->append(0, 15);
  achievedSeries->append(2, 3);

  // add chart instance
  QChart *chart = new QChart();
  // chart->legend()->hide();
  chart->addSeries(setpointSeries);
  chart->addSeries(achievedSeries);
  chart->createDefaultAxes();
  chart->setTitle("Vehicle Heading");

  // add ChartView instance (to actually display the chart)
  QChartView *chartView = new QChartView(chart);
  chartView->setRenderHint(QPainter::Antialiasing);

  // add chart to visual box
  QVBoxLayout *vbox = new QVBoxLayout;
  vbox->addWidget(chartView);
  groupBox->setLayout(vbox);

  return groupBox;
}
