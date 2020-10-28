#include "window.h"

#include <math.h>

#include <QCheckBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QMenu>
#include <QPushButton>
#include <QRadioButton>
#include <QDoubleSpinBox>

Window::Window(const std::shared_ptr<ackermann::Params>& params,
               const std::shared_ptr<ackermann::Controller>& controller,
               const std::shared_ptr<fake::Plant>& plant)
  : QWidget(nullptr),
    params_(params),
    controller_(controller),
    plant_(plant)
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
  grid->addWidget(createSetpointsGroup(), 1, 0);
  grid->addWidget(createControllerOperationsGroup(), 2, 0);
  grid->addWidget(createSpeedPlotGroup(), 0, 1);
  grid->addWidget(createHeadingPlotGroup(), 1, 1);
  setLayout(grid);

  setWindowTitle(tr("Group Boxes"));
  resize(480, 320);
}

void Window::start()
{
  // only process this request if we're not already running
  if (!thread_handle_.joinable())
  {
    // a start should also clear out the plot and any associated states
    this->reset();

    // spin off thread to continually execute
    stop_ = false;
    thread_handle_ = std::thread([this](){this->execute();});
  }
}

void Window::stop()
{
  // stop thread and rejoin
  stop_ = true;
  if (thread_handle_.joinable())
  {
    thread_handle_.join();
    std::cout << "Execution stopped." << std::endl;
  }
}

void Window::reset()
{
  // do nothing if we're currently running (?)
  if (thread_handle_.joinable())
  {
    std::cout << "Not resetting during a live run." << std::endl;
  }
  else
  {
    // reset our objects to base states
    std::cout << "Resetting Controller and Plant." << std::endl;
    controller_->reset();
    plant_->reset();

    // ensure that we keep the current setpoint and plant state
    controller_->setGoal(heading_setpoint_, speed_setpoint_);
    plant_->setState(initial_speed_, initial_heading_);

    // also reset our line series and clear our chart view
    speedSetpointSeries->clear();
    speedAchievedSeries->clear();
    speedChart->createDefaultAxes();
  }  
}

// @TODO add parameters / signals for:
// goal (speed, heading)
// initial conditions (plant)

void Window::execute()
{
  // initialize time and some handy variables
  double time = 0.0;
  double dt = 0.1;

  // start controller
  controller_->start();

  // continually evaluate the controller's commands and send them to the plant
  while (!stop_)
  {
    // poll Plant for current state
    double current_speed, current_heading;
    plant_->getState(current_speed, current_heading);

    // get the latest controller command
    double throttle, steering;
    controller_->getCommand(throttle, steering);

    // apply the command to the plant
    plant_->command(throttle, steering, dt);

    // update the chart with our latest information
    speedSetpointSeries->append(time, speed_setpoint_);
    speedAchievedSeries->append(time, current_speed);
    speedChart->axisX()->setRange(0, time);
    // @TODO calculate this and adjust
    speedChart->axisY()->setRange(0, 10);

    // @TODO add in command chart
    // @TODO add in heading series

    // sleep and update our timestamp
    // @TODO parameterize this
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    time += dt;
  }

  // cleanup; stop controller
  controller_->stop();
}

QGroupBox *Window::createParametersGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("Contoller Parameters"));

  // controller frequency
  QLabel *controlFrequencyLabel = new QLabel(tr("Desired controller loop rate:"));
  QDoubleSpinBox *controlFrequency = new QDoubleSpinBox();
  controlFrequency->setValue(params_->control_frequency);
  controlFrequency->setSuffix(tr(" (hz)"));

  // wheel base
  QLabel *wheelBaseLabel = new QLabel(tr("Vehicle wheel base:"));
  QDoubleSpinBox *wheelBase = new QDoubleSpinBox();
  wheelBase->setValue(params_->wheel_base);
  wheelBase->setSuffix(tr(" (m)"));

  // max steering angle
  QLabel *maxSteeringAngleLabel = new QLabel(tr("Vehicle maximum steering angle:"));
  QDoubleSpinBox *maxSteeringAngle = new QDoubleSpinBox();
  maxSteeringAngle->setValue(params_->max_steering_angle);
  maxSteeringAngle->setSuffix(tr(" (rad)"));

  // heading PID params (KP, KI, KD)
  QLabel *headingPIDLabel = new QLabel(tr("Heading PID Parameters:"));
  QDoubleSpinBox *kpHeading = new QDoubleSpinBox();
  kpHeading->setValue(params_->kp_heading);
  kpHeading->setPrefix(tr("kp: "));
  QDoubleSpinBox *kiHeading = new QDoubleSpinBox();
  kiHeading->setValue(params_->ki_heading);
  kiHeading->setPrefix(tr("ki: "));
  QDoubleSpinBox *kdHeading = new QDoubleSpinBox();
  kdHeading->setValue(params_->kd_heading);
  kdHeading->setPrefix(tr("kd: "));

  // speed PID params (KP, KI, KD)
  QLabel *speedPIDLabel = new QLabel(tr("Speed PID Parameters:"));
  QDoubleSpinBox *kpSpeed = new QDoubleSpinBox();
  kpSpeed->setValue(params_->kp_speed);
  kpSpeed->setPrefix(tr("kp: "));
  QDoubleSpinBox *kiSpeed = new QDoubleSpinBox();
  kiSpeed->setValue(params_->ki_speed);
  kiSpeed->setPrefix(tr("ki: "));
  QDoubleSpinBox *kdSpeed = new QDoubleSpinBox();
  kdSpeed->setValue(params_->kd_speed);
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

QGroupBox *Window::createSetpointsGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("Contoller Goals and Initial Conditions"));

  // speed setpoint
  QLabel *speedSetpointLabel = new QLabel(tr("Desired Plant speed:"));
  QDoubleSpinBox *speedSetpoint = new QDoubleSpinBox();
  speedSetpoint->setValue(speed_setpoint_);
  speedSetpoint->setSuffix(tr(" (m/s)"));

  // heading setpoint
  QLabel *headingSetpointLabel = new QLabel(tr("Desired Plant heading:"));
  QDoubleSpinBox *headingSetpoint = new QDoubleSpinBox();
  headingSetpoint->setValue(heading_setpoint_);
  headingSetpoint->setSuffix(tr(" (rad)"));

  // plant initial speed
  QLabel *initialSpeedLabel = new QLabel(tr("Initial Plant speed:"));
  QDoubleSpinBox *initialSpeed = new QDoubleSpinBox();
  initialSpeed->setValue(initial_speed_);
  initialSpeed->setSuffix(tr(" (m/s)"));

  // speed setpoint
  QLabel *initialHeadingLabel = new QLabel(tr("Desired Plant heading:"));
  QDoubleSpinBox *initialHeading = new QDoubleSpinBox();
  initialHeading->setValue(heading_setpoint_);
  initialHeading->setSuffix(tr(" (rad)"));

  // connect signals to slots


  // add all parameters to box
  QVBoxLayout *vbox = new QVBoxLayout;
  vbox->addWidget(speedSetpointLabel);
  vbox->addWidget(speedSetpoint);
  vbox->addWidget(headingSetpointLabel);
  vbox->addWidget(headingSetpoint);
  vbox->addWidget(initialSpeedLabel);
  vbox->addWidget(initialSpeed);
  vbox->addWidget(initialHeadingLabel);
  vbox->addWidget(initialHeading);

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

  // connect signals and slots to these buttons
  connect(startButton, SIGNAL(clicked()), this, SLOT(start()));
  connect(stopButton, SIGNAL(clicked()), this, SLOT(stop()));
  connect(resetButton, SIGNAL(clicked()), this, SLOT(reset()));

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
  speedSetpointSeries = new QLineSeries();

  // add achieved values series
  speedAchievedSeries = new QLineSeries();

  // add chart instance
  speedChart = new QChart();
  // chart->legend()->hide();
  speedChart->addSeries(speedSetpointSeries);
  speedChart->addSeries(speedAchievedSeries);
  speedChart->createDefaultAxes();
  speedChart->setTitle("Vehicle Speed");

  // add ChartView instance (to actually display the chart)
  QChartView *chartView = new QChartView(speedChart);
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

  // add dummy series (for now)
  QLineSeries* dummy1 = new QLineSeries();
  QLineSeries* dummy2 = new QLineSeries();

  // add chart instance
  QChart *chart = new QChart();
  // chart->legend()->hide();
  chart->addSeries(dummy1);
  chart->addSeries(dummy2);
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
