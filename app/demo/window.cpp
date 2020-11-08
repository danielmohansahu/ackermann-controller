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
  grid->addWidget(createCommandPlotGroup(), 2, 1);
  setLayout(grid);

  setWindowTitle(tr("Group Boxes"));
  resize(1560, 1280);
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
    controller_->setGoal(speed_setpoint_, heading_setpoint_);
    plant_->setState(initial_speed_, initial_heading_);

    // also reset our line series and clear our chart view
    speedSetpointSeries->clear();
    speedAchievedSeries->clear();
    speedGoalSeries->clear();
    speedLeftFrontWheelSeries->clear();
    speedRightFrontWheelSeries->clear();
    speedLeftRearWheelSeries->clear();
    speedRightRearWheelSeries->clear();
    headingSetpointSeries->clear();
    headingAchievedSeries->clear();
    headingGoalSeries->clear();
    commandThrottleSeries->clear();
    commandSteeringSeries->clear();

    speedChart->createDefaultAxes();
    headingChart->createDefaultAxes();
    commandChart->createDefaultAxes();
  }  
}

void Window::execute()
{
  // initialize time and some handy variables
  double time = 0.0;

  // initialize range values
  double speed_min = 0.0;
  double speed_max = 1.0;
  double heading_min = -M_PI/4.0;
  double heading_max = M_PI/4.0;
  double command_min = 0.0;
  double command_max = 1.0;

  // start controller
  controller_->start();

  // continually evaluate the controller's commands and send them to the plant
  while (!stop_)
  {
    // update controller goal (in case it changed locally)
    controller_->setGoal(speed_setpoint_, heading_setpoint_);

    // poll Plant for current state
    double current_speed, current_heading;
    plant_->getState(current_speed, current_heading);

    // poll controller for current goal
    double target_speed, target_heading;
    controller_->getGoal(target_speed, target_heading);

    // get the latest controller command
    double throttle, steering;
    controller_->getCommand(throttle, steering);

    // get the latest wheel speeds
    double left_front, right_front, left_rear, right_rear;
    controller_->getWheelLinVel(left_front, right_front, left_rear, right_rear);

    // apply the command to the plant
    plant_->command(throttle, steering, TIMESTEP);

    // update the QT series with our latest information
    speedSetpointSeries->append(time, speed_setpoint_);
    speedGoalSeries->append(time, target_speed);
    speedAchievedSeries->append(time, current_speed);
    speedLeftFrontWheelSeries->append(time, left_front);
    speedRightFrontWheelSeries->append(time, right_front);
    speedLeftRearWheelSeries->append(time, left_rear);
    speedRightRearWheelSeries->append(time, right_rear);
    headingSetpointSeries->append(time, heading_setpoint_);
    headingGoalSeries->append(time, target_heading);
    headingAchievedSeries->append(time, current_heading);
    commandThrottleSeries->append(time, throttle);
    commandSteeringSeries->append(time, steering);

    // update the QT charts
    double x_min = std::max(0.0, time - TIMEWINDOW);
    speedChart->axisX()->setRange(x_min, time);
    headingChart->axisX()->setRange(x_min, time);
    commandChart->axisX()->setRange(x_min, time);

    // update the Y ranges
    speed_min = std::min({speed_min, static_cast<double>(speed_setpoint_), target_speed, current_speed, left_front, right_front, left_rear, right_rear});
    speed_max = std::max({speed_max, static_cast<double>(speed_setpoint_), target_speed, current_speed, left_front, right_front, left_rear, right_rear});
    heading_min = std::min({heading_min, static_cast<double>(heading_setpoint_), target_heading, current_heading});
    heading_max = std::max({heading_max, static_cast<double>(heading_setpoint_), target_heading, current_heading});
    command_min = std::min({command_min, throttle, steering});
    command_max = std::max({command_max, throttle, steering});

    speedChart->axisY()->setRange(speed_min, speed_max);
    headingChart->axisY()->setRange(heading_min, heading_max);
    commandChart->axisY()->setRange(command_min, command_max);

    // sleep and update our timestamp
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 * TIMESTEP)));
    time += TIMESTEP;
  }

  // cleanup; stop controller
  controller_->stop();
}

QGroupBox *Window::createParametersGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("Controller Parameters"));

  // controller frequency
  QLabel *controlFrequencyLabel = new QLabel(tr("Desired controller loop rate:"));
  QDoubleSpinBox *controlFrequency = new QDoubleSpinBox();
  controlFrequency->setMinimum(params_->control_frequency / 10);
  controlFrequency->setMaximum(params_->control_frequency * 10);
  controlFrequency->setValue(params_->control_frequency);
  controlFrequency->setSuffix(tr(" (hz)"));
  controlFrequency->setSingleStep(10.0);

  connect(controlFrequency, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double new_val){params_->control_frequency = new_val;});

  // wheel base
  QLabel *wheelBaseLabel = new QLabel(tr("Vehicle wheel base:"));
  QDoubleSpinBox *wheelBase = new QDoubleSpinBox();
  wheelBase->setValue(params_->wheel_base);
  wheelBase->setSuffix(tr(" (m)"));
  wheelBase->setSingleStep(0.1);

  connect(wheelBase, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double new_val){params_->wheel_base = new_val;});

  // track width
  QLabel *trackWidthLabel = new QLabel(tr("Vehicle track width:"));
  QDoubleSpinBox *trackWidth = new QDoubleSpinBox();
  trackWidth->setValue(params_->track_width);
  trackWidth->setSuffix(tr(" (m)"));
  trackWidth->setSingleStep(0.1);

  connect(trackWidth, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double new_val){params_->track_width = new_val;});

  // max steering angle
  QLabel *maxSteeringAngleLabel = new QLabel(tr("Vehicle maximum steering angle:"));
  QDoubleSpinBox *maxSteeringAngle = new QDoubleSpinBox();
  maxSteeringAngle->setMinimum(-M_PI);
  maxSteeringAngle->setMaximum(M_PI);
  maxSteeringAngle->setValue(params_->max_steering_angle);
  maxSteeringAngle->setSuffix(tr(" (rad)"));
  maxSteeringAngle->setSingleStep(0.1);

  connect(maxSteeringAngle, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double new_val){params_->max_steering_angle = new_val;});

  // heading PID params (KP, KI, KD)
  QLabel *headingPIDLabel = new QLabel(tr("Heading PID Parameters:"));
  QDoubleSpinBox *kpHeading = new QDoubleSpinBox();
  kpHeading->setValue(params_->pid_heading->kp);
  kpHeading->setPrefix(tr("kp: "));
  kpHeading->setSingleStep(0.1);
  QDoubleSpinBox *kiHeading = new QDoubleSpinBox();
  kiHeading->setValue(params_->pid_heading->ki);
  kiHeading->setPrefix(tr("ki: "));
  kiHeading->setSingleStep(0.1);
  QDoubleSpinBox *kdHeading = new QDoubleSpinBox();
  kdHeading->setValue(params_->pid_heading->kd);
  kdHeading->setPrefix(tr("kd: "));
  kdHeading->setSingleStep(0.1);

  connect(kpHeading, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double new_val){params_->pid_heading->kp = new_val;});
  connect(kiHeading, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double new_val){params_->pid_heading->ki = new_val;});
  connect(kdHeading, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double new_val){params_->pid_heading->kd = new_val;});

  // speed PID params (KP, KI, KD)
  QLabel *speedPIDLabel = new QLabel(tr("Speed PID Parameters:"));
  QDoubleSpinBox *kpSpeed = new QDoubleSpinBox();
  kpSpeed->setValue(params_->pid_speed->kp);
  kpSpeed->setPrefix(tr("kp: "));
  kpSpeed->setSingleStep(0.1);
  QDoubleSpinBox *kiSpeed = new QDoubleSpinBox();
  kiSpeed->setValue(params_->pid_speed->ki);
  kiSpeed->setPrefix(tr("ki: "));
  kiSpeed->setSingleStep(0.1);
  QDoubleSpinBox *kdSpeed = new QDoubleSpinBox();
  kdSpeed->setValue(params_->pid_speed->kd);
  kdSpeed->setPrefix(tr("kd: "));
  kdSpeed->setSingleStep(0.1);

  connect(kpSpeed, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double new_val){params_->pid_speed->kp = new_val;});
  connect(kiSpeed, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double new_val){params_->pid_speed->ki = new_val;});
  connect(kdSpeed, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double new_val){params_->pid_speed->kd = new_val;});

  // add all parameters to box
  QVBoxLayout *vbox = new QVBoxLayout;
  vbox->addWidget(controlFrequencyLabel);
  vbox->addWidget(controlFrequency);
  vbox->addWidget(wheelBaseLabel);
  vbox->addWidget(wheelBase);
  vbox->addWidget(trackWidthLabel);
  vbox->addWidget(trackWidth);
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
  speedSetpoint = new QDoubleSpinBox();
  speedSetpoint->setValue(speed_setpoint_);
  speedSetpoint->setSuffix(tr(" (m/s)"));

  // heading setpoint
  QLabel *headingSetpointLabel = new QLabel(tr("Desired Plant heading:"));
  headingSetpoint = new QDoubleSpinBox();
  headingSetpoint->setMinimum(-M_PI);
  headingSetpoint->setMaximum(M_PI);
  headingSetpoint->setValue(heading_setpoint_);
  headingSetpoint->setSuffix(tr(" (rad)"));
  headingSetpoint->setSingleStep(0.1);

  // plant initial speed
  QLabel *initialSpeedLabel = new QLabel(tr("Initial Plant speed:"));
  initialSpeed = new QDoubleSpinBox();
  initialSpeed->setValue(initial_speed_);
  initialSpeed->setSuffix(tr(" (m/s)"));

  // speed setpoint
  QLabel *initialHeadingLabel = new QLabel(tr("Initial Plant heading:"));
  initialHeading = new QDoubleSpinBox();
  initialHeading->setMinimum(-M_PI);
  initialHeading->setMaximum(M_PI);
  initialHeading->setValue(heading_setpoint_);
  initialHeading->setSuffix(tr(" (rad)"));
  initialHeading->setSingleStep(0.1);

  // connect signals to slots
  connect(speedSetpoint, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double new_val){speed_setpoint_ = new_val;});
  connect(headingSetpoint, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double new_val){heading_setpoint_ = new_val;});
  connect(initialSpeed, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double new_val){initial_speed_ = new_val;});
  connect(initialHeading, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double new_val){initial_heading_ = new_val;});

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

  // add controller internal setpoint series
  speedGoalSeries = new QLineSeries();

  // add achieved values series
  speedAchievedSeries = new QLineSeries();

  // individual wheel speed plots 
  speedLeftFrontWheelSeries = new QLineSeries();
  speedRightFrontWheelSeries = new QLineSeries();
  speedLeftRearWheelSeries = new QLineSeries();
  speedRightRearWheelSeries = new QLineSeries();

  // add chart instance
  speedChart = new QChart();
  speedChart->addSeries(speedSetpointSeries);
  speedChart->addSeries(speedGoalSeries);
  speedChart->addSeries(speedAchievedSeries);
  speedChart->addSeries(speedLeftFrontWheelSeries);
  speedChart->addSeries(speedRightFrontWheelSeries);
  speedChart->addSeries(speedLeftRearWheelSeries);
  speedChart->addSeries(speedRightRearWheelSeries);
  speedChart->createDefaultAxes();
  speedChart->setTitle("Vehicle Speed (m/s)");
  speedChart->legend()->setAlignment(Qt::AlignRight);
  speedChart->legend()->markers(speedSetpointSeries)[0]->setLabel(tr("Desired Setpoint"));
  speedChart->legend()->markers(speedGoalSeries)[0]->setLabel(tr("Controller Goal"));
  speedChart->legend()->markers(speedAchievedSeries)[0]->setLabel(tr("Simulated Response"));
  speedChart->legend()->markers(speedLeftFrontWheelSeries)[0]->setLabel(tr("Wheel Command (front left)"));
  speedChart->legend()->markers(speedRightFrontWheelSeries)[0]->setLabel(tr("Wheel Command (front right)"));
  speedChart->legend()->markers(speedLeftRearWheelSeries)[0]->setLabel(tr("Wheel Command (rear left)"));
  speedChart->legend()->markers(speedRightRearWheelSeries)[0]->setLabel(tr("Wheel Command (rear right)"));

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

  // add heading series
  headingSetpointSeries = new QLineSeries();
  headingGoalSeries = new QLineSeries();
  headingAchievedSeries = new QLineSeries();

  // add chart instance
  headingChart = new QChart();
  headingChart->addSeries(headingSetpointSeries);
  headingChart->addSeries(headingGoalSeries);
  headingChart->addSeries(headingAchievedSeries);
  headingChart->createDefaultAxes();
  headingChart->setTitle("Vehicle Heading (rad)");
  headingChart->legend()->setAlignment(Qt::AlignRight);
  headingChart->legend()->markers(headingSetpointSeries)[0]->setLabel(tr("Desired Setpoint"));
  headingChart->legend()->markers(headingGoalSeries)[0]->setLabel(tr("Controller Goal"));
  headingChart->legend()->markers(headingAchievedSeries)[0]->setLabel(tr("Simulated Response"));
  headingChart->axisY()->setRange(-M_PI, M_PI);

  // add ChartView instance (to actually display the chart)
  QChartView *chartView = new QChartView(headingChart);
  chartView->setRenderHint(QPainter::Antialiasing);

  // add chart to visual box
  QVBoxLayout *vbox = new QVBoxLayout;
  vbox->addWidget(chartView);
  groupBox->setLayout(vbox);

  return groupBox;
}

QGroupBox *Window::createCommandPlotGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("Command Plots"));

  // add dummy series (for now)
  commandThrottleSeries = new QLineSeries();
  commandSteeringSeries = new QLineSeries();

  // add chart instance
  commandChart = new QChart();
  commandChart->addSeries(commandThrottleSeries);
  commandChart->addSeries(commandSteeringSeries);
  commandChart->createDefaultAxes();
  commandChart->setTitle("Controller Commands");
  commandChart->legend()->setAlignment(Qt::AlignRight);
  commandChart->legend()->markers(commandThrottleSeries)[0]->setLabel(tr("Commanded Throttle"));
  commandChart->legend()->markers(commandSteeringSeries)[0]->setLabel(tr("Commanded Steer"));
  commandChart->axisY()->setRange(-M_PI, M_PI);

  // add ChartView instance (to actually display the chart)
  QChartView *chartView = new QChartView(commandChart);
  chartView->setRenderHint(QPainter::Antialiasing);

  // add chart to visual box
  QVBoxLayout *vbox = new QVBoxLayout;
  vbox->addWidget(chartView);
  groupBox->setLayout(vbox);

  return groupBox;
}