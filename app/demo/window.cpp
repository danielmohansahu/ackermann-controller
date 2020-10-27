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
  grid->addWidget(createControllerOperationsGroup(), 1, 0);
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
  }  
}

void Window::execute()
{
  // start the controller and continually evaluate its commands and send them to the plant
  double x = 0.0;
  double y1 = 0.0;
  double y2 = 0.0;
  while (!stop_)
  {
    std::cout << "Continually executing..." << std::endl;
    x += 0.1;
    y1 = std::sin(x);
    y2 = std::sin(x);
    setpointSeries->append(x, y1);
    setpointSeries->append(x, y2);

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
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
  QLineSeries* setpointSeries = new QLineSeries();

  // add achieved values series
  QLineSeries* achievedSeries = new QLineSeries();

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
