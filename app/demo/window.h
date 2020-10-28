#pragma once

#include <memory>
#include <atomic>
#include <thread>
#include <chrono>

#include <QWidget>
#include <QtCharts>

#include <Params.hpp>
#include <Controller.hpp>
#include <fake/plant.h>

QT_BEGIN_NAMESPACE
class QGroupBox;
QT_END_NAMESPACE

class Window : public QWidget
{
  Q_OBJECT

 public:
  explicit Window(QWidget *parent = nullptr);
  Window(const std::shared_ptr<ackermann::Params>& params,
         const std::shared_ptr<ackermann::Controller>& controller,
         const std::shared_ptr<fake::Plant>& plant);

 public slots:
  // QT Slots; called via signals emitted from button clicks
  void start();
  void stop();
  void reset();

 private:
  void init();

  void execute();

  // create group boxes
  QGroupBox *createParametersGroup();
  QGroupBox *createControllerOperationsGroup();
  QGroupBox *createSpeedPlotGroup();
  QGroupBox *createHeadingPlotGroup();

  // QT data series and chart objects (used in visualization)
  QLineSeries* speedSetpointSeries;
  QLineSeries* speedAchievedSeries;
  QChart* speedChart;

  // setpoint (e.g. goal) data
  std::atomic<double> speed_setpoint_ {0.0};
  std::atomic<double> heading_setpoint_ {0.0};
  std::atomic<double> initial_speed_ {0.0};
  std::atomic<double> initial_heading_ {0.0};

  // synchronization objects
  std::atomic<bool> stop_ {true};
  std::thread thread_handle_;
  
  // our shared parameters instance
  std::shared_ptr<ackermann::Params> params_;

  // a reference to our controller (which is executing commands)
  std::shared_ptr<ackermann::Controller> controller_;

  // a reference to our fake plant (for simulation)
  std::shared_ptr<fake::Plant> plant_;
};
