#pragma once

/**
 * @file window.h
 * @brief Declaration for GUI implementation
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @author Sentosh Kesani
 *
 * @copyright [2020]
 */

#include <memory>
#include <atomic>
#include <thread>
#include <chrono>

#include <QWidget>
#include <QtCharts>

#include <Params.hpp>
#include <Controller.hpp>
#include <fake/plant.h>

// some handy visualization variables
#define TIMESTEP 0.1
#define TIMEWINDOW 10.0

QT_BEGIN_NAMESPACE
class QGroupBox;
QT_END_NAMESPACE

/**
* @brief GUI implementation for user interface
 */
class Window : public QWidget
{
  Q_OBJECT

 public:
  explicit Window(QWidget *parent = nullptr);

  /**
  * @brief Create GUI interface with pre-programmed conditions
  * @param params Shared pointer to rover characteristic parameters
  * @param controller Shared pointer to controller object
  * @param plant Shared pointer to plant object (to apply received commands)
  */
  Window(const std::shared_ptr<ackermann::Params>& params,
         const std::shared_ptr<ackermann::Controller>& controller,
         const std::shared_ptr<fake::Plant>& plant);

 public slots:
  // QT Slots; called via signals emitted from button clicks
  /**
  * @brief Begin simulation run with current parameters.
  */
  void start();
  /**
  * @brief Stop current simulation run.
  */
  void stop();
  /**
  * @brief Reset simulation to initial conditions.
  */
  void reset();

 private:
  void init();

  void execute();

  // create group boxes
  QGroupBox *createParametersGroup();
  QGroupBox *createSetpointsGroup();
  QGroupBox *createControllerOperationsGroup();
  QGroupBox *createSpeedPlotGroup();
  QGroupBox *createHeadingPlotGroup();
  QGroupBox *createCommandPlotGroup();

  // QT boxes (need to be class variables to connect to signals)
  QDoubleSpinBox* speedSetpoint;
  QDoubleSpinBox* headingSetpoint;
  QDoubleSpinBox* initialSpeed;
  QDoubleSpinBox* initialHeading;

  // QT data series and chart objects (used in visualization)
  QLineSeries* speedSetpointSeries;
  QLineSeries* speedGoalSeries;
  QLineSeries* speedAchievedSeries;
  QChart* speedChart;

  QLineSeries* headingSetpointSeries;
  QLineSeries* headingGoalSeries;
  QLineSeries* headingAchievedSeries;
  QChart* headingChart;

  QLineSeries* commandThrottleSeries;
  QLineSeries* commandSteeringSeries;
  QChart* commandChart;

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
