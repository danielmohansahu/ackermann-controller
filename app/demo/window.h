
#ifndef WINDOW_H
#define WINDOW_H

#include <memory>

#include <QWidget>

#include <Params.hpp>

QT_BEGIN_NAMESPACE
class QGroupBox;
QT_END_NAMESPACE

class Window : public QWidget
{
  Q_OBJECT

 public:
  Window(QWidget *parent = nullptr);
  Window(const std::shared_ptr<ackermann::Params> params);

 private:
  void init();

  QGroupBox *createParametersGroup();
  QGroupBox *createControllerOperationsGroup();
  QGroupBox *createSpeedPlotGroup();
  QGroupBox *createHeadingPlotGroup();

  // our shared parameters instance
  std::shared_ptr<ackermann::Params> params_;
};

#endif