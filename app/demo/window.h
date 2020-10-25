
#ifndef WINDOW_H
#define WINDOW_H

#include <QWidget>

QT_BEGIN_NAMESPACE
class QGroupBox;
QT_END_NAMESPACE

class Window : public QWidget
{
  Q_OBJECT

 public:
  Window(QWidget *parent = nullptr);

 private:
  QGroupBox *createParametersGroup();
  QGroupBox *createControllerOperationsGroup();
  QGroupBox *createSpeedPlotGroup();
  QGroupBox *createHeadingPlotGroup();
};

#endif