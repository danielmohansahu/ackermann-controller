/* @file demo.cpp
 * @copyright [2020]
 */

#include <QApplication>

#include <Controller.hpp>
#include <Params.hpp>
#include "demo/window.h"

int main(int argc, char *argv[])
{
  // instantiate shared controller parameters instance
  auto params = std::make_shared<ackermann::Params>(0.45, 0.785, 1.0, 1.0);

  // construct Controller instance

  // @TODO Daniel M. Sahu construct dummy plant class and instantiate here
  // plant DummyPlant;

  // construct controller class
  ackermann::Controller controller(params);

  // begin QT application instance
  QApplication app(argc, argv);

  // instantiate QT demo window widget
  Window window(params);

  // show and handle user input
  window.show();
  return app.exec();
}