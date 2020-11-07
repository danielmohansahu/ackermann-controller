/* @file demo.cpp
 * @copyright [2020]
 */

#include <QApplication>

#include <Controller.hpp>
#include <Params.hpp>
#include <fake/plant.h>

#include "demo/window.h"

int main(int argc, char *argv[])
{
  // instantiate shared controller parameters instance
  auto params = std::make_shared<ackermann::Params>(0.45, 0.785, 0.02, 0.2);

  // construct controller class
  auto controller = std::make_shared<ackermann::Controller>(params);

  // Construct dummy plant class
  fake::PlantOptions opts(0.45, 0.785);
  auto plant = std::make_shared<fake::Plant>(opts, params);

  // begin QT application instance
  QApplication app(argc, argv);

  // instantiate QT demo window widget
  Window window(params, controller, plant);

  // show and handle user input
  window.show();
  return app.exec();
}
