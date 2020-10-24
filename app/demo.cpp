/* @file demo.cpp
 * @copyright [2020]
 */

#include <iostream>
#include <Demo.hpp>

#include <QPushButton>

int main(int argc, char ** argv) {

  // start core qt application
  QApplication app(argc, argv);


  QPushButton button("Hello World !");
  button.show();

  return app.exec();
}
