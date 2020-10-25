/* @file demo.cpp
 * @copyright [2020]
 */

#include <QApplication>

#include "demo/window.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    Window window;
    window.show();
    return app.exec();
}