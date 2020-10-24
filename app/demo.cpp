/* @file demo.cpp
 * @copyright [2020]
 */

#include <iostream>
#include <Controller.hpp>
#include <Params.hpp>

int main() {
  ackermann::Params p(0.1, 45.0, 0.25, 1.0);
  ackermann::Controller Rover(p);
}
