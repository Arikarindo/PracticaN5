#ifndef VECTOR2D_H
#define VECTOR2D_H
#include <cmath>
using namespace std;

// Representa un vector en el plano 2D para posiciones, velocidades y fuerzas.
struct Vector2D {
  double x;
  double y;

  Vector2D(double x = 0.0, double y = 0.0) : x(x), y(y) {}

  Vector2D operator+(const Vector2D &otro) const {
    return Vector2D(x + otro.x, y + otro.y);
  }

  Vector2D operator-(const Vector2D &otro) const {
    return Vector2D(x - otro.x, y - otro.y);
  }

  Vector2D operator*(double escalar) const {
    return Vector2D(x * escalar, y * escalar);
  }

  Vector2D operator/(double escalar) const {
    return Vector2D(x / escalar, y / escalar);
  }

  double productoPunto(const Vector2D &otro) const {
    return x * otro.x + y * otro.y;
  }

  double magnitudCuadrada() const { return x * x + y * y; }

  double magnitud() const { return sqrt(magnitudCuadrada()); }

  Vector2D normalizado() const {
    double mag = magnitud();
    if (mag == 0.0) {
      return Vector2D(0.0, 0.0);
    }
    return Vector2D(x / mag, y / mag);
  }
};

#endif // VECTOR2D_H
