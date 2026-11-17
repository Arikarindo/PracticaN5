#include "Particula.h"
#include <cmath>

using namespace std;

Particula::Particula(int id, const Vector2D &posicion,
                     const Vector2D &velocidad, double masa, double radio,
                     int idPadre1, int idPadre2)
    : id(id), posicion(posicion), velocidad(velocidad), masa(masa),
      radio(radio), activa(true), idPadre1(idPadre1), idPadre2(idPadre2) {}

int Particula::getId() const { return id; }
const Vector2D &Particula::getPosicion() const { return posicion; }
const Vector2D &Particula::getVelocidad() const { return velocidad; }
double Particula::getMasa() const { return masa; }
double Particula::getRadio() const { return radio; }
bool Particula::estaActiva() const { return activa; }
int Particula::getIdPadre1() const { return idPadre1; }
int Particula::getIdPadre2() const { return idPadre2; }

void Particula::setPosicion(const Vector2D &nuevaPos) { posicion = nuevaPos; }
void Particula::setVelocidad(const Vector2D &nuevaVel) { velocidad = nuevaVel; }
void Particula::desactivar() { activa = false; }

void Particula::mover(double dt) { posicion = posicion + velocidad * dt; }

double Particula::energiaCinetica() const {
  return 0.5 * masa * velocidad.magnitudCuadrada();
}

Vector2D Particula::momentoLineal() const { return velocidad * masa; }

bool Particula::colisionaCon(const Particula &otra) const {
  if (!activa || !otra.activa) {
    return false;
  }
  Vector2D diferencia = posicion - otra.posicion;
  double sumaRadios = radio + otra.radio;
  return diferencia.magnitudCuadrada() <= (sumaRadios * sumaRadios);
}

Particula Particula::fusionar(int nuevoId, const Particula &p1,
                              const Particula &p2) {
  double masaTotal = p1.masa + p2.masa;

  // Conservación del momento lineal: v' = (m1*v1 + m2*v2) / (m1 + m2)
  Vector2D nuevaVelocidad =
      (p1.velocidad * p1.masa + p2.velocidad * p2.masa) / masaTotal;

  // Posición del centro de masa
  Vector2D centroMasa =
      (p1.posicion * p1.masa + p2.posicion * p2.masa) / masaTotal;

  // Nuevo radio conservando el área total: pi * R^2 = pi * r1^2 + pi * r2^2 //
  // se puede cancelar pi
  double nuevoRadio = sqrt(p1.radio * p1.radio + p2.radio * p2.radio);

  return Particula(nuevoId, centroMasa, nuevaVelocidad, masaTotal, nuevoRadio,
                   p1.id, p2.id);
}
