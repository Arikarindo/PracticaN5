#include "Obstaculo.h"

#include <algorithm>
#include <cmath>

using namespace std;

const char *nombreLadoImpacto(LadoImpacto lado) {
  switch (lado) {
  case LadoImpacto::Izquierdo:
    return "Lado Izquierdo";
  case LadoImpacto::Derecho:
    return "Lado Derecho";
  case LadoImpacto::Inferior:
    return "Lado Inferior";
  case LadoImpacto::Superior:
    return "Lado Superior";
  case LadoImpacto::EsquinaInfIzq:
    return "Esquina Inferior-Izquierda";
  case LadoImpacto::EsquinaInfDer:
    return "Esquina Inferior-Derecha";
  case LadoImpacto::EsquinaSupIzq:
    return "Esquina Superior-Izquierda";
  case LadoImpacto::EsquinaSupDer:
    return "Esquina Superior-Derecha";
  default:
    return "Ninguno";
  }
}

Obstaculo::Obstaculo(int id, const Vector2D &centro, double lado,
                     double coefRestitucion)
    : id(id), centro(centro), lado(lado), coefRestitucion(coefRestitucion) {}
// getters
int Obstaculo::getId() const { return id; }
const Vector2D &Obstaculo::getCentro() const { return centro; }
double Obstaculo::getLado() const { return lado; }
double Obstaculo::getCoefRestitucion() const { return coefRestitucion; }
// min y max del cuadrado
double Obstaculo::xMin() const { return centro.x - lado / 2.0; }
double Obstaculo::xMax() const { return centro.x + lado / 2.0; }
double Obstaculo::yMin() const { return centro.y - lado / 2.0; }
double Obstaculo::yMax() const { return centro.y + lado / 2.0; }

ResultadoColisionObstaculo
Obstaculo::detectarColision(const Particula &p) const {
  ResultadoColisionObstaculo res;
  if (!p.estaActiva()) {
    return res;
  }

  const Vector2D &pos = p.getPosicion();
  double r = p.getRadio();

  // Punto más cercano del cuadrado al centro de la partícula
  double cx = clamp(pos.x, xMin(), xMax());
  double cy = clamp(pos.y, yMin(), yMax());
  Vector2D puntoCercano(cx, cy);

  Vector2D delta = pos - puntoCercano;
  // Al cuadrado
  double dist2 = delta.magnitudCuadrada();

  if (dist2 > r * r) {
    return res;
  }

  res.huboColision = true;
  res.puntoContacto = puntoCercano;
  // Si es menor que un valor infinitesimalmente pequeño se considera que es
  // cero prácticamente
  if (dist2 > 1e-12) {
    // vectore normal unitario
    double dist = sqrt(dist2);
    res.normal = delta / dist;

    // Identificación del lado del cuadrado con el que ocurrió la colisión
    // valores absolutos
    if (abs(delta.x) > 1e-9 && abs(delta.y) <= 1e-9) {
      if (delta.x < 0.0) {
        res.ladoImpacto = LadoImpacto::Izquierdo;
      } else {
        res.ladoImpacto = LadoImpacto::Derecho;
      }
    } else if (abs(delta.y) > 1e-9 && abs(delta.x) <= 1e-9) {
      if (delta.y < 0.0) {
        res.ladoImpacto = LadoImpacto::Inferior;
      } else {
        res.ladoImpacto = LadoImpacto::Superior;
      }
      // Condiciones dobles para colisiones en las esquinas
    } else if (delta.x < 0.0 && delta.y < 0.0) {
      res.ladoImpacto = LadoImpacto::EsquinaInfIzq;
    } else if (delta.x > 0.0 && delta.y < 0.0) {
      res.ladoImpacto = LadoImpacto::EsquinaInfDer;
    } else if (delta.x < 0.0 && delta.y > 0.0) {
      res.ladoImpacto = LadoImpacto::EsquinaSupIzq;
    } else {
      res.ladoImpacto = LadoImpacto::EsquinaSupDer;
    }
  } else {
    // Caso límite: el centro de la partícula quedó exactamente dentro del
    // cuadrado
    double penIzq = pos.x - xMin();
    double penDer = xMax() - pos.x;
    double penInf = pos.y - yMin();
    double penSup = yMax() - pos.y;
    double minPen = min({penIzq, penDer, penInf, penSup});
    // Por donde va a salir (NORMAL)
    // Inelastico
    if (minPen == penIzq) {
      res.normal = Vector2D(-1.0, 0.0);
      res.ladoImpacto = LadoImpacto::Izquierdo;
    } else if (minPen == penDer) {
      res.normal = Vector2D(1.0, 0.0);
      res.ladoImpacto = LadoImpacto::Derecho;
    } else if (minPen == penInf) {
      res.normal = Vector2D(0.0, -1.0);
      res.ladoImpacto = LadoImpacto::Inferior;
    } else {
      res.normal = Vector2D(0.0, 1.0);
      res.ladoImpacto = LadoImpacto::Superior;
    }
  }

  return res;
}

bool Obstaculo::resolverColision(Particula &p,
                                 ResultadoColisionObstaculo &resultado) const {
  resultado = detectarColision(p);
  if (!resultado.huboColision) {
    return false;
  }

  const Vector2D &vel = p.getVelocidad();
  double vNormal = vel.productoPunto(resultado.normal);

  // Corregir posición para sacar la partícula fuera del obstáculo
  p.setPosicion(resultado.puntoContacto + resultado.normal * p.getRadio());

  // Solo rebotar si la partícula se está acercando al plano de impacto
  if (vNormal < 0.0) {
    Vector2D vPerp = resultado.normal * vNormal;
    Vector2D vPar = vel - vPerp;

    // v'_perp = -e * v_perp, v'_par = v_par
    Vector2D nuevaVel = vPar - vPerp * coefRestitucion;
    p.setVelocidad(nuevaVel);
    return true;
  }

  return false;
}
