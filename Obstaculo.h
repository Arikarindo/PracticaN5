#ifndef OBSTACULO_H
#define OBSTACULO_H

#include "Particula.h"
#include "Vector2D.h"

using namespace std;

// Identifica con qué lado o esquina del obstáculo cuadrado ocurrió el impacto.
enum class LadoImpacto {
  Ninguno,
  Izquierdo,
  Derecho,
  Inferior,
  Superior,
  EsquinaInfIzq,
  EsquinaInfDer,
  EsquinaSupIzq,
  EsquinaSupDer
};

// Convierte LadoImpacto a texto constante para impresión/reporte.
const char *nombreLadoImpacto(LadoImpacto lado);

// Información geométrica resultante de detectar una colisión contra un obstáculo cuadrado.
struct ResultadoColisionObstaculo {
  bool huboColision = false;
  Vector2D puntoContacto;
  Vector2D normal;
  LadoImpacto ladoImpacto = LadoImpacto::Ninguno;
};

class Obstaculo {
private:
  int id;
  Vector2D centro;
  double lado;
  double coefRestitucion;

public:
  Obstaculo(int id, const Vector2D &centro, double lado,
            double coefRestitucion);

  int getId() const;
  const Vector2D &getCentro() const;
  double getLado() const;
  double getCoefRestitucion() const;

  double xMin() const;
  double xMax() const;
  double yMin() const;
  double yMax() const;

  // Detecta si la partícula circular colisiona con el cuadrado y determina el
  // lado de impacto.

  ResultadoColisionObstaculo detectarColision(const Particula &p) const;

  // Aplicar rebote, o colision inelastica
  bool resolverColision(Particula &p,
                        ResultadoColisionObstaculo &resultado) const;
};

#endif // OBSTACULO_H
