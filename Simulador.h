#ifndef SIMULADOR_H
#define SIMULADOR_H

#include "Obstaculo.h"
#include "Particula.h"
#include <vector>

using namespace std;

// clasifica el tipo de colision (char)
enum class TipoColision { Pared, Obstaculo, ParticulaParticula };

// en cual pared reboto?
enum class ParedCaja { Ninguna, Izquierda, Derecha, Inferior, Superior };

const char *nombreTipoColision(TipoColision tipo);
const char *nombreParedCaja(ParedCaja pared);

// Almacena el estado de una partícula en un instante de tiempo dado.
struct PuntoTrayectoria {
  double tiempo;
  int idParticula;
  Vector2D posicion;
  Vector2D velocidad;
  double masa;
  double radio;
  int idPadre1;
  int idPadre2;
};

// registra una colision en simulacióm
struct RegistroColision {
  double tiempo;
  TipoColision tipo;
  int idParticula;
  int idObjetivo;  // 0 para pared, id obstáculo o id segunda partícula
  int idResultado; // id de nueva partícula tras fusión (0 en otros casos)
  ParedCaja pared;
  LadoImpacto ladoObstaculo;
  Vector2D posicion;
};

// administrador de los objetos reuqeridos para la simulacion
class Simulador {
private:
  double anchoCaja;
  double altoCaja;
  double dt;
  double tiempoTotal;
  int siguienteIdParticula;

  vector<Particula> particulas;
  vector<Obstaculo> obstaculos;
  vector<PuntoTrayectoria> trayectoria;
  vector<RegistroColision> colisiones;

  void registrarEstado(double t);
  void resolverColisionParedes(Particula &p, double t);
  void resolverColisionesObstaculos(Particula &p, double t);
  void resolverColisionesEntreParticulas(double t);

public:
  Simulador(double anchoCaja, double altoCaja, double dt, double tiempoTotal);

  void agregarParticula(const Vector2D &posicion, const Vector2D &velocidad,
                        double masa, double radio);
  void agregarObstaculo(const Vector2D &centro, double lado,
                        double coefRestitucion);

  void ejecutar();
  bool guardarResultadosTxt(const char *rutaArchivo) const;

  double getAnchoCaja() const;
  double getAltoCaja() const;
  const vector<Particula> &getParticulas() const;
  const vector<Obstaculo> &getObstaculos() const;
  const vector<PuntoTrayectoria> &getTrayectoria() const;
  const vector<RegistroColision> &getColisiones() const;
};

#endif // SIMULADOR_H
