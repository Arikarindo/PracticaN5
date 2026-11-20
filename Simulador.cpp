#include "Simulador.h"
#include <fstream>
#include <iomanip>

using namespace std;

const char *nombreTipoColision(TipoColision tipo) {
  switch (tipo) {
  case TipoColision::Pared:
    return "Pared";
  case TipoColision::Obstaculo:
    return "Obstaculo";
  case TipoColision::ParticulaParticula:
    return "Particula-Particula";
  default:
    return "Desconocido";
  }
}

const char *nombreParedCaja(ParedCaja pared) {
  switch (pared) {
  case ParedCaja::Izquierda:
    return "Pared Izquierda";
  case ParedCaja::Derecha:
    return "Pared Derecha";
  case ParedCaja::Inferior:
    return "Pared Inferior";
  case ParedCaja::Superior:
    return "Pared Superior";
  default:
    return "Ninguna";
  }
}

Simulador::Simulador(double anchoCaja, double altoCaja, double dt,
                     double tiempoTotal)
    : anchoCaja(anchoCaja), altoCaja(altoCaja), dt(dt),
      tiempoTotal(tiempoTotal), siguienteIdParticula(1) {}

void Simulador::agregarParticula(const Vector2D &posicion,
                                 const Vector2D &velocidad, double masa,
                                 double radio) {
  //
  particulas.emplace_back(siguienteIdParticula++, posicion, velocidad, masa,
                          radio, 0, 0);
}

void Simulador::agregarObstaculo(const Vector2D &centro, double lado,
                                 double coefRestitucion) {
  int idObs = static_cast<int>(obstaculos.size()) + 1;
  // Crear
  obstaculos.emplace_back(idObs, centro, lado, coefRestitucion);
}

void Simulador::registrarEstado(double t) {
  for (const auto &p : particulas) {
    if (!p.estaActiva()) {
      continue;
    }
    // tiempo,....
    trayectoria.push_back({t, p.getId(), p.getPosicion(), p.getVelocidad(),
                           p.getMasa(), p.getRadio(), p.getIdPadre1(),
                           p.getIdPadre2()});
  }
}

void Simulador::resolverColisionParedes(Particula &p, double t) {
  if (!p.estaActiva()) {
    return;
  }

  Vector2D pos = p.getPosicion();
  Vector2D vel = p.getVelocidad();
  double r = p.getRadio();

  // Pared izquierda (x = 0)
  if (pos.x - r <= 0.0) {
    pos.x = r;
    if (vel.x < 0.0) {
      vel.x = -vel.x;
      colisiones.push_back({t, TipoColision::Pared, p.getId(), 0, 0,
                            ParedCaja::Izquierda, LadoImpacto::Ninguno, pos});
    }
  }
  // Pared derecha (x = anchoCaja)
  else if (pos.x + r >= anchoCaja) {
    pos.x = anchoCaja - r;
    if (vel.x > 0.0) {
      vel.x = -vel.x;
      colisiones.push_back({t, TipoColision::Pared, p.getId(), 0, 0,
                            ParedCaja::Derecha, LadoImpacto::Ninguno, pos});
    }
  }

  // Pared inferior (y = 0)
  if (pos.y - r <= 0.0) {
    pos.y = r;
    if (vel.y < 0.0) {
      vel.y = -vel.y;
      colisiones.push_back({t, TipoColision::Pared, p.getId(), 0, 0,
                            ParedCaja::Inferior, LadoImpacto::Ninguno, pos});
    }
  }
  // Pared superior (y = altoCaja)
  else if (pos.y + r >= altoCaja) {
    pos.y = altoCaja - r;
    if (vel.y > 0.0) {
      vel.y = -vel.y;
      colisiones.push_back({t, TipoColision::Pared, p.getId(), 0, 0,
                            ParedCaja::Superior, LadoImpacto::Ninguno, pos});
    }
  }

  p.setPosicion(pos);
  p.setVelocidad(vel);
}

void Simulador::resolverColisionesObstaculos(Particula &p, double t) {
  if (!p.estaActiva()) {
    return;
  }

  for (const auto &obs : obstaculos) {
    ResultadoColisionObstaculo res;
    if (obs.resolverColision(p, res)) {
      colisiones.push_back({t, TipoColision::Obstaculo, p.getId(), obs.getId(),
                            0, ParedCaja::Ninguna, res.ladoImpacto,
                            res.puntoContacto});
    }
  }
}

void Simulador::resolverColisionesEntreParticulas(double t) {
  vector<Particula> nuevasParticulas;
  //
  int n = particulas.size();

  for (int i = 0; i < n; ++i) {
    if (!particulas[i].estaActiva()) {
      continue;
    }
    for (int j = i + 1; j < n; ++j) {
      if (!particulas[j].estaActiva()) {
        continue;
      }
      if (particulas[i].colisionaCon(particulas[j])) {
        int nuevoId = siguienteIdParticula++;
        Particula fusionada =
            Particula::fusionar(nuevoId, particulas[i], particulas[j]);

        colisiones.push_back({t, TipoColision::ParticulaParticula,
                              particulas[i].getId(), particulas[j].getId(),
                              nuevoId, ParedCaja::Ninguna, LadoImpacto::Ninguno,
                              fusionada.getPosicion()});

        particulas[i].desactivar();
        particulas[j].desactivar();
        nuevasParticulas.push_back(fusionada);
        break; // La partícula i ya se fusionó en este paso
      }
    }
  }

  for (auto &nueva : nuevasParticulas) {
    resolverColisionParedes(nueva, t);
    resolverColisionesObstaculos(nueva, t);
    particulas.push_back(nueva);
  }
}

void Simulador::ejecutar() {
  trayectoria.clear();
  colisiones.clear();

  double t = 0.0;
  registrarEstado(t);

  int pasos = static_cast<int>(tiempoTotal / dt + 0.5);
  for (int paso = 1; paso <= pasos; ++paso) {
    t = paso * dt;

    // 1. Actualizar posiciones de todas las partículas activas
    for (auto &p : particulas) {
      if (p.estaActiva()) {
        p.mover(dt);
      }
    }

    // 2. Resolver colisiones con paredes y obstáculos
    for (auto &p : particulas) {
      if (p.estaActiva()) {
        resolverColisionParedes(p, t);
        resolverColisionesObstaculos(p, t);
      }
    }

    // 3. Resolver colisiones entre partículas (fusión completamente inelástica)
    resolverColisionesEntreParticulas(t);

    // 4. Registrar posiciones del instante actual
    registrarEstado(t);
  }
}

bool Simulador::guardarResultadosTxt(const char *rutaArchivo) const {
  ofstream archivo(rutaArchivo);
  if (!archivo.is_open()) {
    return false;
  }

  archivo << fixed << setprecision(4);
  archivo << "# SIMULACION DE COLISIONES MULTIPLES (ACTIVIDAD 2.1)\n";
  archivo << "# Caja: ancho=" << anchoCaja << ", alto=" << altoCaja
          << ", dt=" << dt << ", tiempo_total=" << tiempoTotal << "\n";
  archivo << "# Obstaculos cuadrados: " << obstaculos.size() << "\n";
  for (const auto &obs : obstaculos) {
    archivo << "#   Obstaculo " << obs.getId() << ": centro=("
            << obs.getCentro().x << ", " << obs.getCentro().y << ")"
            << ", lado=" << obs.getLado() << ", e=" << obs.getCoefRestitucion()
            << "\n";
  }

  archivo << "\n[POSICIONES]\n";
  archivo << "tiempo\tid_particula\tx\ty\tvx\tvy\tmasa\tradio\torigen\n";
  for (const auto &pt : trayectoria) {
    archivo << pt.tiempo << "\t" << pt.idParticula << "\t" << pt.posicion.x
            << "\t" << pt.posicion.y << "\t" << pt.velocidad.x << "\t"
            << pt.velocidad.y << "\t" << pt.masa << "\t" << pt.radio << "\t";
    if (pt.idPadre1 == 0) {
      archivo << "Inicial\n";
    } else {
      archivo << "Fusion(P" << pt.idPadre1 << "+P" << pt.idPadre2 << ")\n";
    }
  }

  archivo << "\n[COLISIONES]\n";
  archivo << "tiempo\ttipo\tid_particula\tid_objetivo\tx\ty\tdetalle\n";
  for (const auto &col : colisiones) {
    archivo << col.tiempo << "\t" << nombreTipoColision(col.tipo) << "\t"
            << col.idParticula << "\t" << col.idObjetivo << "\t"
            << col.posicion.x << "\t" << col.posicion.y << "\t";

    if (col.tipo == TipoColision::Pared) {
      archivo << nombreParedCaja(col.pared) << "\n";
    } else if (col.tipo == TipoColision::Obstaculo) {
      archivo << "Obstaculo " << col.idObjetivo << " - "
              << nombreLadoImpacto(col.ladoObstaculo) << "\n";
    } else {
      archivo << "Fusion P" << col.idParticula << " + P" << col.idObjetivo
              << " -> P" << col.idResultado << "\n";
    }
  }

  return true;
}

double Simulador::getAnchoCaja() const { return anchoCaja; }
double Simulador::getAltoCaja() const { return altoCaja; }
const vector<Particula> &Simulador::getParticulas() const { return particulas; }
const vector<Obstaculo> &Simulador::getObstaculos() const { return obstaculos; }
const vector<PuntoTrayectoria> &Simulador::getTrayectoria() const {
  return trayectoria;
}
const vector<RegistroColision> &Simulador::getColisiones() const {
  return colisiones;
}
