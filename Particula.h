#ifndef PARTICULA_H
#define PARTICULA_H

#include "Vector2D.h"

using namespace std;

// Modela una partícula circular con masa, radio, posición y velocidad.
class Particula {
private:
    int id;
    Vector2D posicion;
    Vector2D velocidad;
    double masa;
    double radio;
    bool activa;
    int idPadre1;
    int idPadre2;

public:
    Particula(int id, const Vector2D& posicion, const Vector2D& velocidad,
              double masa, double radio, int idPadre1 = 0, int idPadre2 = 0);

    int getId() const;
    const Vector2D& getPosicion() const;
    const Vector2D& getVelocidad() const;
    double getMasa() const;
    double getRadio() const;
    bool estaActiva() const;
    int getIdPadre1() const;
    int getIdPadre2() const;

    void setPosicion(const Vector2D& nuevaPos);
    void setVelocidad(const Vector2D& nuevaVel);
    void desactivar();

    // Actualiza la posición en un paso discreto dt: r(t + dt) = r(t) + v * dt.
    void mover(double dt);

    double energiaCinetica() const;
    Vector2D momentoLineal() const;

    // Verifica si esta partícula se solapa o toca a otra partícula.
    bool colisionaCon(const Particula& otra) const;

    // Fusiona dos partículas mediante colisión completamente inelástica.
    // Conserva el momento lineal total, ubica el centro de masa y conserva el área.
    static Particula fusionar(int nuevoId, const Particula& p1, const Particula& p2);
};

#endif // PARTICULA_H
