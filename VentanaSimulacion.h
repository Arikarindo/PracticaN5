#ifndef VENTANASIMULACION_H
#define VENTANASIMULACION_H

#include "Simulador.h"
#include <QGraphicsEllipseItem>
#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsView>
#include <QMainWindow>
#include <QTimer>
#include <map>
#include <vector>

using namespace std;

// Ventana sencilla en Qt que anima automáticamente las partículas y dibuja sus
// trayectorias y colisiones
class VentanaSimulacion : public QMainWindow {
private:
const Simulador &simulador;
QGraphicsScene *escena;
QGraphicsView *vista;
QTimer timer;

double escala;
int pasoActual;
int colisionActual;

vector<vector<PuntoTrayectoria>> frames;
map<int, QGraphicsEllipseItem *> circulos;
map<int, QGraphicsSimpleTextItem *> etiquetas;
map<int, QPointF> ultimaPos;

QPointF aPantalla(const Vector2D &pos) const;
void dibujarEscenario();
void avanzarPaso();

public:
explicit VentanaSimulacion(const Simulador &sim, QWidget *parent = nullptr);
}
;

#endif // VENTANASIMULACION_H
