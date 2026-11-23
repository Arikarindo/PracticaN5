#include "Simulador.h"
#include "VentanaSimulacion.h"
#include <QApplication>
#include <iostream>

using namespace std;

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);

    // Configuración de la caja: 100 x 80 unidades, paso dt = 0.01 s, duración 12 s
    const double anchoCaja = 100.0;
    const double altoCaja = 80.0;
    const double dt = 0.01;
    const double tiempoTotal = 12.0;

    Simulador sim(anchoCaja, altoCaja, dt, tiempoTotal);

    // 4 obstáculos cuadrados estáticos dentro de la caja (coeficiente de restitución e = 0.80)
    const double coefRestitucion = 0.80;
    sim.agregarObstaculo(Vector2D(30.0, 25.0), 10.0, coefRestitucion);
    sim.agregarObstaculo(Vector2D(70.0, 25.0), 10.0, coefRestitucion);
    sim.agregarObstaculo(Vector2D(30.0, 55.0), 10.0, coefRestitucion);
    sim.agregarObstaculo(Vector2D(70.0, 55.0), 10.0, coefRestitucion);

    // 5 partículas simultáneas (mínimo requerido: 4)
    sim.agregarParticula(Vector2D(12.0, 40.0), Vector2D(18.0, 12.0), 2.0, 2.5);   // P1
    sim.agregarParticula(Vector2D(88.0, 42.0), Vector2D(-17.0, 10.0), 3.0, 2.8);  // P2
    sim.agregarParticula(Vector2D(50.0, 10.0), Vector2D(8.0, 22.0), 1.5, 2.2);    // P3
    sim.agregarParticula(Vector2D(50.0, 70.0), Vector2D(-10.0, -19.0), 2.5, 2.6); // P4
    sim.agregarParticula(Vector2D(15.0, 15.0), Vector2D(14.0, 16.0), 1.8, 2.0);   // P5

    sim.ejecutar();

    if (!sim.guardarResultadosTxt("simulacion_salida.txt")) {
        cerr << "Error al guardar simulacion_salida.txt\n";
    }

    VentanaSimulacion ventana(sim);
    ventana.show();

    return app.exec();
}
