#include "VentanaSimulacion.h"
#include <QBrush>
#include <QPainter>
#include <QPen>

using namespace std;

VentanaSimulacion::VentanaSimulacion(const Simulador& sim, QWidget* parent)
    : QMainWindow(parent), simulador(sim), escala(7.5), pasoActual(0), colisionActual(0) {
    setWindowTitle("Practica 5 - Simulacion de Colisiones Multiples");

    double wPx = sim.getAnchoCaja() * escala;
    double hPx = sim.getAltoCaja() * escala;

    escena = new QGraphicsScene(this);
    escena->setSceneRect(-20, -20, wPx + 40, hPx + 40);

    vista = new QGraphicsView(escena, this);
    vista->setRenderHint(QPainter::Antialiasing, true);
    vista->setBackgroundBrush(QBrush(QColor("#f1f5f9")));
    setCentralWidget(vista);
    resize(static_cast<int>(wPx + 80), static_cast<int>(hPx + 80));

    // Agrupar la trayectoria por instante de tiempo (un frame por cada paso dt)
    map<double, vector<PuntoTrayectoria>> porTiempo;
    for (const auto& pt : sim.getTrayectoria()) {
        porTiempo[pt.tiempo].push_back(pt);
    }
    for (const auto& par : porTiempo) {
        frames.push_back(par.second);
    }

    dibujarEscenario();

    connect(&timer, &QTimer::timeout, this, [this]() { avanzarPaso(); });
    timer.start(16); // Inicia el movimiento automáticamente al abrir (~60 FPS)
}

QPointF VentanaSimulacion::aPantalla(const Vector2D& pos) const {
    return QPointF(pos.x * escala, (simulador.getAltoCaja() - pos.y) * escala);
}

void VentanaSimulacion::dibujarEscenario() {
    // Caja rectangular
    escena->addRect(0, 0, simulador.getAnchoCaja() * escala, simulador.getAltoCaja() * escala,
                    QPen(QColor("#0f172a"), 2.5), QBrush(QColor("#ffffff")));

    // 4 obstáculos cuadrados
    for (const auto& obs : simulador.getObstaculos()) {
        QPointF supIzq = aPantalla(Vector2D(obs.xMin(), obs.yMax()));
        double ladoPx = obs.getLado() * escala;

        escena->addRect(supIzq.x(), supIzq.y(), ladoPx, ladoPx,
                        QPen(QColor("#334155"), 2), QBrush(QColor("#cbd5e1")));

        auto* txt = escena->addSimpleText(QString("Obs %1").arg(obs.getId()));
        txt->setPos(supIzq.x() + ladoPx / 2.0 - 18, supIzq.y() + ladoPx / 2.0 - 8);
    }
}

void VentanaSimulacion::avanzarPaso() {
    if (pasoActual >= static_cast<int>(frames.size())) {
        timer.stop();
        return;
    }

    const vector<QColor> colores = {
        QColor("#1f77b4"), QColor("#d62728"), QColor("#2ca02c"),
        QColor("#ff7f0e"), QColor("#9467bd"), QColor("#8c564b")
    };

    // Ocultar partículas anteriores (por si alguna se fusionó y desapareció)
    for (auto& par : circulos) par.second->setVisible(false);
    for (auto& par : etiquetas) par.second->setVisible(false);

    const auto& frame = frames[pasoActual];
    double tActual = frame.front().tiempo;

    for (const auto& pt : frame) {
        QPointF pos = aPantalla(pt.posicion);
        QColor color = colores[(pt.idParticula - 1) % colores.size()];
        double rPx = pt.radio * escala;

        if (circulos.find(pt.idParticula) == circulos.end()) {
            circulos[pt.idParticula] = escena->addEllipse(-rPx, -rPx, 2 * rPx, 2 * rPx,
                                                          QPen(QColor("#0f172a"), 1.2), QBrush(color));
            etiquetas[pt.idParticula] = escena->addSimpleText(QString("P%1").arg(pt.idParticula));
            etiquetas[pt.idParticula]->setBrush(QBrush(color));
        } else {
            QPointF ant = ultimaPos[pt.idParticula];
            escena->addLine(ant.x(), ant.y(), pos.x(), pos.y(), QPen(color, 1.8));
        }

        circulos[pt.idParticula]->setPos(pos);
        circulos[pt.idParticula]->setVisible(true);
        etiquetas[pt.idParticula]->setPos(pos.x() + rPx + 3, pos.y() - rPx - 6);
        etiquetas[pt.idParticula]->setVisible(true);
        ultimaPos[pt.idParticula] = pos;
    }

    // Dibujar marcadores de las colisiones ocurridas hasta este instante
    const auto& colisiones = simulador.getColisiones();
    while (colisionActual < static_cast<int>(colisiones.size()) &&
           colisiones[colisionActual].tiempo <= tActual + 1e-9) {
        QPointF p = aPantalla(colisiones[colisionActual].posicion);
        TipoColision tipo = colisiones[colisionActual].tipo;

        if (tipo == TipoColision::Pared) {
            escena->addEllipse(p.x() - 3.5, p.y() - 3.5, 7, 7,
                               QPen(QColor("#0369a1")), QBrush(QColor("#0ea5e9")));
        } else if (tipo == TipoColision::Obstaculo) {
            escena->addRect(p.x() - 4, p.y() - 4, 8, 8,
                            QPen(QColor("#b45309")), QBrush(QColor("#f59e0b")));
        } else {
            escena->addEllipse(p.x() - 7, p.y() - 7, 14, 14,
                               QPen(QColor("#dc2626"), 2.2), Qt::NoBrush);
        }
        ++colisionActual;
    }

    ++pasoActual;
}
