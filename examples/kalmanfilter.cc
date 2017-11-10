#include <iostream>

#include <QApplication>
#include <QPushButton>
#include <qt/QtCharts/QtCharts>

#include "mobilerobot.h"

int main(int argc, char **argv) {
  double ticktime = 0.1;
  double substeps = 1;
  double gamma_v = 0.1;
  double gamma_theta = 0.1;
  double noise_v = 0.1;
  double noise_theta = 0.2;

  world::World world(ticktime, substeps, gamma_v, gamma_theta, noise_v, noise_theta);
  world::State x0;
  x0 << 0, 0, 0.5, 0;
  world::Control u0;
  u0 << 1.0, 0.1;

  double noise_position = 1.2;
  double noise_velocity = 0.3;
  sensors::GPSSensor sensor(noise_position, noise_velocity);
  sensor.measure(0, x0);

  filters::ExtendedKalmanFilter filter(ticktime, gamma_v, gamma_theta,
                                       noise_v, noise_theta,
                                       noise_position, noise_velocity);
  filters::KalmanProbability believe;
  believe.mu << sensor.x, sensor.y, sqrt(pow(sensor.vx, 2) + pow(sensor.vy, 2)), atan2(sensor.vy, sensor.vx);
  believe.sigma = filters::Covariance::Zero();
  believe.sigma(0, 0) = noise_position;
  believe.sigma(1, 1) = noise_position;
  believe.sigma(2, 2) = noise_velocity;
  believe.sigma(3, 3) = 0.1;

  QLineSeries *series_worldstate = new QLineSeries();
  QLineSeries *series_measuredstate = new QLineSeries();
  QLineSeries *series_believestate = new QLineSeries();

  series_worldstate->append(x0(0), x0(1));
  series_measuredstate->append(sensor.x, sensor.y);
  series_believestate->append(believe.mu(0), believe.mu(1));

  int N = 100;
  world::State x = x0;
  sensors::Measurement z;
  for (int i=1; i<N; i++) {
    x = world.step(x, u0);
    sensor.measure(i, x);
    z << sensor.x, sensor.y, sensor.vx, sensor.vy;
    believe = filter.filter(believe, u0, z);

    series_worldstate->append(x(0), x(1));
    series_measuredstate->append(sensor.x, sensor.y);
    series_believestate->append(believe.mu(0), believe.mu(1));

    std::cout << "t=" << ticktime*i << ": " << x.transpose() << std::endl;
  }

  QApplication app(argc, argv);

  QChart *chart = new QChart();
  chart->legend()->hide();
  chart->addSeries(series_worldstate);
  chart->addSeries(series_measuredstate);
  chart->addSeries(series_believestate);
  chart->createDefaultAxes();
  chart->setTitle("Simple line chart example");

  QChartView *chartView = new QChartView(chart);
  chartView->setRenderHint(QPainter::Antialiasing);

  QMainWindow window;
  window.setCentralWidget(chartView);
  window.resize(400, 300);
  window.show();


  return app.exec();
}
