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
  control::State u0;
  u0 << 1.0, 0.1;

  double noise_position = 0.1;
  double noise_velocity = 0.3;
  sensors::GPSSensor sensor(noise_position, noise_velocity);
  sensor.measure(0, x0);

  sensormodel::GPSSensor model_sensor(noise_position, noise_velocity);
  worldmodel::World model_world(gamma_v, gamma_theta, noise_v, noise_theta);
  filters::ParticleFilter filter_particles(model_world, model_sensor);

  int Nparticles = 200;
  filters::Particles believe_particles(Nparticles);
  for (auto& particle: believe_particles) particle = worldmodel::State::Zero();

  QLineSeries *series_worldstate = new QLineSeries();
  series_worldstate->setName("World");
  QLineSeries *series_measuredstate = new QLineSeries();
  series_measuredstate->setName("GPSSensor");
  QLineSeries *series_particles = new QLineSeries();
  series_particles->setName("Particle Filter");
  std::vector<QLineSeries*> series_singleparticles;
  for (int i=0; i<Nparticles; i++) series_singleparticles.push_back(new QLineSeries);
  QColor color(122, 122, 122);
  for (int i=0; i<Nparticles; i++) {
    series_singleparticles[i]->setColor(color);
    series_singleparticles[i]->setOpacity(0.1);
    series_singleparticles[i]->append(believe_particles[i](0), believe_particles[i](1));
  }
  series_worldstate->append(x0(0), x0(1));
  series_measuredstate->append(sensor.x, sensor.y);
  worldmodel::State guess_particles = filter_particles.guess(believe_particles);
  series_particles->append(guess_particles(0), guess_particles(1));
  for (int i=0; i<Nparticles; i++) {
    series_singleparticles[i]->append(believe_particles[i](0), believe_particles[i](1));
  }

  int N = 50;
  world::State x = x0;
  measurement::State z;
  for (int i=1; i<N; i++) {
    x = world.step(x, u0);
    sensor.measure(i, x);
    z << sensor.x, sensor.y, sensor.vx, sensor.vy;
    believe_particles = filter_particles.filter(ticktime, believe_particles, u0, z);

    series_worldstate->append(x(0), x(1));
    series_measuredstate->append(sensor.x, sensor.y);
    guess_particles = filter_particles.guess(believe_particles);
    series_particles->append(guess_particles(0), guess_particles(1));
    for (int i=0; i<Nparticles; i++) series_singleparticles[i]->append(believe_particles[i](0), believe_particles[i](1));
  }

  QApplication app(argc, argv);

  QChart *chart = new QChart();
  for (int i=0; i<Nparticles; i++) {
    chart->addSeries(series_singleparticles[i]);
    chart->legend()->markers(series_singleparticles[i])[0]->setVisible(0);
  }
  chart->addSeries(series_worldstate);
  chart->addSeries(series_measuredstate);
  chart->addSeries(series_particles);


  chart->createDefaultAxes();
  chart->setTitle("Particle Filter");

  QChartView *chartView = new QChartView(chart);
  chartView->setRenderHint(QPainter::Antialiasing);

  QMainWindow window;
  window.setCentralWidget(chartView);
  window.resize(400, 300);
  window.show();

  return app.exec();
}
