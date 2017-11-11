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
  filters::ExtendedKalmanFilter filter_kalman(model_world, model_sensor);
  filters::ParticleFilter filter_particles(model_world, model_sensor);
//   filters::ExtendedKalmanFilter filter(ticktime, gamma_v, gamma_theta,
//                                        noise_v, noise_theta,
//                                        noise_position, noise_velocity);
  filters::KalmanProbability believe_kalman;
  believe_kalman.mu << sensor.x, sensor.y, sqrt(pow(sensor.vx, 2) + pow(sensor.vy, 2)), atan2(sensor.vy, sensor.vx);
  believe_kalman.sigma = worldmodel::Matrix::Zero();
  believe_kalman.sigma(0, 0) = noise_position;
  believe_kalman.sigma(1, 1) = noise_position;
  believe_kalman.sigma(2, 2) = noise_velocity;
  believe_kalman.sigma(3, 3) = 0.1;

  int Nparticles = 1000;
  filters::Particles believe_particles(Nparticles);
  for (auto& particle: believe_particles) particle = worldmodel::State::Zero();

  QLineSeries *series_worldstate = new QLineSeries();
  QLineSeries *series_measuredstate = new QLineSeries();
  QLineSeries *series_kalman = new QLineSeries();
  QLineSeries *series_particles = new QLineSeries();

  series_worldstate->append(x0(0), x0(1));
  series_measuredstate->append(sensor.x, sensor.y);
  series_kalman->append(believe_kalman.mu(0), believe_kalman.mu(1));
  worldmodel::State guess_particles = filter_particles.guess(believe_particles);
  series_particles->append(guess_particles(0), guess_particles(1));

  int N = 100;
  world::State x = x0;
  measurement::State z;
  for (int i=1; i<N; i++) {
    x = world.step(x, u0);
    sensor.measure(i, x);
    z << sensor.x, sensor.y, sensor.vx, sensor.vy;
    believe_kalman = filter_kalman.filter(ticktime, believe_kalman, u0, z);
    believe_particles = filter_particles.filter(ticktime, believe_particles, u0, z);

    series_worldstate->append(x(0), x(1));
    series_measuredstate->append(sensor.x, sensor.y);
    series_kalman->append(believe_kalman.mu(0), believe_kalman.mu(1));
    guess_particles = filter_particles.guess(believe_particles);
    series_particles->append(guess_particles(0), guess_particles(1));

    std::cout << "t=" << ticktime*i << ": " << x.transpose() << std::endl;
  }

  QApplication app(argc, argv);

  QChart *chart = new QChart();
  chart->legend()->hide();
  chart->addSeries(series_worldstate);
  chart->addSeries(series_measuredstate);
  chart->addSeries(series_kalman);
  chart->addSeries(series_particles);
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
