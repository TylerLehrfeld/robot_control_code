#include "jacobian.h"
#include "templated_classes/Templated_Point.h"
#include "templated_classes/Templated_Transform.h"
#include <vector>
#include <fstream>
#include <cmath>
#include <iostream>
#include <algorithm>
struct LogEntry {
  int index;
  Point<double> target, actual;
  double targeting_error;
  double angular_error;
  Transform<double> expected_F_EE;
  Transform<double> actual_F_EE;
  Transform<double> F_OM1;
  Transform<double> F_OM2;
};
const std::vector<std::vector<int>> vec_inliers = {
    {0, 1, 1, 1, 1, 1, 1, 0, 0, 1}, {0, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {0, 1, 1, 1, 1, 1, 1, 1, 1, 1}, {0, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {0, 1, 1, 1, 1, 0, 1, 1, 1, 1}, {0, 1, 1, 1, 1, 1, 1, 0, 0, 1},
    {0, 1, 1, 1, 1, 1, 1, 0, 0, 1}, {0, 1, 1, 1, 1, 1, 1, 0, 0, 1},
    {0, 1, 1, 1, 1, 1, 1, 0, 0, 1}, {0, 1, 1, 1, 1, 1, 1, 0, 0, 1}};
const int num_positions = 10;
std::vector<LogEntry> read_log_file(const std::string &filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open log file: " + filename);
  }

  std::vector<LogEntry> entries;
  std::string line;

  // Skip header
  std::getline(file, line);

  while (std::getline(file, line)) {
    if (line.empty())
      continue;

    std::stringstream ss(line);
    std::string token;
    std::vector<double> values;

    // Split by comma
    while (std::getline(ss, token, ',')) {
      try {
        values.push_back(std::stod(token));
      } catch (...) {
        // Handle possible parsing errors gracefully
        values.push_back(0.0);
      }
    }

    if (values.size() != 1 + 3 + 3 + 2 + 9 + 9 + 3 + 9 + 3 + 9) {
      std::cerr << "Skipping malformed line with " << values.size()
                << " values\n";
      continue;
    }

    LogEntry e;
    int i = 0;
    e.index = static_cast<int>(values[i++]);
    e.target = {values[i++], values[i++], values[i++]};
    e.actual = {values[i++], values[i++], values[i++]};
    e.targeting_error = values[i++];
    e.angular_error = values[i++];

    // expected_F_EE.R
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        e.expected_F_EE.R.matrix[r][c] = values[i++];
    // actual_F_EE.R
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        e.actual_F_EE.R.matrix[r][c] = values[i++];

    // F_OM1 position
    e.F_OM1.p = {values[i++], values[i++], values[i++]};
    // F_OM1 rotation
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        e.F_OM1.R.matrix[r][c] = values[i++];

    // F_OM2 position
    e.F_OM2.p = {values[i++], values[i++], values[i++]};
    // F_OM2 rotation
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        e.F_OM2.R.matrix[r][c] = values[i++];

    entries.push_back(e);
  }

  return entries;
}

// --- Helper math utilities ---
double rotation_angle_deg(Rotation<double> &Rdiff) {
  double trace = Rdiff.matrix[0][0] + Rdiff.matrix[1][1] + Rdiff.matrix[2][2];
  double val = (trace - 1.0) / 2.0;
  val = std::clamp(val, -1.0, 1.0);
  return std::acos(val) * 180.0 / M_PI;
}

struct precision_struct {
  double angular_precision;
  double targeting_precision;
};

Point<double>
get_average_rotation(const std::vector<Transform<double>> &transforms) {

  Point<double> z;
  Point<double> tot = {0, 0, 0};
  for (int i = 0; i < transforms.size(); i++) {
    z = {transforms[i].R.matrix[0][2], transforms[i].R.matrix[1][2],
         transforms[i].R.matrix[2][2]};
    z.print_desmos();
    tot = tot + z;
  }
  tot = tot * (1 / (float)transforms.size());
  tot.print_desmos();
  return tot.normalize();
}

Point<double>
get_average_translation(const std::vector<Transform<double>> &transforms) {
  Point<double> t;
  Point<double> tot = {0, 0, 0};
  for (int i = 0; i < transforms.size(); i++) {
    tot = tot + transforms[i].p;
  }
  tot = tot * (1 / (float)transforms.size());
  return tot;
}

std::vector<double>
get_angular_diffs(const Point<double> &avg,
                  const std::vector<Transform<double>> &transforms) {
  std::vector<double> ret;
  Point<double> z;
  for (Transform<double> transform : transforms) {

    z = {transform.R.matrix[0][2], transform.R.matrix[1][2],
         transform.R.matrix[2][2]};
    ret.push_back(std::acos(std::clamp<double>(avg * z, -1, 1)));
  }
  return ret;
}
std::vector<Point<double>>
get_position_diffs(const Point<double> &average_translation,
                   const std::vector<Transform<double>> &transforms) {
  Point<double> diff;
  std::vector<Point<double>> diffs;
  for (Transform<double> transform : transforms) {
    diffs.push_back(average_translation - transform.p);
  }
  return diffs;
}

void get_precision(const std::vector<Transform<double>> &transforms) {
  std::cout << "translation over time:" << std::endl;
  for (int i = 0; i < transforms.size(); i++) {
    std::cout << transforms[i].p.x << ", " << transforms[i].p.y << ", "
              << transforms[i].p.z << std::endl;
  }
  std::cout << "end ToT." << std::endl;
  Point<double> average_rotation = get_average_rotation(transforms);
  Point<double> average_translation = get_average_translation(transforms);
  std::vector<double> angles = get_angular_diffs(average_rotation, transforms);
  std::vector<Point<double>> diffs =
      get_position_diffs(average_translation, transforms);

  for (int i = 0; i < transforms.size(); i++) {
    Point<double> z = {transforms[i].R.matrix[0][2],
                       transforms[i].R.matrix[1][2],
                       transforms[i].R.matrix[2][2]};

    double needle_vec_len = std::sqrt(115 * 115 + 47.7 * 47.7);
    double needle_error =
        (needle_vec_len * (average_rotation + (-1 *z))).magnitude() + diffs[i].magnitude();
    std::cout << angles[i] << ", " << diffs[i].x << ", " << diffs[i].y << ", "
              << diffs[i].z << ", " << needle_error << std::endl;
  }
}

int main() {
  std::vector<LogEntry> entries;
  try {
    entries = read_log_file("precision_many_points4.csv");
    std::cout << "Loaded " << entries.size() << " entries.\n";
  } catch (const std::exception &ex) {
    std::cerr << "Error: " << ex.what() << "\n";
  }
  // std::vector<Transform<double>> homes;
  // std::vector<Transform<double>> targets;
  std::vector<std::vector<Transform<double>>> vecs;
  for (int i = 0; i < num_positions; i++) {
    vecs.push_back(std::vector<Transform<double>>());
  }
  for (int i = 0; i < entries.size() / num_positions; i++) {
    for (int j = 0; j < num_positions; j++) {
      vecs[j].push_back(entries[i * num_positions + j].F_OM1.inverse() *
                        entries[i * num_positions + j].F_OM2 * F_M2N<double>());
      // vecs[j].push_back(entries[i * num_positions + j].F_OM1.inverse() *
      //                   entries[i * num_positions + j].F_OM2);
    }
  }

  // get precision of these transforms:
  for (int i = 0; i < num_positions; i++) {
    get_precision(vecs[i]);
  }
}
