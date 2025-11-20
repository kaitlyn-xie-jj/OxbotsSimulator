#include <webots/Supervisor.hpp>
#include <webots/Node.hpp>
#include <webots/Field.hpp>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>
#include <string>

using namespace webots;
using std::cout;
using std::endl;

// ---------- CONFIG ----------
const int PING_COUNT = 16;
const int STEEL_COUNT = 24;

const double ARENA_HALF = 1.0;      // 2m arena → -1.0 to 1.0
const double MARGIN = 0.12;         // avoid walls
const double PING_R = 0.02;
const double STEEL_R = 0.01;
const double BUFFER = 0.005;        // safety spacing
const int MAX_TRIES = 2000;

bool USE_TIME_AS_SEED = true;
int USER_SEED = 12345;

bool USE_PROTO = true;
std::string PING_PROTO_NAME = "PingBall";
std::string STEEL_PROTO_NAME = "SteelBall";
// ----------------------------

Supervisor supervisor;

bool okPosition(double x, double y, const std::vector<std::tuple<double,double,double>>& placed, double r) {
  for (auto& p : placed) {
    double px = std::get<0>(p);
    double py = std::get<1>(p);
    double pr = std::get<2>(p);
    double dist = sqrt((px-x)*(px-x) + (py-y)*(py-y));
    if (dist < (pr + r + BUFFER)) return false;
  }
  return true;
}

std::string makePingString(double x, double y) {
  double z = PING_R;
  if (USE_PROTO) {
    return PING_PROTO_NAME + " { translation " + std::to_string(x) +
           " " + std::to_string(y) + " " + std::to_string(z) + " }";
  }
  // fallback raw Solid
  return "Solid { translation " + std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) +
         " children [ Shape { appearance PBRAppearance { baseColor 1 0.953 0.008 metalness 0 roughness 1 } "
         "geometry Sphere { radius 0.02 } } ] boundingObject Sphere { radius 0.02 } physics Physics { density 50 } }";
}

std::string makeSteelString(double x, double y) {
  double z = STEEL_R;
  if (USE_PROTO) {
    return STEEL_PROTO_NAME + " { translation " + std::to_string(x) +
           " " + std::to_string(y) + " " + std::to_string(z) + " }";
  }
  return "Solid { translation " + std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) +
         " children [ Shape { appearance PBRAppearance { baseColor 0.5 0.5 0.5 metallic 1 roughness 0.3 } "
         "geometry Sphere { radius 0.01 } } ] boundingObject Sphere { radius 0.01 } physics Physics { density 7800 } }";
}

int main() {
  int timeStep = (int)supervisor.getBasicTimeStep();

  // Seed
  int seed = USE_TIME_AS_SEED ? time(NULL) : USER_SEED;
  srand(seed);
  cout << "[random_ball_spawner] Using seed: " << seed << endl;

  // Delete old balls
  Node* root = supervisor.getRoot();
  Field* children = root->getField("children");

  int count = children->getCount();
  for (int i = count - 1; i >= 0; i--) {
    Node* child = children->getMFNode(i);
    if (!child) continue;

    std::string name = child->getField("name") ?
                       child->getField("name")->getSFString() : "";

    if (name.rfind("Ping", 0) == 0 || name.rfind("Steel", 0) == 0) {
      children->removeMF(i);
    }
  }

  std::vector<std::tuple<double,double,double>> placed;

  auto sampleXY = [](){
    double x = ((double)rand() / RAND_MAX) * (2*(ARENA_HALF - MARGIN)) - (ARENA_HALF - MARGIN);
    double y = ((double)rand() / RAND_MAX) * (2*(ARENA_HALF - MARGIN)) - (ARENA_HALF - MARGIN);
    return std::make_pair(x, y);
  };

  // Place ping balls
  for (int i = 0; i < PING_COUNT; i++) {
    bool placedOK = false;
    for (int t = 0; t < MAX_TRIES; t++) {
      auto xy = sampleXY();
      if (okPosition(xy.first, xy.second, placed, PING_R)) {
        children->importMFNodeFromString(-1, makePingString(xy.first, xy.second));
        placed.push_back({xy.first, xy.second, PING_R});
        placedOK = true;
        break;
      }
    }
    if (!placedOK) cout << "[WARN] Could not place ping ball " << i << endl;
  }

  // Place steel balls
  for (int i = 0; i < STEEL_COUNT; i++) {
    bool placedOK = false;
    for (int t = 0; t < MAX_TRIES; t++) {
      auto xy = sampleXY();
      if (okPosition(xy.first, xy.second, placed, STEEL_R)) {
        children->importMFNodeFromString(-1, makeSteelString(xy.first, xy.second));
        placed.push_back({xy.first, xy.second, STEEL_R});
        placedOK = true;
        break;
      }
    }
    if (!placedOK) cout << "[WARN] Could not place steel ball " << i << endl;
  }

  cout << "[random_ball_spawner] Placed " << placed.size() << " balls." << endl;

  // let things settle for a few steps
  for (int i = 0; i < 10; i++)
    supervisor.step(timeStep);

  return 0;
}