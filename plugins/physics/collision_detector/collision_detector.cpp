// Author(s):         Chansol Hong, Taeyoung Kim

#include <ode/ode.h>
#include <plugins/physics.h>
#include <stdbool.h>

#define MAX_CONTACTS 10
#define ROBOT_COUNT 3

static pthread_mutex_t mutex;

const char robot_name[2][3][12] = {
  {"DEF_ROBOTR0", "DEF_ROBOTR1", "DEF_ROBOTR2"},
  {"DEF_ROBOTB0", "DEF_ROBOTB1", "DEF_ROBOTB2"}
};
const char head_name[2][3][22] = {
  {"DEF_ROBOTR0.HEADSHAPE", "DEF_ROBOTR1.HEADSHAPE", "DEF_ROBOTR2.HEADSHAPE"},
  {"DEF_ROBOTB0.HEADSHAPE", "DEF_ROBOTB1.HEADSHAPE", "DEF_ROBOTB2.HEADSHAPE"}
};
const char leftarm_name[2][34] = {"DEF_ROBOTR0.LEFTARM_JOINT.LEFTARM", "DEF_ROBOTB0.LEFTARM_JOINT.LEFTARM"};
const char rightarm_name[2][36] = {"DEF_ROBOTR0.RIGHTARM_JOINT.RIGHTARM", "DEF_ROBOTB0.RIGHTARM_JOINT.RIGHTARM"};

// plugin variables
static dGeomID robot_geom[2][3] = {{NULL, NULL, NULL}, {NULL, NULL, NULL}};
static dGeomID frontslider_geom[2][3] = {{NULL, NULL, NULL}, {NULL, NULL, NULL}};
static dGeomID arm_geom[2][2] = {{NULL, NULL}, {NULL, NULL}};
static dGeomID ball_geom = NULL;
static dGeomID robot_ceiling_geom = NULL;

bool robot_collision[2][3] = {{false, false, false}, {false, false, false}};

int step = 0;
void webots_physics_init() {
  pthread_mutex_init(&mutex, NULL);

  // get ODE handles to .wbt objects
  for (int i = 0; i < 2; i++) {
    arm_geom[i][0] = dWebotsGetGeomFromDEF(leftarm_name[i]);
    arm_geom[i][1] = dWebotsGetGeomFromDEF(rightarm_name[i]);
    if (arm_geom[i][0] == NULL)
      dWebotsConsolePrintf("%s is missing.", leftarm_name[i]);
    if (arm_geom[i][1] == NULL)
      dWebotsConsolePrintf("%s is missing.", rightarm_name[i]);
    for (int j = 0; j < ROBOT_COUNT; j++) {
      robot_geom[i][j] = dWebotsGetGeomFromDEF(robot_name[i][j]);
      frontslider_geom[i][j] = dWebotsGetGeomFromDEF(head_name[i][j]);
      if (robot_geom[i][j] == NULL)
        dWebotsConsolePrintf("%s is missing.", robot_name[i][j]);
      if (frontslider_geom[i][j] == NULL)
        dWebotsConsolePrintf("%s is missing.", head_name[i][j]);
    }
  }
  ball_geom = dWebotsGetGeomFromDEF("DEF_BALL");
  robot_ceiling_geom = dWebotsGetGeomFromDEF("ROBOTCEILING");
  if (robot_ceiling_geom == NULL)
    dWebotsConsolePrintf("Robot ceiling is missing");
}

void webots_physics_step() {
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < ROBOT_COUNT; j++)
      robot_collision[i][j] = false;
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {
  pthread_mutex_lock(&mutex);
  if (dAreGeomsSame(g1, ball_geom)) {
    for (int i = 0; i < 2; i++) {
      if ((dAreGeomsSame(g2, arm_geom[i][0]))||(dAreGeomsSame(g2, arm_geom[i][1]))) {
        robot_collision[i][0] = true;
      }
      for (int j = 0; j < ROBOT_COUNT; j++) {
        if ((dAreGeomsSame(g2, robot_geom[i][j]))||(dAreGeomsSame(g2, frontslider_geom[i][j]))) {
          robot_collision[i][j] = true;
        }
      }
    }

    if (dAreGeomsSame(g2, robot_ceiling_geom)) {
      pthread_mutex_unlock(&mutex);
      return 1;
    }
  }
  else if (dAreGeomsSame(g2, ball_geom)) {
    for (int i = 0; i < 2; i++) {
      if ((dAreGeomsSame(g1, arm_geom[i][0]))||(dAreGeomsSame(g1, arm_geom[i][1]))) {
        robot_collision[i][0] = true;
      }
      for (int j = 0; j < ROBOT_COUNT; j++) {
        if ((dAreGeomsSame(g1, robot_geom[i][j]))||(dAreGeomsSame(g1, frontslider_geom[i][j]))) {
          robot_collision[i][j] = true;
        }
      }
    }

    if (dAreGeomsSame(g1, robot_ceiling_geom)) {
      pthread_mutex_unlock(&mutex);
      return 1;
    }
  }
  pthread_mutex_unlock(&mutex);

  return 0;
}

void webots_physics_step_end() {
  char collision_packet[2*ROBOT_COUNT];
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < ROBOT_COUNT; j++) {
      collision_packet[i+2*j] = (char)robot_collision[i][j];
    }
  }

  dWebotsSend(0, collision_packet, sizeof(char)*2*ROBOT_COUNT);
}

void webots_physics_cleanup() {
   pthread_mutex_destroy(&mutex);
}
