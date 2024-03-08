#include "motors.h"
#include "encoders.h"
#include "pid.h"

#define POP_SIZE 10
#define NUM_PARAMS 3
#define MAX_GENERATIONS 5
#define SETTLE_TOLERANCE 0.2
#define RUN_TIME 5000
#define MUTATION_RATE 0.1

Motors_c motors;
PID_c pid_left;


// Structure to represent an individual (solution)
typedef struct {
  double params[NUM_PARAMS]; // PID parameters: Kp, Ki, Kd
  double fitness; // Fitness score
} Individual;

Individual population[POP_SIZE];

unsigned long update_move_ts;
long left_count_old, right_count_old;
float ave_speed_left, ave_speed_right;
float kp_low, kp_high;
float ki_low, ki_high;
float kd_low, kd_high;
int generation;

void printFitnessResults(Individual population[]) {

  sortPopulationByFitness(population);

  Serial.print("Generation ");
  Serial.println(generation);
  Serial.print("Best Kp/Ki/Kd/fitness: ");
  for (int i = 0; i < NUM_PARAMS; i++) {
    Serial.print(population[0].params[i]);
    Serial.print("/");
  }
  Serial.println(population[0].fitness);
  Serial.print("Worst Kp/fitness: ");
  for (int i = 0; i < NUM_PARAMS; i++) {
    Serial.print(population[POP_SIZE - 1].params[i]);
    Serial.print("/");
  }
  Serial.println(population[POP_SIZE - 1].fitness);
}


void sortPopulationByFitness(Individual population[]) {
  for (int i = 0; i < POP_SIZE - 1; i++) {
    for (int j = 0; j < POP_SIZE - i - 1; j++) {
      if (population[j].fitness > population[j + 1].fitness) {
        // Swap the Individuals
        Individual temp = population[j];
        population[j] = population[j + 1];
        population[j + 1] = temp;
      }
    }
  }
}


float generateRandomFloat(float min_val, float max_val) {
  // Scale rand() to [0, 1]
  float scale = rand() / (float) RAND_MAX;
  // Adjust scale to the specified range [min_val, max_val]
  return min_val + scale * (max_val - min_val);
}


void generateNewPopulation(Individual population[]) {

  for (int i = 0; i < POP_SIZE; i++) {

    population[i].params[0] = generateRandomFloat(kp_low, kp_high);
    population[i].params[1] = generateRandomFloat(ki_low, ki_high);
    population[i].params[2] = generateRandomFloat(kd_low, kd_high);
  }
}


// Simple one-point crossover
void crossover(const Individual &parent1, const Individual &parent2, Individual &child) {
  int crossoverPoint = rand() % NUM_PARAMS; // For more parameters, choose a point to crossover
  for (int i = 0; i < NUM_PARAMS; i++) {
    if (i < crossoverPoint) child.params[i] = parent1.params[i];
    else child.params[i] = parent2.params[i];
  }
}


// Mutation with a small chance
void mutate(Individual &individual) {

  if ((float)rand() / RAND_MAX < MUTATION_RATE) {
    individual.params[0] = generateRandomFloat(kp_low, kp_high);
  }

  if ((float)rand() / RAND_MAX < MUTATION_RATE) {
    individual.params[1] = generateRandomFloat(ki_low, ki_high);
  }

  if ((float)rand() / RAND_MAX < MUTATION_RATE) {
    individual.params[2] = generateRandomFloat(kd_low, kd_high);
  }


}


void evolvePopulation(Individual population[]) {

  Individual newPopulation[POP_SIZE];

  sortPopulationByFitness(population);

  // Keep the top 3 individuals
  for (int i = 0; i < 3; i++) {
    newPopulation[i] = population[i];
  }

  // Fill the rest of the new population
  for (int i = 3; i < POP_SIZE; i++) {
    // Select two random parents from the top 3
    int parent1Index = rand() % 3;
    int parent2Index = rand() % 3;
    while (parent2Index == parent1Index) { // Ensure they are not the same
      parent2Index = rand() % 3;
    }

    // Perform crossover
    crossover(population[parent1Index], population[parent2Index], newPopulation[i]);

    // Mutate the new individual
    mutate(newPopulation[i]);
  }

  // Replace the old population with the new population
  for (int i = 0; i < POP_SIZE; i++) {
    population[i] = newPopulation[i];
  }

}


void loop() {

  for (int i = 0; i < POP_SIZE; i++) {

    pid_left.init(population[i].params[0], population[i].params[1], population[i].params[2]);
    pid_left.reset();

    unsigned long start_time = millis();
    unsigned long current_time = 0;
    unsigned long settling_time = RUN_TIME;
    float left_demand = 1.5;
    bool outside_tol = true;

    update_move_ts = millis();

    while (current_time < RUN_TIME) {

      unsigned long elapsed_move;
      elapsed_move = millis() - update_move_ts;

      // update every 20ms
      if (elapsed_move > 20) {
        update_move_ts = millis();

        long diff_e_left = left_count_old - count_e_left;
        left_count_old = count_e_left;

        float left_e_speed = (float)diff_e_left;
        left_e_speed /= (float)elapsed_move;

        ave_speed_left = 0.7 * ave_speed_left + 0.3 * left_e_speed;

        float pwm_left = pid_left.update( ave_speed_left, left_demand );
        motors.setMotorPower( pwm_left, 0.0);

        // evaluate settling time
        float error = pid_left.last_error;

        if (abs(error) <= SETTLE_TOLERANCE * left_demand) {

          if (outside_tol) {
            settling_time = millis() - start_time;
            outside_tol = false;
          }
        }
        else {
          settling_time = RUN_TIME;
          outside_tol = true;
        }

      }

      current_time = millis() - start_time;

    }

    population[i].fitness = settling_time;

    motors.setMotorPower(0.0, 0.0);
    delay(500);
  }

  if (generation < MAX_GENERATIONS) {

    printFitnessResults(population);

    evolvePopulation(population);
    generation++;
    delay(1000);
  }
  else {

    printFitnessResults(population);

    while (1) {

      motors.setMotorPower(0.0, 0.0);
    }

  }
}

void setup() {

  srand(analogRead(0));
  randomSeed(analogRead(0));

  motors.init();
  setupEncoderLeft();
  setupEncoderRight();
  kp_low = 30;
  kp_high = 200;
  ki_low = 0.0;
  ki_high = 1.0;
  kd_low = -50.0;
  kd_high = 0.0;
  generateNewPopulation(population);
  generation = 0;

  Serial.begin(9600);
  delay(1500);
  Serial.println("***RESET***");

}

/*
  Serial.print(left_demand);
  Serial.print(",");
  float up_tol = left_demand + (SETTLE_TOLERANCE * left_demand);
  Serial.print(up_tol);
  Serial.print(",");
  float down_tol = left_demand - (SETTLE_TOLERANCE * left_demand);
  Serial.print(down_tol);
  Serial.print(",");
  Serial.print(error);
  Serial.print(",");
  Serial.println();
*/
