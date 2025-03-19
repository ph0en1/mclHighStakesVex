#include <vector>
#include <random>
#include <cmath>
#include "vex.h"

using namespace vex;

// Define the particle structure
struct Particle {
    double x;
    double y;
    double theta;
    double weight;
};

// Sensor offsets from the center of the robot (in feet)
const double FRONT_SENSOR_OFFSET = 0.583; // 7 inches in feet
const double BACK_SENSOR_OFFSET = -0.583; // -7 inches in feet
const double LEFT_SENSOR_OFFSET = -0.417; // -5 inches in feet
const double RIGHT_SENSOR_OFFSET = 0.417; // 5 inches in feet

// Function to add random noise
double random_noise(double stddev) {
    static std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, stddev);
    return distribution(generator);
}

// Initialize particles
std::vector<Particle> initialize_particles(int num_particles) {
    std::vector<Particle> particles(num_particles);
    for (auto& p : particles) {
        p.x = 6.0 + random_noise(1.0); // Center of the field with some noise
        p.y = 6.0 + random_noise(1.0);
        p.theta = fmod(random_noise(1.0), 2 * M_PI); // Random orientation
        p.weight = 1.0 / num_particles;
    }
    return particles;
}

// Motion update
void motion_update(std::vector<Particle>& particles, double delta_x, double delta_y, double delta_theta) {
    for (auto& p : particles) {
        p.x += delta_x + random_noise(0.1);
        p.y += delta_y + random_noise(0.1);
        p.theta += delta_theta + random_noise(0.1);
    }
}

// Calculate expected sensor readings based on particle's state and sensor offsets
std::vector<double> calculate_expected_sensor_data(const Particle& p) {
    std::vector<double> expected_sensor_data(4);

    // Calculate the position of each sensor
    double front_sensor_x = p.x + FRONT_SENSOR_OFFSET * cos(p.theta);
    double front_sensor_y = p.y + FRONT_SENSOR_OFFSET * sin(p.theta);
    double back_sensor_x = p.x + BACK_SENSOR_OFFSET * cos(p.theta);
    double back_sensor_y = p.y + BACK_SENSOR_OFFSET * sin(p.theta);
    double left_sensor_x = p.x + LEFT_SENSOR_OFFSET * sin(p.theta);
    double left_sensor_y = p.y - LEFT_SENSOR_OFFSET * cos(p.theta);
    double right_sensor_x = p.x + RIGHT_SENSOR_OFFSET * sin(p.theta);
    double right_sensor_y = p.y - RIGHT_SENSOR_OFFSET * cos(p.theta);

    // Calculate distances to the walls
    expected_sensor_data[0] = fmin(12.0 - front_sensor_y, 12.0); // Front sensor
    expected_sensor_data[1] = fmin(12.0 - right_sensor_x, 12.0); // Right sensor
    expected_sensor_data[2] = fmin(back_sensor_y, 12.0); // Back sensor
    expected_sensor_data[3] = fmin(left_sensor_x, 12.0); // Left sensor

    // Add some noise to the expected readings to simulate random poles
    for (auto & data : expected_sensor_data) {
        data += random_noise(1.0);
    }

    return expected_sensor_data;
}

// Calculate likelihood based on sensor data
double calculate_likelihood(const Particle& p, const std::vector<double>& sensor_data) {
    double likelihood = 1.0;
    
    auto expected_sensor_data = calculate_expected_sensor_data(p);

    // Compare the expected readings with actual sensor readings
    for (size_t i = 0; i < sensor_data.size(); ++i) {
        likelihood *= exp(-pow(sensor_data[i] - expected_sensor_data[i], 2) / (2 * pow(1.0, 2)));
    }

    return likelihood;
}

// Normalize weights of particles
void normalize_weights(std::vector<Particle>& particles) {
    double sum_weights = 0.0;
    for (const auto& p : particles) {
        sum_weights += p.weight;
    }
    for (auto& p : particles) {
        p.weight /= sum_weights;
    }
}

// Resample particles
std::vector<Particle> resample_particles(const std::vector<Particle>& particles) {
    std::vector<Particle> new_particles;
    std::vector<double> cumulative_weights(particles.size());
    cumulative_weights[0] = particles[0].weight;
    for (size_t i = 1; i < particles.size(); ++i) {
        cumulative_weights[i] = cumulative_weights[i - 1] + particles[i].weight;
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    for (size_t i = 0; i < particles.size(); ++i) {
        double rand = dis(gen);
        auto it = std::lower_bound(cumulative_weights.begin(), cumulative_weights.end(), rand);
        new_particles.push_back(particles[it - cumulative_weights.begin()]);
    }

    return new_particles;
}

// Main MCL function
void monte_carlo_localization(int num_particles) {
    auto particles = initialize_particles(num_particles);

    while (true) {
        // Get odometry data (replace with actual odometry functions)
        double delta_x = 0.0, delta_y = 0.0, delta_theta = 0.0;
        // double delta_x = get_delta_x();
        // double delta_y = get_delta_y();
        // double delta_theta = get_delta_theta();

        // Update particles based on motion
        motion_update(particles, delta_x, delta_y, delta_theta);

        // Get sensor data
        std::vector<double> sensor_data(4);
        sensor_data[0] = frontSensor.get_value();
        sensor_data[1] = rightSensor.get_value();
        sensor_data[2] = backSensor.get_value();
        sensor_data[3] = leftSensor.get_value();

        // Get robot heading
        double heading = robot.getHeading() * (M_PI / 180.0); // Convert degrees to radians

        // Update particle weights based on sensor data
        for (auto& p : particles) {
            p.theta = heading; // Update particle orientation based on heading
            p.weight = calculate_likelihood(p, sensor_data);
        }
        normalize_weights(particles);

        // Resample particles
        particles = resample_particles(particles);

        // Estimate robot state (optional)
        // auto state = estimate_robot_state(particles);
        // display_robot_state(state);
    }
}
