
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

// 1. Define the Node structure to store Map Data
typedef struct {
    int id;
    float x;
    float y;
} Node;

// Heuristic function to calculate Euclidean distance between 2 points
float calculate_distance(float x1, float y1, float x2, float y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

int main() {
    // 2. Mandatory Route (Digital Mapping)
    Node strict_path[] = {
        {422, 10.0, 20.0},
        {423, 15.0, 25.0}, 
        {502, 20.0, 30.0},
        {503, 25.0, 35.0}
    };

    int total_nodes = sizeof(strict_path) / sizeof(strict_path[0]);
    int next_node_index = 1; // Initial target is the second Node in the array
    
    // Simulated initial vehicle coordinates
    float car_x = 10.0;
    float car_y = 20.0;
    
    // Acceptance Threshold (3.0 cm to trigger the next waypoint)
    float arrival_threshold = 3.0; 

    printf("--- BFMC Path Planning: Strict Path Follower Simulation ---\n");

    // 3. Planning Loop (Simulating vehicle movement)
    while (next_node_index < total_nodes) {
        Node target = strict_path[next_node_index];

        // Calculate current heuristic distance to the target node
        float dist = calculate_distance(car_x, car_y, target.x, target.y);

        printf("Car Position: (%.1f, %.1f) -> Target: Node %d (%.1f, %.1f) | Distance: %.2f cm\n", 
                car_x, car_y, target.id, target.x, target.y, dist);

        // CHECKPOINT DETECTION LOGIC (3cm Threshold)
        if (dist < arrival_threshold) {
            printf(">>> TARGET NODE %d REACHED! Switching to the next waypoint.\n\n", target.id);
            next_node_index++; // Increment index to select the next coordinate
            
            if (next_node_index >= total_nodes) {
                printf("The vehicle has reached the final destination.\n");
                break;
            }
        }

        // Simulating Movement (In reality, this is handled by Local Control/Sensors)
        // Simulate the car moving 1.0cm closer to the target per iteration
        car_x += 1.0; 
        car_y += 1.0; 
    }

    return 0;
}
