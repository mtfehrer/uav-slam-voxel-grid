#include <fstream>
#include <vector>
#include <sstream>
#include <thread>
#include <chrono>
#include <cmath>
#include <limits>
#include <iostream>
#include <algorithm>
#include <unistd.h>

using namespace std;

const int GRID_SIZE = 50;
const double VOXEL_SIZE = 0.1;

//in docker
string mapPointsFilename = "/all-points.txt";
string newestPoseFilename = "/newest-pose.txt";
string allPosesFilename = "/all-poses.txt";
string occupancyGridTempFilename = "/output/occupancy-grid-temp.txt";
string occupancyGridFilename = "/output/occupancy-grid.txt";

//without docker
// string mapPointsFilename = "/home/michael/Projects/edgeslam-path-planner/edgeslam/exported-data/all-points.txt";
// string newestPoseFilename = "/home/michael/Projects/edgeslam-path-planner/edgeslam/exported-data/newest-pose.txt";
// string allPosesFilename = "/home/michael/Projects/edgeslam-path-planner/edgeslam/exported-data/all-poses.txt";
// string occupancyGridTempFilename = "/home/michael/Projects/edgeslam-path-planner/planner/occupancy-grid-temp.txt";
// string occupancyGridFilename = "/home/michael/Projects/edgeslam-path-planner/planner/occupancy-grid.txt";

struct GridCoord {
    int i, j, k;
};

struct Vec3 {
    double x = 0.0f, y = 0.0f, z = 0.0f;
    Vec3 operator+(const Vec3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }
    Vec3 operator-(const Vec3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }
    Vec3 operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }
    Vec3 operator-() const {
        return {-x, -y, -z};
    }
    Vec3 crossProduct(const Vec3& other) {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        };
    }
    double dotProduct(const Vec3& other) {
        return x * other.x + y * other.y + z * other.z;
    }
};

struct Pyramid {
    Vec3 apex;
    Vec3 base[4];
};

struct Plane {
    Vec3 normal;
    Vec3 point;
};

const Vec3 gridOrigin = {-5.0, -5.0, -1.0};

vector<vector<double>> loadMapPoints() {
    vector<vector<double>> points;
    ifstream f(mapPointsFilename);

    string line;
    double x, y, z;

    while (getline(f, line)) {
        stringstream ss(line);
        if (ss >> x >> y >> z) {
            vector<double> point = {x, y, z};
            points.push_back(point);
        }
    }

    f.close();
    return points;
}

vector<vector<double>> loadNewestPose() {
    ifstream f(newestPoseFilename);
    vector<vector<double>> pose;
    string line;
    int row = 0;

    for (int i = 0; i < 4; i++) {
        vector<double> row;
        for (int j = 0; j < 4; j++) {
            row.push_back(0);
        }
        pose.push_back(row);
    }

    while (getline(f, line) && row < 4) {
        stringstream ss(line);
        double value;
        int col = 0;
        while (ss >> value && col < 4) {
            pose[row][col] = value;
            col++;
        }
        row++;
    }

    f.close();
    
    return pose;
}

vector<vector<vector<double>>> loadAllPoses() {
    vector<vector<vector<double>>> poses;
    ifstream f(allPosesFilename);
    string line;
    vector<vector<double>> pose;

    while (getline(f, line)) {
        if (!line.empty()) {
            vector<double> row;
            stringstream ss(line);
            double value;

            while (ss >> value) {
                row.push_back(value);
            }

            if (!row.empty()) {
                pose.push_back(row);
            }
        }
        else {
            if (pose.size() == 4) {
                poses.push_back(pose);
                pose.clear();
            }
        }
    }

    if (pose.size() == 4) {
        poses.push_back(pose);
    }

    f.close();

    return poses;
}

GridCoord worldToGrid(const Vec3& worldPoint) {
    Vec3 relativePos = worldPoint - gridOrigin;
    GridCoord gridCoord;
    gridCoord.i = static_cast<int>(relativePos.x / VOXEL_SIZE);
    gridCoord.j = static_cast<int>(relativePos.y / VOXEL_SIZE);
    gridCoord.k = static_cast<int>(relativePos.z / VOXEL_SIZE);
    return gridCoord;
}

Vec3 gridToWorld(GridCoord gridCoord) {
    Vec3 relativePos = {
        (gridCoord.i + 0.5) * VOXEL_SIZE, 
        (gridCoord.j + 0.5) * VOXEL_SIZE, 
        (gridCoord.k + 0.5) * VOXEL_SIZE
    };
    return gridOrigin + relativePos;
}

void resetOccupancyGrid(vector<vector<vector<char>>>& occupancyGrid) {
    occupancyGrid.clear();
    for (int i = 0; i < GRID_SIZE; i++) {
        vector<vector<char>> matrix;
        for (int j = 0; j < GRID_SIZE; j++) {
            vector<char> row;
            for (int k = 0; k < GRID_SIZE; k++) {
                row.push_back(0);
            }
            matrix.push_back(row);
        }
        occupancyGrid.push_back(matrix);
    }
}

void addOccupiedVoxelsToOccupancyGrid(vector<vector<vector<char>>>& occupancyGrid, vector<vector<double>>& mapPoints) {
    for (const auto& p : mapPoints) {
        Vec3 worldPoint = {p[0], p[1], p[2]};
        GridCoord coord = worldToGrid(worldPoint);

        if (coord.i >= 0 && coord.i < GRID_SIZE &&
            coord.j >= 0 && coord.j < GRID_SIZE &&
            coord.k >= 0 && coord.k < GRID_SIZE) {
            occupancyGrid[coord.i][coord.j][coord.k] = 2;
        }
    }
}

double getExtreme(vector<Vec3> vectors, char direction, bool findMax) {
    if (findMax) {
        double max = -std::numeric_limits<double>::infinity();
        if (direction == 'x') {
            for (int i = 0; i < vectors.size(); i++) {
                if (vectors[i].x > max) {
                    max = vectors[i].x;
                }
            }
        }
        else if (direction == 'y') {
            for (int i = 0; i < vectors.size(); i++) {
                if (vectors[i].y > max) {
                    max = vectors[i].y;
                }
            }
        }
        else if (direction == 'z') {
            for (int i = 0; i < vectors.size(); i++) {
                if (vectors[i].z > max) {
                    max = vectors[i].z;
                }
            }
        }
        return max;
    }
    else {
        double min = std::numeric_limits<double>::infinity();
        if (direction == 'x') {
            for (int i = 0; i < vectors.size(); i++) {
                if (vectors[i].x < min) {
                    min = vectors[i].x;
                }
            }
        }
        else if (direction == 'y') {
            for (int i = 0; i < vectors.size(); i++) {
                if (vectors[i].y < min) {
                    min = vectors[i].y;
                }
            }

        }
        else if (direction == 'z') {
            for (int i = 0; i < vectors.size(); i++) {
                if (vectors[i].z < min) {
                    min = vectors[i].z;
                }
            }

        }
        return min;
    }
}

bool isPointInsidePyramid(const Pyramid& pyramid, const Vec3& point) {
    std::vector<Plane> planes;

    // --- 1. Define the Base Plane ---
    // The plane is defined by the first three base vertices.
    Vec3 base_v1 = pyramid.base[1] - pyramid.base[0];
    Vec3 base_v2 = pyramid.base[2] - pyramid.base[0];
    Vec3 base_normal = base_v1.crossProduct(base_v2);

    // Orient the normal to point "inwards" (towards the apex).
    // We check the dot product of the vector from the base to the apex.
    // If it's negative, the normal is pointing outwards, so we flip it.
    if (base_normal.dotProduct(pyramid.apex - pyramid.base[0]) < 0) {
        base_normal.x *= -1;
        base_normal.y *= -1;
        base_normal.z *= -1;
    }
    planes.push_back({base_normal, pyramid.base[0]});

    // --- 2. Define the 4 Side Planes ---
    for (int i = 0; i < 4; ++i) {
        // Each side plane is a triangle formed by the apex and two adjacent base vertices.
        const Vec3& p1 = pyramid.apex;
        const Vec3& p2 = pyramid.base[i];
        const Vec3& p3 = pyramid.base[(i + 1) % 4]; // Wrap around for the last vertex

        Vec3 side_v1 = p2 - p1;
        Vec3 side_v2 = p3 - p1;
        Vec3 side_normal = side_v1.crossProduct(side_v2);

        // To orient the normal inwards, we use a point we know is on the "other side"
        // of the plane, relative to the pyramid's interior. The opposite base vertex works well.
        // For the face (apex, base[i], base[i+1]), the opposite vertex is base[i+2].
        const Vec3& opposite_vertex = pyramid.base[(i + 2) % 4];
        
        if (side_normal.dotProduct(opposite_vertex - p1) < 0) {
             side_normal.x *= -1;
             side_normal.y *= -1;
             side_normal.z *= -1;
        }
        planes.push_back({side_normal, p1});
    }

    // --- 3. Check the Point Against All Planes ---
    // For each plane, calculate the dot product of the vector from the plane's point
    // to the test point, with the plane's normal.
    // If this value is negative, the point is on the "outer" side of the plane,
    // and thus cannot be inside the pyramid.
    for (const auto& plane : planes) {
        Vec3 vector_to_point = point - plane.point;
        if (vector_to_point.dotProduct(plane.normal) < -1e-9) { // Use a small tolerance for floating point errors
            return false; // Point is outside
        }
    }

    // If the point is on the inner side of all planes, it's inside the pyramid.
    return true;
}

void addFreeVoxelsToOccupancyGrid(vector<vector<vector<char>>>& occupancyGrid, vector<vector<vector<double>>> allPoses) {
    double fovY = 0.75;
    double aspectRatio = 1.333;
    double farDist = 10;

    for (vector<vector<double>> p : allPoses) {
        Vec3 cameraPos = {p[0][3], p[1][3], p[2][3]};

        Vec3 right = {p[0][0], p[1][0], p[2][0]};
        Vec3 up    = {p[0][1], p[1][1], p[2][1]};
        Vec3 forward = -Vec3{p[0][2], p[1][2], p[2][2]};

        const float tanHalfFovY = tan(fovY / 2.0);
        const float farHeight = 2.0 * tanHalfFovY * farDist;
        const float farWidth = farHeight * aspectRatio;

        const Vec3 farCenter = cameraPos + (forward * farDist);
        const Vec3 halfUp = up * (farHeight / 2.0);
        const Vec3 halfRight = right * (farWidth / 2.0);

        Pyramid pyramid;
        pyramid.apex = cameraPos;
        pyramid.base[0] = farCenter + halfUp - halfRight; // Top-Left
        pyramid.base[1] = farCenter + halfUp + halfRight; // Top-Right
        pyramid.base[2] = farCenter - halfUp + halfRight; // Bottom-Right
        pyramid.base[3] = farCenter - halfUp - halfRight; // Bottom-Left

        vector<Vec3> vectors = {pyramid.base[0], pyramid.base[1], pyramid.base[2], pyramid.base[3], cameraPos};

        double minX_world = getExtreme(vectors, 'x', false);
        double minY_world = getExtreme(vectors, 'y', false);
        double minZ_world = getExtreme(vectors, 'z', false);
        double maxX_world = getExtreme(vectors, 'x', true);
        double maxY_world = getExtreme(vectors, 'y', true);
        double maxZ_world = getExtreme(vectors, 'z', true);

        GridCoord minGrid = worldToGrid({minX_world, minY_world, minZ_world});
        GridCoord maxGrid = worldToGrid({maxX_world, maxY_world, maxZ_world});

        int i_start = std::max(0, minGrid.i);
        int i_end   = std::min(GRID_SIZE, maxGrid.i + 1);
        int j_start = std::max(0, minGrid.j);
        int j_end   = std::min(GRID_SIZE, maxGrid.j + 1);
        int k_start = std::max(0, minGrid.k);
        int k_end   = std::min(GRID_SIZE, maxGrid.k + 1);

        for (int i = i_start; i < i_end; i++) {
            for (int j = j_start; j < j_end; j++) {
                for (int k = k_start; k < k_end; k++) {
                    if (occupancyGrid[i][j][k] == 0 && isPointInsidePyramid(pyramid, gridToWorld({i, j, k}))) {
                        occupancyGrid[i][j][k] = 1; // Mark as free
                    }
                }
            }
        }
    }
}

void exportOccupancyGrid(vector<vector<vector<char>>> occupancyGrid) {
    ofstream tempFile(occupancyGridTempFilename);
    if (!tempFile) {
        cerr << "Error: Could not open temporary file." << endl;
    }

    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            for (int k = 0; k < GRID_SIZE; k++) {
                tempFile << (int) occupancyGrid[i][j][k] << " ";
            }
            tempFile << "\n";
        }
        tempFile << "\n";
    }
    tempFile.close();

    if (rename(occupancyGridTempFilename.c_str(), occupancyGridFilename.c_str()) != 0) {
        perror("Error renaming file");
    }
}

int main() {
    vector<vector<vector<char>>> occupancyGrid;

    while (1) {
        vector<vector<double>> mapPoints = loadMapPoints();
        vector<vector<double>> newestPose = loadNewestPose();
        vector<vector<vector<double>>> allPoses = loadAllPoses();

        resetOccupancyGrid(occupancyGrid);
        addFreeVoxelsToOccupancyGrid(occupancyGrid, allPoses);
        addOccupiedVoxelsToOccupancyGrid(occupancyGrid, mapPoints);

        exportOccupancyGrid(occupancyGrid);

        sleep(1);
    }

    return 0;
}