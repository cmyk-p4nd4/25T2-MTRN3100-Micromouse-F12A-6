#pragma once

#include <math.h>
#include <iostream>
#include <cstdint>
#include <stack>
#include <algorithm>

enum Direction {
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3
};


namespace mtrn3100 {

class Grid {
public:
    Grid(int start_r, int start_c, Direction orientation, int goal_r, int goal_c) : start_r(start_r) , start_c(start_c), 
                                                                                orientation(orientation), goal_r(goal_r), goal_c(goal_c) {}
    
    void initialiseMaze() {
        for (int r = 0; r < ROWS; r++) {
            for (int c = 0; c < COLS; c++) {
                // Manhattan distance to goal
                costs[r][c] = std::abs(goal_r - r) + std::abs(goal_c - c);
                walls[r][c] = 0;
                visited[r][c] = false;
            }
        }
    }

                
    // Add wall in one direction and automatically add the opposite wall in the neighbor cell
    void addWall(int r, int c, Direction dir) {
        if (!isInside(r, c)) return;

        walls[r][c] |= (1 << dir);
        // add south and east wall

        // Update the neighboring cell
        int nr = r, nc = c;
        Direction opposite;

        switch (dir) {
            case NORTH: nr = r - 1; nc = c; opposite = SOUTH; break;
            case SOUTH: nr = r + 1; nc = c; opposite = NORTH; break;
            case EAST:  nr = r;     nc = c + 1; opposite = WEST;  break;
            case WEST:  nr = r;     nc = c - 1; opposite = EAST;  break;
        }

        if (isInside(nr, nc)) {
            walls[nr][nc] |= (1 << opposite);
        }

        runFloodFill(r, c);
    }

    bool hasWall(int r, int c, Direction dir) const {
        if (!isInside(r, c)) return true;
        return walls[r][c] & (1 << dir);
    }


    // Computes new costs when wall is added
    void runFloodFill(int curr_r, int curr_c) {
        std::stack<std::pair<int,int>> stack;
        stack.push({curr_r, curr_c});

        while (!stack.empty()) {
            auto [r, c] = stack.top();
            stack.pop();

            // 1. Get minimum cost of open neighbors
            int md = INT32_MAX;
            for (int dir = 0; dir < 4; dir++) {
                int nr = r, nc = c;
                switch (dir) {
                    case NORTH: nr--; break;
                    case SOUTH: nr++; break;
                    case EAST:  nc++; break;
                    case WEST:  nc--; break;
                }
                
                if (isInside(nr, nc) && !hasWall(r, c, (Direction)dir)) {
                    md = std::min(md, costs[nr][nc]);
                }
            }

            // 2. Update cost if needed
            int fr = targetReached ? start_r : goal_r;
            int fc = targetReached ? start_c : goal_c;
            if (costs[r][c] != md + 1 && !(r == fr && c == fc)) {
                costs[r][c] = md + 1;

                // 3. Push neighbors (except goal)
                for (int dir = 0; dir < 4; dir++) {
                    int nr = r, nc = c;
                    switch (dir) {
                        case NORTH: nr--; break;
                        case SOUTH: nr++; break;
                        case EAST:  nc++; break;
                        case WEST:  nc--; break;
                    }

                    if (isInside(nr, nc) && !hasWall(r, c, (Direction)dir)) {
                        stack.push({nr, nc});
                    }
                }
            }
        }
    }

    char nextStep (int curr_r, int curr_c, Direction curr_dir) {
        if (!targetReached && curr_r == goal_r && curr_c == goal_c) {
            targetReached = true;
            int start_cost = costs[start_r][start_c];
            for (int r = 0; r < ROWS; r++) {
                for (int c = 0; c < COLS; c++) {
                    costs[r][c] = (start_cost - costs[r][c]);
                }
            }
            for (int r = 0; r < ROWS; r++) {
                for (int c = 0; c < COLS; c++) {
                    for (int dir = 0; dir < 4; dir++) {
                        if (hasWall(r, c, (Direction)dir)) {
                            runFloodFill(r, c);
                        }
                    }
                }
            }
            costs[start_r][start_c] = 0;
        }
        if (targetReached && curr_r == start_r && curr_c == start_c) {
            returnedHome = true;
            int goal_cost = costs[goal_r][goal_c];
            for (int r = 0; r < ROWS; r++) {
                for (int c = 0; c < COLS; c++) {
                    costs[r][c] = goal_cost - costs[r][c];
                }
            }
            for (int r = 0; r < ROWS; r++) {
                for (int c = 0; c < COLS; c++) {
                    for (int dir = 0; dir < 4; dir++) {
                        if (hasWall(r, c, (Direction)dir)) {
                            runFloodFill(r, c);
                        }
                    }
                }
            }
            costs[goal_r][goal_c] = 0;
        }

        int next_r = 0;
        int next_c = 0;
        int md = 10000;
        Direction next_dir = curr_dir;
        
        for (int dir = 0; dir < 4; dir++) {
            int nr = curr_r, nc = curr_c;
            switch (dir) {
                case NORTH: nr--; break;
                case SOUTH: nr++; break;
                case EAST:  nc++; break;
                case WEST:  nc--; break;
            }
            if (!isInside(nr, nc)) {continue;}  
            if (hasWall(curr_r, curr_c, (Direction)dir)) {continue;} 
            if (costs[nr][nc] < md) {
                md = costs[nr][nc];
                next_r = nr;
                next_c = nc;
                next_dir = static_cast<Direction>(dir);
            }
        }

        int turn = (next_dir - curr_dir + 4) % 4;
        switch (turn) {
            case 0: return returnedHome ? 'x' : 'f'; // Forward
            case 1: return 'r'; // Right
            case 2: return 'u'; // U-turn
            case 3: return 'l'; // Left
        }

        return 'x';
    }

    bool returnedHome = false;

    String path(Direction curr_dir) {
        String result = "";
        int curr_r = start_r;
        int curr_c = start_c;
        while (curr_r != goal_r && curr_c != goal_c) {
            int md = INT32_MAX;
            Direction next_dir = curr_dir;
            
            for (int dir = 0; dir < 4; dir++) {
                int nr = curr_r, nc = curr_c;
                switch (dir) {
                    case NORTH: nr--; break;
                    case SOUTH: nr++; break;
                    case EAST:  nc++; break;
                    case WEST:  nc--; break;
                }
                if (isInside(nr, nc) && costs[nr][nc] < md && !hasWall(curr_r, curr_c, (Direction)dir)) {
                    md = costs[nr][nc];
                    next_dir = static_cast<Direction>(dir);
                }
            }

            int turn = (next_dir - curr_dir + 4) % 4;
            switch (turn) {
                case 0: result+= 'f'; // Forward
                case 1: result+=  'r'; // Right
                case 3: result+=  'l'; // Left
            }
        }
        return result;
    }

    std::vector<Cell> buildCellPath(Direction curr_dir) {
        std::vector<Cell> out;
        int r = start_r, c = start_c;
        out.push_back({r,c});

        const int MAX_STEPS = ROWS*COLS*4;  // safety
        int steps = 0;

        while (!(r == goal_r && c == goal_c) && steps++ < MAX_STEPS) {
            int best_cost = INT32_MAX;
            int best_dir  = -1;

            for (int dir = 0; dir < 4; ++dir) {
                int nr = r, nc = c;
                switch (dir) {
                    case NORTH: nr--; break;
                    case SOUTH: nr++; break;
                    case EAST:  nc++; break;
                    case WEST:  nc--; break;
                }
                if (!isInside(nr, nc)) continue;
                if (hasWall(r, c, (Direction)dir)) continue;

                int cost = costs[nr][nc];
                if (cost >= best_cost) continue;

                best_cost = cost;
                best_dir  = dir;
            }

            // tie-breaking (prefer straight; avoid back)
            if (best_dir >= 0) {
                int straight_dir = curr_dir;
                int back_dir     = (curr_dir + 2) & 3;

                for (int dir = 0; dir < 4; ++dir) {
                    if (dir == best_dir) continue;
                    int nr = r, nc = c;
                    switch (dir) {
                        case NORTH: nr--; break;
                        case SOUTH: nr++; break;
                        case EAST:  nc++; break;
                        case WEST:  nc--; break;
                    }
                    if (!isInside(nr, nc) || hasWall(r, c, (Direction)dir) || !visited[nr][nc]) continue;
                    if (costs[nr][nc] == best_cost) {
                        // prefer straight > right/left > back
                        if (dir == straight_dir) best_dir = dir;
                        else if (best_dir == back_dir && dir != back_dir) best_dir = dir;
                    }
                }
            }

            if (best_dir < 0) {
                // dead end; bail to avoid infinite loop
                break;
            }

            // advance
            switch (best_dir) {
                case NORTH: r--; break;
                case SOUTH: r++; break;
                case EAST:  c++; break;
                case WEST:  c--; break;
            }
            curr_dir = (Direction)best_dir;
            out.push_back({r,c});
        }
        return out;
    }

    void applyMove(int curr_r, int curr_c, Direction curr_dir, char curr_move,
               int &new_r, int &new_c, Direction &new_dir) {
    
        // Start with current state
        new_r = curr_r;
        new_c = curr_c;
        new_dir = curr_dir;

        // Update direction first if turning
        if (curr_move == 'r') {
            new_dir = static_cast<Direction>((curr_dir + 1) % 4);
        }
        else if (curr_move == 'l') {
            new_dir = static_cast<Direction>((curr_dir + 3) % 4);
        } else if (curr_move == 'u') {
            new_dir = static_cast<Direction>((curr_dir + 2) % 4);
        }

        // If moving forward, step in the current (possibly updated) direction
        if (curr_move == 'f') {
            switch (curr_dir) {
                case NORTH: new_r--; break;
                case SOUTH: new_r++; break;
                case EAST:  new_c++; break;
                case WEST:  new_c--; break;
            }
        }

        visited[new_r][new_c] = true;
    }

    void printCosts() {
        Serial.println("----------");
        for (int r = 0; r < ROWS; r++) {
            Serial.print("| ");
            for (int c = 0; c < COLS; c++) {
                Serial.print(costs[r][c]);
                Serial.print(" ");
            }
            Serial.println(" |");
        }
        Serial.println("----------");
    }

    float getPercentage () {
        int sum = 0;
        for (int r = 0; r < ROWS; r++) {
            for (int c = 0; c < COLS; c++) {
                if (visited[r][c]) sum++;
            }
        }
        return float (float(sum) / (ROWS * COLS)) * 100.0f;
    }


private:
    int start_r; 
    int start_c; 
    Direction orientation; 
    int goal_r; 
    int goal_c;
    static const int ROWS = 9;
    static const int COLS = 9;
    int costs[ROWS][COLS];   // 2D array
    uint8_t walls[ROWS][COLS];
    bool visited [ROWS][COLS];
    bool isInside(int r, int c) const {
        return (r >= 0 && r < ROWS && c >= 0 && c < COLS);
    }
    bool targetReached = false;
};

}  // namespace mtrn3100
