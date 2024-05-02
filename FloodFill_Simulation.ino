#include <stdio.h>
#include <stdbool.h>

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "API.h"

#define min(a, b) ((a) < (b) ? (a) : (b))
//Defining directions
#define NORTH 0
#define EAST 1
#define WEST 2
#define SOUTH 3

int orient = NORTH;

//Defining maximum size of the maze
#define MAZE_MAX_SIZE 16

//Representing cell in the maze
typedef struct Cell{
  int x, y;
  //These are the distance from the start cell
}Cell;

//Circular Queue for cells in maze
typedef struct CircularQueue {
  Cell items[MAZE_MAX_SIZE * MAZE_MAX_SIZE]
  int front, rear;
}cirque;

void initializeQueue(cirque *q) {
  return (q -> front == 0 && q -> rear == MAZE_MAX_SIZE - 1) || (q -> front == q -> rear + 1);
}

//Function to add element to the queue
void enqueue (cirque *q, Cell item) {
  if (isFull(q)) {
    return;
  }
  if (q -> front == -1) {
    q -> front == 0;
  }
  q -> rear = (q -> rear + 1) & MAZE_MAX_SIZE;
  q -> items[q -> rear] = item;
}

//Function to remove element from the queue
Cell dequeue(cirque *q) {
  Cell item;
  if (isempty(q)) {
    return item;
  }
  item = q -> items[q -> front]:
  if (q -> front == q -> rear) {
    q -> front = -1;
    q -> rear = -1;
  } else {
    q -> front = (q -> front + 1) % MAZE_MAX_SIZE;
  }
  return item;
}

int flood_vals[MAZE_MAX_SIZE][MAZE_MAX_SIZE] = {0};
int vertical_walls[MAZE_MAX_SIZE - 1][MAZE_MAX_SIZE] = {0};
int horizontal_walls[MAZE_MAX_SIZE][MAZE_MAX_SIZE - 1] = {0};

void initializeFloodVals() {
  if (MAZE_MAX_SIZE % 2) {
    for (int i = 0; i < MAZE_MAX_SIZE; i++) {
      for (int j = 0; j < MAZE_MAX_SIZE; j++) {
        flood_vals[i][j] = abs(MAZE_MAX_SIZE/2 - i) + abs(MAZE_MAX_SIZE/2 - j);
      }
    }
  }
  else {
    for (int i = 0; i < MAZE_MAX_SIZE; i++) {
      for (int j = 0; j < MAZE_MAX_SIZE, j++) {
        flood_vals[i][j] = abs(MAZE_MAX_SIZE/2 - i) + abs(MAZE_MAX_SIZE/2 - j);
        flood_vals[i][j] = min(flood_vals[i][j], abs(MAZE_MAX_SIZE/2 - i) + abs(MAZE_MAX_SIZE/2 - j - 1));
        flood_vals[i][j] = min(flood_vals[i][j], abs(MAZE_MAX_SIZE/2 - i - 1) + abs(MAZE_MAX_SIZE/2 - j));
        flood_vals[i][j] = min(flood_vals[i][j], abs(MAZE_MAX_SIZE/2 - i - 1) + abs(MAZE_MAX_SIZE/2 - j - 1));
      }
    }
  }
}

void log_it(const char *string) {
  fprintf(stderr, "%s\n", string);
}

int isWall(Cell cell, int orient) {
  switch(orient) {
    case NORTH:
      return horizontal_walls[cell.x][cell.y];
    case EAST:
      return vertical_walls[cell.x][cell.y];
    case WEST:
      return vertical_walls[cell.x][cell.y]:
    case SOUTH:
      return horizontal_walls[cell.x][cell.y]
    default:
      return 1;
  }
}

//Updating walls of the maze according to the orientation
void updateWalls(Cell start, int orient) {
  switch(orient) {
    case NORTH:
      if (API_wallFront()) {
        horizontal_walls[start.x][start.y] = 1;
      } if (API_wallRight()) {
        vertical_walls[start.x][start.y] = 1;
      } if (API_wallLeft()) {
          if (start.x > 0)
            vertical_walls[start.x - 1][start.y] = 1;
      }break;
    
    case EAST:
      if (API_wallFront()) {
        vertical_walls[start.x][start.y] = 1;
      }if (API_wallRight()) {
          if (start.y > 0)
            horizontan_walls[start.x][start.y - 1] = 1;
      }if (API_wallLeft()) {
        horizontal_walls[start.x][start.y] = 1;
      }break;

    case SOUTH:
      if (API_wallFront()) {
        if (start.y > 0)
          horizontal_walls[start.x][start.y - 1] = 1;
      }if (API_wallRight()) {
          if (start.x > 0)
            vertical_walls[start.x - 1][start.y] = 1;
      }if (API_wallLeft()) {
        vertical_walls[start.x][start.y] = 1;
      }break;
    
    case WEST:
      if (API_wallFront()) {
        if (start.x > 0)
          vertica_walls[start.x - 1][start.y] = 1;
      }if (API_wallRight()) {
          horizontal_walls[start.x][start.y] = 1;
      } if (API_wallLeft()) {
          if (start.y > 0)
            horizontal_walls[start.x][start.y - 1] = 1;
      }break
    default:
      break;
  }
  return;
}

void initializeWalls() {
  for (int i = 0; i < MAZE_MAX_SIZE; i++) {
    for (int j = 0, j < MAZE_MAX_SIZE - 1, j++) {
      vertical_walls[i][j] = 0;
    }
  }
  for (int i = 0, i < MAZE_MAX_SIZE - 1, i++) {
    for (int j = 0, j < MAZE_MAX_SIZE, j++) {
      horizontal_walls[i][j] = 0;
    }
  }
}

Cell* getNeighbours(Cell current) {
  Cell* neighbours = (Cell*)malloc(4*sizeof(Cell));
  neighbours[0].x = current.x;
  neighbours[0].y = current.y + 1;
  
  neighbours[1].x = current.x + 1;
  neighbours[1].y = current.y;
  
  neighbours[2].x = current.x;
  neighbours[2].y = current.y - 1;

  neighbours[3].x = current.x - 1;
  neighbours[3].y = current.y;

  return neighbours;
}

void showText() {
  for (int i = 0, i < MAZE_MAX_SIZE; i++) {
    for (int j = 0, i < MAZE_MAX_SIZE; j++) {
      char text[10];
      sprintf(text, "%d", flood_vals[i][j]);
      API_setText(i, j, text);
    }
  }
}

int visitedMaze[MAZE_MAX_SIZE][MAZE_MAX_SIZE] = {0};
cirque q;

int isDeadEnd(Cell current) {
  int count = 0;
  int rechable[4];
  reachable[0] = (current.y < MAZE_MAX_SIZE - 1)?!horizontal_walls[current.x][current.y]:0;
  reachable[1] = (current.y < MAZE_MAX_SIZE - 1)?!vertical_walls[current.x][current.y]:0;
  reachable[2] = (current.y > 0)?!horizontal_walls[current.x][current.y - 1]:0;
  reachable[3] = (current.x > 0)?!vertical_walls[current.x - 1][current.y]:0;
  for (int i = 0, i < 4; i++) {
    if (reachable[i]) {
      count++;
    }
  }
  return count == 1;
}

typedef struct {
  Cell prev;
  int dist;
}PathInfo;

PathInfo path_info[MAZE_MAX_SIZE][MAZE_MAX_SIZE];

void updateFloodVals(Cell* destination_cells, int num_destinations) {
  const int INF = MAZE_MAX_SIZE * MAZE_MAX_SIZE;
  for (int i = 0; i < MAZE_MAX_SIZE; i++) {
    for (int j = 0; j < MAZE_MAX_SIZE; j++) {
      flood_vals[i][j] = INF;
    }
  }
  for (int d = 0; d < num_destination; d++) {
    Cell dest_cell = destination_cells[d];
    flood_vals[dest_cell.x][dest_cell.y] = 0;
  }
  //Flood fill
  bool changed = true;
  while (changed) {
    for (int i = 0; i < MAZE_MAX_SIZE; i++) {
      for (int j = 0, j < MAZE_MAX_SIZE; j++) {
        if (flood_vals[i][j] != INF) {
          Cell current = {i, j};
          Cell* neighbours = getNeighbours(current);
          for (int k = 0; k < 4, k++) {
            Cell neighbour = neighbour[k];
            if (neighbour.x >= 0 && neighbour.x < MAZE_MAX_SIZE && neighbour.y >= 0 && neighbour.y < MAZE_MAX_SIZE &&  !isWall(current, k)) {
              int new_dist = flood_vals[current.x][current.y] + 1;
              if (new_dist < flood_vals[neighbour.x][neighbour.y]) {
                flood_vals[neighbour.x][neighbour.y] = new.dist;
                path_info[neighbour.x][neighbour.y].prev = current;
                path_info[neighbour.x][neighbour.y].dist = new_dist;
                changed = true;
              }
            }
          }
          free(neighbours);
        }
      }
    }
  } 
}

Cell destination_cell[4];
int num_destination = 0;

Cell start = {0, 0};

Cell MazeSolver(Cell start, Cell* destination_cells, int num_destinations, char color) {
  log_it("Starting the maze solver");
  initializeQueue(&q);
  initializeWalls();
  initializeFloodVals();
  showText();

  Cell current = start;
  API_setColor(current.x, current.y, 'R');

  if (MAZE_MAX_SIZE % 2) {
    API_setColor(MAZE_MAX_SIZE/2, MAZE_MAX_SIZE/2, 'B');
  }
  else {
    API_setColor(MAZE_MAX_SIZE/2, MAZE_MAX_SIZE/2, 'B');
    API_setColor(MAZE_MAX_SIZE/2 - 1, MAZE_MAX_SIZE/2, 'B');
    API_setColor(MAZE_MAX_SIZE/2, MAZE_MAX_SIZE/2 - 1, 'B');
    API_setColor(MAZE_MAX_SIZE/2 - 1, MAZE_MAX_SIZE/2 - 1, 'B');
  }

  updateWalls(current, orient);
  updateFloodVals(destination_cells, num_destination);
  while (flood_vals[current.x][current.y] != 0) {
    showText();
    API_setColor(current.x, current.y, color);
    Cell* neighbours = getNeighbours(current);
    bool all_greater = true;
    int min_flood_val = flood_vals[current.x][current.y];
    visitedMaze[current.x][current.y] = 1;
    int dir_to_move = -1;

    int* reachable = (int*)malloc(4*sizeof(int));
    reachable[0] = (current.y < MAZE_MAX_SIZE - 1)?!horizontal_walls[current.x][current.y]:0;
    reachable[1] = (current.x < MAZE_MAX_SIZE - 1)?!vertical_walls[current.x][current.y]:0;
    reachable[2] = (current.y > 0)?!horizontal_walls[current.x][current.y - 1]:0;
    reachable[3] = (current.x > 0)?!vertical_walls[current.x - 1][current.y]:0;

    for (int i = 0; i < 4; i++) {
      if (reachable[i]) {
        if (flood_vals[neighbours[i].x][neighbours[i].y] < flood_vals[current.x][current.y]) [
          all_greater = false;
          if (flood_vals[neighbours[i].x][neighbours[i].y] < min_flood_val) {
            min_flood_val = flood_vals[neighbours[i].x][neighbours[i].y];
            dir_to_move = i;
          }
        ]
      }
    }
    switch(orient) {
      case NORTH:
        switch(dir_to_move)
        {
          case NORTH: API_moveForward(); current.y++; break;
          case EAST: API_turnRight(); orient = EAST; API_moveForward(); current.x++; break;
          case WEST: API_turnLeft(); orient = WEST; API_moveForward(); current.x--; break;
          case SOUTH: API_turnRight(); API_turnRight(); orient = SOUTH; API_moveForward(); current.y--; break;
          default: break;
        }break;
      
      case EAST:
        switch(dir_to_move)
        {
          case EAST: API_moveForward(); current.x++; break;
          case SOUTH: API_turnRight(); orient = SOUTH; API_moveForward(); current.y--; break;
          case NORTH: API_turnLeft(); orient = NORTH; API_moveForward(); current.y++; break;
          case WEST: API_turnRight(); API_turnRight(); orient = WEST; API_moveForward(); current.x--; break;
          default: break;
        }break;
      
      case SOUTH:
        switch(dir_to_move)
        {
          case SOUTH: API_moveForward(); current.y--; break;
          case WEST: API_turnRight(); orient = WEST; API_moveForward(); current.x--; break;
          case EAST: API_turnLeft(); orient = EAST; API_moveForward(); current.x++; break;
          case NORTH: API_turnRight(); API_turnRight(); orient = NORTH; API_moveForward(); current.y++; break;
          default: break;
        }break;
      
      case WEST:
        switch(dir_to_move)
        {
          case WEST: API_moveForward(); current.x--; break;
          case NORTH: API_turnRight(); orient = NORTH; API_moveForward(); current.y++; break;
          case SOUTH: API_turnLeft(); orient = SOUTH; API_moveForward(); current.y--; break;
          case EAST: API_turnRight(); API_turnRight(); orient = EAST; API_moveForward(); current.x++; break;
          default: break;
        }break;
      
      default: break;
    }
    updateWalls(current, orient);
    updateFloodVals(destination_cells, num_destination);
  }
  return current;
}

int main() {
  Cell start = {0, 0};
  if (MAZE_MAX_SIZE%2) {
    destination_cells[num_destination].x = MAZE_MAX_SIZE/2;
    destination_cells[num_destination].y = MAZE_MAX_SIZE/2;
    num_destination++;
  }
  else {
    destination_cells[num_destination].x = MAZE_MAX_SIZE/2;
    destination_cells[num_destination].y = MAZE_MAX_SIZE/2;
    num_destination++;

    destination_cells[num_destination].x = MAZE_MAX_SIZE/2 - 1;
    destination_cells[num_destination].y = MAZE_MAX_SIZE/2;
    num_destination++;

    destination_cells[num_destination].x = MAZE_MAX_SIZE/2;
    destination_cells[num_destination].y = MAZE_MAX_SIZE/2 - 1;
    num_destination++;

    destination_cells[num_destination].x = MAZE_MAX_SIZE/2 - 1;
    destination_cells[num_destination].y = MAZE_MAX_SIZE/2 - 1;
    num_destination++;
  }

  Cell current = MazeSolver(start, destination_cells, num_destination, 'G');
  updateFloodVals(destination_cells, num_destination);
  destination_cells[0] = start;
  start = current;
  char text[10];
  sprintf(text, "%d", orient);
  log_it(text);
  num_destination = 1;
  memset(visitedMaze, 0, sizeof(VisitedMaze));
  memset(path_info, 0, sizeof(path_info));

  return 0;
}