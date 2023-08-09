#include <bits/stdc++.h>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <math.h>
#include <set>
#include <stack>
#include <stdio.h>
using namespace std;
typedef Eigen::MatrixXf mat;
// typedef pair<int, int> Pair;
class astar {

  struct cell {
    int parent_i, parent_j;
    double f, g, h;
  };
  struct pair {
    int r;
    int c;
  };
  struct ppair {
    int f;
    pair p;
  };

private:
  mat map;
  int ROW;
  int COL;
  double start_h;

public:
  astar(mat grid, int r, int col, double h) {
    map = grid;
    ROW = r;
    COL = col;
    start_h = h;
  }

  bool is_valid(int row, int col) {
    return (row >= 0) && (row < ROW) && (col >= 0) && (col < COL);
  }

  bool isunblocked(int row, int col) {
    if (map(row, col) == 100) {
      return false;
    }
    return true;
  }
  double Hcost(pair dest, int row, int col) {
    return ((double)sqrt((row - dest.r) * (row - dest.r) +
                         (col - dest.c) * (col - dest.c)));
  }

  bool isDestination(pair dest, int row, int col) {
    return (row == dest.r && col == dest.c);
  }
  void tracePath(cell cellDetails[][COL], pair dest) {
    printf("\nThe Path is ");
    int row = dest.r;
    int col = dest.c;

    stack<pair> Path;
    pair i;
    while (!(cellDetails[row][col].parent_i == row &&
             cellDetails[row][col].parent_j == col)) {
      i.r = row;
      i.c = col;
      Path.push(i);
      int temp_row = cellDetails[row][col].parent_i;
      int temp_col = cellDetails[row][col].parent_j;
      row = temp_row;
      col = temp_col;
    }
    i.r = row;
    i.c = col;
    Path.push(i);
    while (!Path.empty()) {
      pair p = Path.top();
      Path.pop();
      printf("-> (%d,%d) ", p.r, p.c);
    }

    return;
  }
  void plan(pair src, pair dist) {
    if (!is_valid(src.r, src.c)) {
      std::cout << "src is invalid";
    }
    if (!is_valid(dist.r, src.c)) {
      std::cout << "src is invalid";
    }

    if (!isunblocked(dist.r, dist.c)) {
      cout << "is blocked";
    }
    if (!isunblocked(src.r, src.c)) {
      cout << "is blocked";
    }

    bool closedlist[ROW][COL];
    memset(closedlist, false, sizeof(closedlist));

    cell cellDetails[ROW][COL];
    int i, j, pi, pj;
    for (i = 0; i < ROW; i++) {
      for (j = 0; j < COL; j++) {
        cellDetails[i][j].f = FLT_MAX;
        cellDetails[i][j].g = map(i, j);
        cellDetails[i][j].h = FLT_MAX;
        cellDetails[i][j].parent_i = -1;
        cellDetails[i][j].parent_j = -1;
      }
    }
    i = src.r, j = src.c;
    cellDetails[i][j].f = 0.0;
    cellDetails[i][j].g = 0.0;
    cellDetails[i][j].h = 0.0;
    cellDetails[i][j].parent_i = i;
    cellDetails[i][j].parent_j = j;

    set<ppair> openlist;
    pair p;
    p.c = i;
    p.r = j;
    ppair k;
    k.f = 0.0;
    k.p = p;
    openlist.insert(k);

    bool foundDest = false;
    while (!openlist.empty()) {
      ppair p = *openlist.begin;
      openlist.erase(openlist.begin());
      pi = p.p.c;
      pj = p.p.r;
      closedlist[i][j] = true;
      i = pi;
      j = pj;
      double gnew, fnew, hnew;
      int suc[8][2] = {{i + 1, j},     {i + 1, j},     {i, j + 1},
                       {i, j - 1},     {i - 1, j + 1}, {i - 1, j - 1},
                       {i + 1, j + 1}, {i + 1, j - 1}};
      pair pt;
      ppair pr;
      for (int it = 0; it < 8; it++) {
        i = suc[it][0];
        j = suc[it][1];
        pt.r = i;
        pt.c = j;

        if (is_valid(i, j)) {

          if (isDestination(dist, i, j)) {
            cellDetails[i][j].parent_i = pi;
            cellDetails[i][j].parent_j = pj;
            cout << "Destination Found\n";
            tracePath(cellDetails, dist);
            foundDest = true;
          }
          if (closedlist[i][j] == false && isunblocked(i, j)) {
            gnew = cellDetails[i][j].g + 1.0;
            hnew = Hcost(dist, i, j);
            fnew = gnew + hnew;
            if (cellDetails[i][j].f == FLT_MAX || cellDetails[i][j].f > fnew) {
              pr.p = pt;
              pr.f = fnew;
              openlist.insert(pr);
              cellDetails[i][j].f = fnew;
              cellDetails[i][j].g = gnew;
              cellDetails[i][j].h = hnew;
              cellDetails[i][j].parent_i = pi;
              cellDetails[i][j].parent_j = pj;
            }
          }
        }
      }
    }
  }
};
