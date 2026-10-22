void saveWallsToFlash();  // definida en UMouse.ino
class Floodfill
{
private:
    int cells[MAZE_ROWS][MAZE_COLS];
    bool (*walls)[WALL_COLS]; // puntero a walls[WALL_ROWS][WALL_COLS]
    int cellGoal[2];
    Maus *maus;

    bool secondFlood = false;

public:
    // Constructor para primer floodfill
    Floodfill(bool (*wallsIn)[WALL_COLS], const int goal[2], Maus *mausPtr, bool isSecond = false)
    {
        walls = wallsIn; // apuntamos a la matriz compartida
        maus = mausPtr;
        secondFlood = isSecond;

        cellGoal[0] = goal[0];
        cellGoal[1] = goal[1];

        // Inicializamos cells a -1 (null)
        for (int r = 0; r < MAZE_ROWS; r++)
            for (int c = 0; c < MAZE_COLS; c++)
                cells[r][c] = -1;
    }

    void solve()
    {
        flood();

        int initialMausCoords[2] = {maus->coords[0], maus->coords[1]};

        while (cells[maus->coords[0]][maus->coords[1]] != 0)
        {
            int neighbors[4][2];
            int neighborCount = getNeighbors(maus->coords[0], maus->coords[1], neighbors);

            scanWalls(maus->coords[0], maus->coords[1]);

            bool deadEnd = true;

            for (int i = 0; i < neighborCount; i++)
            {
                int nr = neighbors[i][0];
                int nc = neighbors[i][1];

                if (cells[nr][nc] >= cells[maus->coords[0]][maus->coords[1]])
                    continue;
                if (blockedByWall(maus->coords[0], maus->coords[1], nr, nc))
                    continue;

                maus->move(nr, nc);
                deadEnd = false;
                break;
            }

            if (deadEnd)
            {
                flush();
                flood();
            }
        }

        saveWallsToFlash();

        if (secondFlood)
            return;

        delay(5000);
        {
            // Usamos la MISMA matriz walls compartida
            Floodfill flood2(walls, initialMausCoords, maus, true);
            flood2.solve();
        }

        secondFlood = true;
        delay(5000);
        solve();
    }

private:
    void scanWalls(int mausRow, int mausCol)
    {
        if (maus->FrontWallDetected())
        {
            switch (maus->direction)
            {
            case 1: // norte
            {
                int newRow = mausRow - 1;
                if (cellWithinBounds(newRow, mausCol))
                    SetWall(mausRow, mausCol, newRow, mausCol);
                break;
            }
            case 3: // sur
            {
                int newRow = mausRow + 1;
                if (cellWithinBounds(newRow, mausCol))
                    SetWall(mausRow, mausCol, newRow, mausCol);
                break;
            }
            case 2: // este
            {
                int newCol = mausCol + 1;
                if (cellWithinBounds(mausRow, newCol))
                    SetWall(mausRow, mausCol, mausRow, newCol);
                break;
            }
            case 4: // oeste
            {
                int newCol = mausCol - 1;
                if (cellWithinBounds(mausRow, newCol))
                    SetWall(mausRow, mausCol, mausRow, newCol);
                break;
            }
            }
        }

        if (maus->LeftWallDetected())
        {
            switch (maus->direction)
            {
            case 2: // este -> izquierda = norte
            {
                int newRow = mausRow - 1;
                if (cellWithinBounds(newRow, mausCol))
                    SetWall(mausRow, mausCol, newRow, mausCol);
                break;
            }
            case 4: // oeste -> izquierda = sur
            {
                int newRow = mausRow + 1;
                if (cellWithinBounds(newRow, mausCol))
                    SetWall(mausRow, mausCol, newRow, mausCol);
                break;
            }
            case 3: // sur -> izquierda = este
            {
                int newCol = mausCol + 1;
                if (cellWithinBounds(mausRow, newCol))
                    SetWall(mausRow, mausCol, mausRow, newCol);
                break;
            }
            case 1: // norte -> izquierda = oeste
            {
                int newCol = mausCol - 1;
                if (cellWithinBounds(mausRow, newCol))
                    SetWall(mausRow, mausCol, mausRow, newCol);
                break;
            }
            }
        }

        if (maus->RightWallDetected())
        {
            switch (maus->direction)
            {
            case 4: // oeste -> derecha = norte
            {
                int newRow = mausRow - 1;
                if (cellWithinBounds(newRow, mausCol))
                    SetWall(mausRow, mausCol, newRow, mausCol);
                break;
            }
            case 2: // este -> derecha = sur
            {
                int newRow = mausRow + 1;
                if (cellWithinBounds(newRow, mausCol))
                    SetWall(mausRow, mausCol, newRow, mausCol);
                break;
            }
            case 1: // norte -> derecha = este
            {
                int newCol = mausCol + 1;
                if (cellWithinBounds(mausRow, newCol))
                    SetWall(mausRow, mausCol, mausRow, newCol);
                break;
            }
            case 3: // sur -> derecha = oeste
            {
                int newCol = mausCol - 1;
                if (cellWithinBounds(mausRow, newCol))
                    SetWall(mausRow, mausCol, mausRow, newCol);
                break;
            }
            }
        }
    }

    void SetWall(int row, int col, int newRow, int newCol)
    {
        int wallRow = row * 2 + (newRow - row);
        int wallCol = col * 2 + (newCol - col);
        walls[wallRow][wallCol] = true;
    }

    void flush()
    {
        for (int r = 0; r < MAZE_ROWS; r++)
            for (int c = 0; c < MAZE_COLS; c++)
                cells[r][c] = -1;
    }

    void flood()
    {
        struct Coord
        {
            int r;
            int c;
        };

        Coord queue[MAZE_ROWS * MAZE_COLS];
        int head = 0;
        int tail = 0;

        queue[tail++] = {cellGoal[0], cellGoal[1]};
        cells[cellGoal[0]][cellGoal[1]] = 0;

        while (head != tail)
        {
            Coord cur = queue[head++];
            int r0 = cur.r;
            int c0 = cur.c;

            int neighbors[4][2];
            int neighborCount = getNeighbors(r0, c0, neighbors);

            for (int i = 0; i < neighborCount; i++)
            {
                int nr = neighbors[i][0];
                int nc = neighbors[i][1];

                if (blockedByWall(r0, c0, nr, nc))
                    continue;
                if (cells[nr][nc] != -1)
                    continue;

                cells[nr][nc] = cells[r0][c0] + 1;
                queue[tail++] = {nr, nc};
            }
        }
    }

    bool blockedByWall(int srcRow, int srcCol, int dstRow, int dstCol)
    {
        int row = srcRow * 2 + (dstRow - srcRow);
        int col = srcCol * 2 + (dstCol - srcCol);
        return walls[row][col];
    }

    bool cellWithinBounds(int row, int col)
    {
        if (row < 0 || row >= MAZE_ROWS)
            return false;
        if (col < 0 || col >= MAZE_COLS)
            return false;
        return true;
    }

    int getNeighbors(int row, int col, int outNeighbors[4][2])
    {
        int count = 0;

        int candidates[4][2] = {
            {row - 1, col},
            {row, col + 1},
            {row + 1, col},
            {row, col - 1}};

        for (int i = 0; i < 4; i++)
        {
            int nr = candidates[i][0];
            int nc = candidates[i][1];
            if (cellWithinBounds(nr, nc))
            {
                outNeighbors[count][0] = nr;
                outNeighbors[count][1] = nc;
                count++;
            }
        }
        return count;
    }
};