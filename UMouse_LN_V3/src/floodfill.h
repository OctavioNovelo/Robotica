void saveWallsToFlash();  // definida en UMouse.ino
void ff_goalUnreachable(); // definida en main.cpp: detiene con diagnóstico (no vuelve)
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
        // [cambio mínimo] flush() antes de flood(): en la 3.ª fase (secondFlood) cells conservaba
        // las distancias de la ida inicial y el BFS (que solo rellena celdas en -1) no las
        // actualizaba con las paredes descubiertas al volver.
        flush();
        flood();

        int initialMausCoords[2] = {maus->coords[0], maus->coords[1]};

        while (cells[maus->coords[0]][maus->coords[1]] != 0)
        {
            int neighbors[4][2];
            int neighborCount = getNeighbors(maus->coords[0], maus->coords[1], neighbors);
            // [cambio] empate de distancia: primero RECTO (antes el orden fijo N,E,S,O metía al robot en
            // giros y medias vueltas innecesarias, p. ej. callejón aparente -> 180° hacia el norte con el sur libre).
            preferStraight(maus->coords[0], maus->coords[1], dirDRow(maus->direction), dirDCol(maus->direction), neighbors, neighborCount);

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

                // [cambio] false = pared al frente confirmada al ir recto: la próxima vuelta del bucle la
                // registra (scanWalls reutiliza la lectura, ya con F=1) y elige otro vecino o recalcula.
                ff_hintNext(predictTurnOrStop(maus->coords[0], maus->coords[1], nr, nc));
                maus->move(nr, nc);
                deadEnd = false;
                break;
            }

            if (deadEnd)
            {
                flush();
                flood();
                // [cambio mínimo] meta inalcanzable con las paredes conocidas (p. ej. una pared
                // falsa): antes era un bucle infinito sin diagnóstico.
                if (cells[maus->coords[0]][maus->coords[1]] == -1)
                    ff_goalUnreachable();
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

    // [cambio] Con las paredes conocidas, ¿en (nr,nc) girará o parará? (mismo criterio que solve: primer vecino más
    // cercano a la meta sin pared). Con mapa completo es exacto; explorando es solo una estimación.
    bool predictTurnOrStop(int r, int c, int nr, int nc)
    {
        if (cells[nr][nc] <= 0)
            return true; // meta: para
        int neighbors[4][2];
        int count = getNeighbors(nr, nc, neighbors);
        preferStraight(nr, nc, nr - r, nc - c, neighbors, count);
        for (int i = 0; i < count; i++)
        {
            int pr = neighbors[i][0];
            int pc = neighbors[i][1];
            if (cells[pr][pc] >= cells[nr][nc] || cells[pr][pc] < 0)
                continue;
            if (blockedByWall(nr, nc, pr, pc))
                continue;
            return (pr - nr) != (nr - r) || (pc - nc) != (nc - c);
        }
        return true; // callejón: media vuelta
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

    static int dirDRow(int dir) { return dir == 1 ? -1 : (dir == 3 ? 1 : 0); }
    static int dirDCol(int dir) { return dir == 2 ? 1 : (dir == 4 ? -1 : 0); }

    // Pone primero el vecino que queda recto (fila+dRow, col+dCol); el resto conserva el orden N,E,S,O.
    static void preferStraight(int row, int col, int dRow, int dCol, int nb[4][2], int count)
    {
        for (int i = 1; i < count; i++)
        {
            if (nb[i][0] != row + dRow || nb[i][1] != col + dCol)
                continue;
            int sr = nb[i][0], sc = nb[i][1];
            for (int j = i; j > 0; j--)
            {
                nb[j][0] = nb[j - 1][0];
                nb[j][1] = nb[j - 1][1];
            }
            nb[0][0] = sr;
            nb[0][1] = sc;
            return;
        }
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