// --------- Prototipos de las funciones definidas en el .ino ---------
void ff_moveForwardOneCell();
void ff_turnLeft90();
void ff_turnRight90();
bool ff_hasWallFront();
bool ff_hasWallLeft();
bool ff_hasWallRight();

class Maus
{
public:
    // coords[row, col] en el MAZE
    int coords[2] = {0, 0};
    // 1 = norte, 2 = este, 3 = sur, 4 = oeste
    int direction = 1;

    // Moverse de la celda actual (coords) a (row,col)
    void move(int row, int col)
    {
        int rowMove = row - coords[0];
        int colMove = col - coords[1];

        // 1. Determinar la dirección que se necesita tomar
        int targetDir = direction;

        if (rowMove == -1)
            targetDir = 1; // Norte
        else if (colMove == 1)
            targetDir = 2; // Este
        else if (rowMove == 1)
            targetDir = 3; // Sur
        else if (colMove == -1)
            targetDir = 4; // Oeste

        // 2. Calcular cuántos giros se requieren
        int diff = targetDir - direction;

        // Normalización a -3..3
        if (diff > 2)
            diff -= 4;
        if (diff < -2)
            diff += 4;

        // 3. Ejecutar el giro adecuado
        if (diff == 1 || diff == -3)
        {
            // Girar a la derecha una vez
            TurnRight();
        }
        else if (diff == -1 || diff == 3)
        {
            // Girar a la izquierda una vez
            TurnLeft();
        }
        else if (diff == 2 || diff == -2)
        {
            // Voltearse completamente
            TurnRight();
            TurnRight();
        }

        // 4. Avanzar hacia la nueva celda
        GoForward();

        // 5. Actualizar posición y dirección
        coords[0] = row;
        coords[1] = col;
        direction = targetDir;
    }

    // ---------------- Sensado de paredes (usa ff_* del .ino) ----------------
    bool FrontWallDetected()
    {
        return ff_hasWallFront();
    }

    bool LeftWallDetected()
    {
        return ff_hasWallLeft();
    }

    bool RightWallDetected()
    {
        return ff_hasWallRight();
    }

    // ---------------- Movimientos básicos (usa ff_* del .ino) ----------------
    void TurnRight()
    {
        ff_turnRight90();
    }

    void TurnLeft()
    {
        ff_turnLeft90();
    }

    void GoForward()
    {
        ff_moveForwardOneCell();
    }
};