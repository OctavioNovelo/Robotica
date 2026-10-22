// --------- Prototipos de las funciones definidas en el .ino ---------
void ff_moveForwardOneCell();
void ff_turnLeft90();
void ff_turnRight90();
void ff_turnAround180();
bool ff_hasWallFront();
bool ff_hasWallLeft();
bool ff_hasWallRight();
bool ff_frontClearAhead(); // TURN_MODE 2: confirma el frontal antes de seguir recto (true = libre)
void ff_hintNext(int straightAfter); // FloodFill avisa cuántas celdas rectas siguen tras la siguiente antes de girar o parar (0 = gira/para en ella)
// Tramos de pared CONOCIDOS (vistos con el IR o recorridos), definidas en main.cpp. Un tramo conocido y abierto no
// necesita la confirmación del frontal: el robot sigue de corrido.
void ff_markKnown(int r, int c, int nr, int nc);      // visto con el IR
void ff_markTraversed(int r, int c, int nr, int nc);  // recorrido: abierto seguro
bool ff_wallAllowed(int r, int c, int nr, int nc);    // false: tramo ya recorrido, la "pared" es un error de posición
bool ff_knownOpen(int r, int c, int nr, int nc);
bool ff_frontSure();       // la última lectura del frontal vale (TURN_MODE 2: la lectura temprana no alcanza la pared)

class Maus
{
public:
    // coords[row, col] en el MAZE
    int coords[2] = {0, 0};
    // 1 = norte, 2 = este, 3 = sur, 4 = oeste
    int direction = 1;

    // Moverse de la celda actual (coords) a (row,col)
    // [cambio] devuelve false si, al ir RECTO, la confirmación del frontal ve una pared: no se movió
    // (coords/dirección sin cambiar) y FloodFill vuelve a decidir con esa pared.
    bool move(int row, int col)
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
        if (diff == 0)
        {
            // [cambio] tramo ya conocido y abierto: sin confirmar (sigue de corrido)
            if (!ff_knownOpen(coords[0], coords[1], row, col) && !ff_frontClearAhead())
                return false;
        }
        else if (diff == 1 || diff == -3)
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
            // Voltearse completamente: un solo giro de 180°
            ff_turnAround180();
            // [cambio] Lo que había DETRÁS nunca se leyó (p. ej. al arrancar a mitad del laberinto). Ya girado,
            // confirmar el frontal; si hay pared: no se avanza, la dirección sí cambió y FloodFill vuelve a decidir.
            direction = targetDir;
            if (!ff_knownOpen(coords[0], coords[1], row, col) && !ff_frontClearAhead())
                return false;
        }

        // 4. Avanzar hacia la nueva celda
        GoForward();

        // 5. Actualizar posición y dirección ([cambio] el tramo recorrido queda conocido y abierto)
        ff_markTraversed(coords[0], coords[1], row, col);
        coords[0] = row;
        coords[1] = col;
        direction = targetDir;
        return true;
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