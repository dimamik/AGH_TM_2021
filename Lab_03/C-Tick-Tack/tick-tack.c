#include <stdio.h>

#define PLAYER1 1  // circle
#define PLAYER2 -1 // cross

#define PLAYER1_WIN 1
#define PLAYER2_WIN -1
#define DRAW 2

int whichMove = PLAYER1;
int gameBoard[3][3];
// Indicates whether player can make a move
int make_move = 1;

void printBoard(void)
{
    printf("------\n");
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            printf("%d ", gameBoard[i][j]);
        }
        printf("\n");
    }
    printf("------\n");
}

void initBoard()
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            gameBoard[i][j] = 0;
        }
    }
}

int checkIfEnded()
{
    // check tick tack game for end
    // Return 1 if player 1 wins and -1 if player 2 wins
    // Return 2 if draw
    // Return 0 if game is not ended

    // Check rows and columns
    for (int i = 0; i < 3; i++)
    {
        int rowSum = 0;
        int colSum = 0;
        for (int j = 0; j < 3; j++)
        {
            rowSum += gameBoard[i][j];
            colSum += gameBoard[j][i];
        }
        if (rowSum == 3 || colSum == 3)
        {
            return PLAYER1_WIN;
        }
        else if (rowSum == -3 || colSum == -3)
        {
            return PLAYER2_WIN;
        }
    }
    // Check diagonals
    int diagSum = 0;
    int antiDiagSum = 0;
    for (int i = 0; i < 3; i++)
    {
        diagSum += gameBoard[i][i];
        antiDiagSum += gameBoard[i][2 - i];
    }
    if (diagSum == 3 || antiDiagSum == 3)
    {
        return PLAYER1_WIN;
    }
    else if (diagSum == -3 || antiDiagSum == -3)
    {
        return PLAYER2_WIN;
    }
    // Check draw
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (gameBoard[i][j] == 0)
            {
                return 0;
            }
        }
    }
    return DRAW;
}

int checkThrio(int first, int second, int third)
{
    // Check if trio is valid
    // Return 1 if valid
    // Return 0 if invalid
    if (first == second && first == third && first != 0)
    {
        return 1;
    }
    return 0;
}

int checkForEndWithDrawingOnVictory()
{ // and if end draw the line marking the win
    printf("%d %d %d", gameBoard[0][0], gameBoard[1][1], gameBoard[2][2]);
    for (int i = 0; i < 3; i++)
    {
        if (checkThrio(gameBoard[i][0], gameBoard[i][1], gameBoard[i][2]) == 1)

        {
            // BSP_LCD_DrawHLine(0, 272 / (2 * (3 - i)), 480); // line through the middle f.ex.: i = 0 (first row) => 272/2*3 = 45
            return gameBoard[i][0]; // it will be 1 or -1 (PLAYER1 or PLAYER2)
        }

        if (checkThrio(gameBoard[0][i], gameBoard[1][i], gameBoard[2][i]) == 1)
        {

            // BSP_LCD_DrawVLine(480 / (2 * (3 - i)), 0, 272);
            return gameBoard[0][i];
        }
    }
    // if ((gameBoard[0][0] == gameBoard[1][1] == gameBoard[2][2]) && gameBoard[2][2] != 0)
    if (checkThrio(gameBoard[0][0], gameBoard[1][1], gameBoard[2][2]) == 1)
    {
        // Point lines[2];
        // lines[0].X = 0;
        // lines[0].Y = 272;

        // lines[1].X = 480;
        // lines[1].Y = 0;

        // BSP_LCD_FillPolygon(lines, 2);
        return gameBoard[1][1];
    }

    if (checkThrio(gameBoard[0][2], gameBoard[1][1], gameBoard[2][0]) == 1)
    {
        // Point lines[2];
        // lines[0].X = 480;
        // lines[0].Y = 272;

        // lines[1].X = 0;
        // lines[1].Y = 0;

        // BSP_LCD_FillPolygon(lines, 2);
        return gameBoard[1][1];
    }

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (gameBoard[i][j] == 0)
                return 0; // if there is no winner and in game board is free place
        }
    }

    return 2; // if there is no free place in game board and nobody won - it means draw
}

int isValidMove(int row, int col)
{
    if (row < 0 || row > 2 || col < 0 || col > 2)
    {
        return 0;
    }

    if (gameBoard[row][col] == 0)
    {
        return 1;
    }
    return 0;
}

void drawCross(int cursorX, int cursorY)
{
    // TODO: Draw cross
    printf("Drawing cross\n");

    /*    uint16_t size = 70;

    BSP_LCD_SelectLayer(LCD_LAYER_FG);
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    Point lines[2];
    lines[0].X = cursorX - size/2;
    lines[0].Y = cursorY + size/2;

    lines[1].X = cursorX + size/2;
    lines[1].Y = cursorY - size/2;

    BSP_LCD_FillPolygon(lines, 2);

    lines[0].X = cursorX - size/2;
    lines[0].Y = cursorY - size/2;

    lines[1].X = cursorX + size/2;
    lines[1].Y = cursorY + size/2;

    BSP_LCD_FillPolygon(lines, 2);*/

    // cursorPosition[0] = cursorX;
    // cursorPosition[1] = cursorY;

    // return cursorPosition;
}

void drawCircle(int cursorX, int cursorY)
{

    // TODO Draw Circle
    printf("Drawing circle\n");

    // to delete !!
    // int cursorX = 480;
    // int cursorY = 272;

    /*    BSP_LCD_SelectLayer(LCD_LAYER_FG);
    BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
    BSP_LCD_FillCircle( cursorX, cursorY, 36 );*/

    // cursorPosition[0] = cursorX;
    // cursorPosition[1] = cursorY;

    // return cursorPosition;
}

int *convertCursorToRowColumn(int cursorX, int cursorY)
{
    int col = cursorX % (480 / 3);
    int row = cursorY % (272 / 3);

    static int rowColumn[2];
    rowColumn[0] = row;
    rowColumn[1] = col;

    return rowColumn;
}

void drawMove(int player, int row, int col)
{
    // Convert row and col to center coordinates (x, y) in our display and draw object
    int cursorX = 150;
    int cursorY = 150;

    if (player == PLAYER1)
    {
        drawCircle(cursorX, cursorY);
    }
    else
    {
        drawCross(cursorX, cursorY);
    }
}
// Get the next move
int *awaitGetMove(int player)
{
    // TODO There we are awaiting the next move from UI
    // Getting posX and posY of Cursor and converting it to row/col values
    // Make some sleeps or just combine -> Watever

    // There use convertCursorToRowColumn()

    int row, col;
    printf("Enter row and column: ");
    scanf("%d %d", &row, &col);
    static int rc[2];
    rc[0] = row;
    rc[1] = col;
    return rc;
}

void makeMove(int player)
{
    int is_valid = 0;
    while (is_valid == 0)
    {
        int *rc = awaitGetMove(player);
        int row = rc[0];
        int col = rc[1];
        if (isValidMove(row, col))
        {
            is_valid = 1;
            gameBoard[row][col] = player;
            // Draw move
            drawMove(player, row, col);
            return;
        }
        printf("Invalid move\n");
    }
}

void cleanBoard(void)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            gameBoard[i][j] = 0;
        }
    }

    // TODO RESET Background/Foreground somehow
}

void printTextOnLCD(char *text)
{
    // TODO Print text on LCD
    printf("Printing text: %s\n", text);
}

void playGame()
{
    initBoard();
    while (checkForEndWithDrawingOnVictory() == 0)
    {
        if (whichMove == PLAYER1)
        {
            printf("Player 1's turn\n");
            makeMove(PLAYER1);
        }
        else
        {
            printf("Player 2's turn\n");
            makeMove(PLAYER2);
        }
        whichMove = -whichMove;
        printBoard();
    }

    int result = checkForEndWithDrawingOnVictory();
    if (result == PLAYER1)
    {
        printf("Player 1 won\n");
    }
    else if (result == PLAYER2)
    {
        printf("Player 2 won\n");
    }
    else
    {
        cleanBoard();
        printf("Draw\n");
        printTextOnLCD("Draw");
    }
}

void draw_background(void)
{
    // TODO

    /*    BSP_LCD_SelectLayer(LCD_LAYER_BG);
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

    BSP_LCD_DrawVLine(160, 0, 272);
    BSP_LCD_DrawVLine(320, 0, 272);

    BSP_LCD_DrawHLine(0, 90, 480);
    BSP_LCD_DrawHLine(0, 180, 480);*/
}

int main()
{
    draw_background();
    printBoard();
    playGame();
}