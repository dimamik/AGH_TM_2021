#include <stdio.h>

#define PLAYER1 1 // circle
#define PLAYER2 -1 // cross

int whichMove = PLAYER1;
int gameBoard[3][3];

int* drawCircle(void){
    static int cursorPosition[2];

    // to delete !!
    int cursorX = 480;
    int cursorY = 272;

/*    BSP_LCD_SelectLayer(LCD_LAYER_FG);
    BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
    BSP_LCD_FillCircle( cursorX, cursorY, 36 );*/

    cursorPosition[0] = cursorX;
    cursorPosition[1] = cursorY;

    return cursorPosition;
}

int* drawCross(void){
    static int cursorPosition[2];

    // to delete !!
    int cursorX = 480;
    int cursorY = 272;


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


    cursorPosition[0] = cursorX;
    cursorPosition[1] = cursorY;

    return cursorPosition;
}

void initGameBoard(){
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            gameBoard[i][j] = 0;
        }
    }
}

int checkForEnd(){ // and if end draw the line marking the win
    for(int i = 0; i < 3; i++){
        if(gameBoard[i][0] == gameBoard[i][1] == gameBoard[i][2] != 0) {
            BSP_LCD_DrawHLine(0, 272 / (2*(3 - i)), 480); // line through the middle f.ex.: i = 0 (first row) => 272/2*3 = 45
            return gameBoard[i][0]; // it will be 1 or -1 (PLAYER1 or PLAYER2)
        }
        if(gameBoard[0][i] == gameBoard[1][i] == gameBoard[2][i] != 0) {
            BSP_LCD_DrawVLine(480 / (2*(3-i)), 0, 272);
            return gameBoard[0][i];
        }
    }
    if(gameBoard[0][0] == gameBoard[1][1] == gameBoard[2][2] != 0){
        Point lines[2];
        lines[0].X = 0;
        lines[0].Y = 272;

        lines[1].X = 480;
        lines[1].Y = 0;

        BSP_LCD_FillPolygon(lines, 2);
        return gameBoard[1][1];
    }

    if(gameBoard[0][2] == gameBoard[1][1] == gameBoard[2][0] != 0) {
        Point lines[2];
        lines[0].X = 0;
        lines[0].Y = 0;

        lines[1].X = 480;
        lines[1].Y = 272;

        BSP_LCD_FillPolygon(lines, 2);
        return gameBoard[1][1];
    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            if(gameBoard[i][j] == 0) return 0; // if there is no winner and in game board is free place
        }
    }

    return 2; // if there is no free place in game board and nobody won - it means draw
}

void fillGameBoard(const int* cursorPosition, int player){
    int cursorX = cursorPosition[0];
    int cursorY = cursorPosition[1];

    int x = cursorX / (480 / 3);
    int y = cursorY / (272 / 3);

    gameBoard[x][y] = player;
}

void gameLogic(){
    initGameBoard();
    while(checkForEnd() == 0){
        if(whichMove == PLAYER1){
            fillGameBoard(drawCircle(), PLAYER1);
        }
        else if(whichMove == PLAYER2){
            fillGameBoard(drawCross(), PLAYER2);
        }
        whichMove = -whichMove;
    }
}


// todo: Fajnie byłoby gdyby przy remisie czyściło plasznę i wypisywało "DRAW"
/* todo: Zastanowić się czy kółka i krzyżyki nie powinny być rysowane na warstwie background
    (bo teraz chyba są rysowane na fg, a ta warstwa jest stale odświeżana chyba, więc kółka i krzyżyki mogą znikać) */
/* todo: w sumie fajnie by było zrobić funkcję drawGameBoard() która wydrukuje stan tablicy gameBoard na putty'ego
    ale nie jest to konieczność - to bardziej dla celów diagnostycznych */

void draw_background(void)
{
/*    BSP_LCD_SelectLayer(LCD_LAYER_BG);
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

    BSP_LCD_DrawVLine(160, 0, 272);
    BSP_LCD_DrawVLine(320, 0, 272);

    BSP_LCD_DrawHLine(0, 90, 480);
    BSP_LCD_DrawHLine(0, 180, 480);*/

    gameLogic();
}


int main() {
    draw_background();

    return 0;
}