#ifndef _POSITIONS_H_
#define _POSITIONS_H_

constexpr int lid = 77;
constexpr int base = 67;
constexpr int lid_on_base = 37;

struct coordinates
{
    int x;
    int y;
};

// sorting Box Position
coordinates sortBox;
sortBox.x = 380;
sortBox.y = 210;

// assembly Box Positions
coordinates assemBox[8];
assemBox[0].x = 10;
assemBox[0].y = 95;
assemBox[1].x = 65;
assemBox[1].y = 95;
assemBox[2].x = 123;
assemBox[2].y = 95;
assemBox[3].x = 175;
assemBox[3].y = 95;

assemBox[4].x = 10;
assemBox[4].y = 155;
assemBox[5].x = 65;
assemBox[5].y = 155;
assemBox[6].x = 123;
assemBox[6].y = 155;
assemBox[7].x = 175;
assemBox[7].y = 155;

// reserve Box Positions
coordinates reserveBox[8];
reserveBox[0].x = 10;
reserveBox[0].y = 260;
reserveBox[1].x = 65;
reserveBox[1].y = 260;
reserveBox[2].x = 123;
reserveBox[2].y = 260;
reserveBox[3].x = 175;
reserveBox[3].y = 260;

reserveBox[4].x = 10;
reserveBox[4].y = 315;
reserveBox[5].x = 65;
reserveBox[5].y = 315;
reserveBox[6].x = 123;
reserveBox[6].y = 315;
reserveBox[7].x = 175;
reserveBox[7].y = 315;

#endif