#ifndef _POSITIONS_H_
#define _POSITIONS_H_

struct Coordinates
{
    int x;
    int y;
};

constexpr int z_lid = 77;
constexpr int z_base = 67;
constexpr int z_lid_on_base = 37;

// sorting Box Position
inline constexpr Coordinates sortBox = {380, 210};

// assembly Box Positions
inline constexpr Coordinates assemBox[8] =
{
    {10, 95}, {65, 95}, {123, 95}, {175, 95},
    {10, 155}, {65, 155}, {123, 155}, {175, 155}
};

// reserve Box Positions
inline constexpr Coordinates reserveBox[8] =
{
    {10, 260}, {65, 260}, {123, 260}, {175, 260},
    {10, 315}, {65, 315}, {123, 315}, {175, 315}
};

#endif
