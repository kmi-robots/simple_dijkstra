#ifndef DIJKSTRA_MAP2D_H
#define DIJKSTRA_MAP2D_H

struct Map2D
{
    Map2D()
            : occupiedThresh(0.0)
            , freeThresh(0.0)
            , cellSizeX(0)
            , cellSizeY(0)
            , cells()  { }

    Map2D(double ot, double ft, int cx, int cy)
            : occupiedThresh(ot)
            , freeThresh(ft)
            , cellSizeX(cx)
            , cellSizeY(cy)
            , cells()  { }

    double occupiedThresh;
    double freeThresh;
    int cellSizeX;
    int cellSizeY;
    std::vector<uint8_t> cells;
}; // struct Map2D

#endif //DIJKSTRA_MAP2D_H
