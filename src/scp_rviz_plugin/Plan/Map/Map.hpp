#ifndef MAP_HPP
#define MAP_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <map>
#include <set>

#include "Shape.hpp"

////////////////////////////////////////////////////
//                                    //ElementMap//
////////////////////////////////////////////////////

class Element
{
public:
    std::string name;

    Polygon originShape;
    std::vector<CoordD> originAnchors;

    CoordD panning;
    double rotation;
    Polygon shape;
    std::vector<CoordD> anchors;

    int id;
    bool isStatic;
    bool isAgent;
public:
    Element(Polygon &originShape, std::vector<CoordD> &originAnchors, CoordD &panning, double rotation);
    Element(){}
    Element(const Element &e);
    Element &operator=(const Element &e);
    void update();
    void setPanning(CoordD &panning);
    void setRotation(double &rotation);
    void setGeometry(CoordD &panning, double &rotation);
    void pan(CoordD &delta);
    void rotate(double &delta);
    void panAndRotate(CoordD &delta, double &rotation);
};

class ElementMap
{
public:
    std::vector<Element> elements;
    Element agent;
    double minX, minY, maxX, maxY;
    CoordSystem visualizeCS;
public:
    ElementMap(){}
    ElementMap(const std::string &mapfile);
};

////////////////////////////////////////////////////
//                                   //DistanceMap//
////////////////////////////////////////////////////

class DistanceElement
{
public:
    std::set<int> elementIds;
    double distance = std::numeric_limits<double>::max();
};

class DistanceElementMap
{
public:
    DistanceElement** distanceMap;
    CoordI size;
    double resolution;
    double minX, minY, maxX, maxY;
public:
    DistanceElementMap();
    DistanceElementMap(ElementMap &map, double resolution);
    ~DistanceElementMap();

    DistanceElementMap& operator=(const DistanceElementMap &distanceElementMap);
    DistanceElement* operator[](CoordI &index);
    DistanceElement* operator[](CoordD &position);
    DistanceElement* operator()(int x, int y);
    DistanceElement* operator()(double x, double y);
};

bool checkElementCollision(Element &e, std::vector<Element> &obstacles);

void genreateDistanceElementMap(ElementMap &map, double resolution, DistanceElementMap &distanceElementMap);

#endif // MAP_HPP