#include "Map.hpp"
#include "json/json.h"
#include <fstream>

class _uniqueIdGenerator
{
public:
    int nextId = 0;
    std::vector<int> usedIds;
public:
    static _uniqueIdGenerator &getInstance()
    {
        static _uniqueIdGenerator instance;
        return instance;
    }
    int generateId()
    {
        int id = nextId;
        nextId++;
        usedIds.push_back(id);
        return id;
    }
};

Element::Element(Polygon &originShape, std::vector<CoordD> &originAnchors, CoordD &panning, double rotation)
{
    this->originShape = originShape;
    this->originAnchors = originAnchors;
    this->panning = panning;
    this->rotation = rotation;
    this->id = _uniqueIdGenerator::getInstance().generateId();
    update();
}

Element::Element(const Element &e)
{
    this->originShape = e.originShape;
    this->originAnchors = e.originAnchors;
    this->shape = e.shape;
    this->anchors = e.anchors;
    this->panning = e.panning;
    this->rotation = e.rotation;
    this->id = e.id;
}

Element &Element::operator=(const Element &e)
{
    this->originShape = e.originShape;
    this->originAnchors = e.originAnchors;
    this->shape = e.shape;
    this->anchors = e.anchors;
    this->panning = e.panning;
    this->rotation = e.rotation;
    this->id = e.id;
    return *this;
}

void Element::update()
{
    this->shape = originShape;
    this->anchors = originAnchors;
    for (int i = 0; i < shape.vertices.size(); i++)
    {
        shape.vertices[i].x = originShape.vertices[i].x * cos(rotation) - originShape.vertices[i].y * sin(rotation) + panning.x;
        shape.vertices[i].y = originShape.vertices[i].x * sin(rotation) + originShape.vertices[i].y * cos(rotation) + panning.y;
    }
    shape.edges.clear();
    for (int i = 0; i < shape.vertices.size(); i++)
    {
        shape.edges.push_back(Line(shape.vertices[i], shape.vertices[(i + 1) % shape.vertices.size()]));
    }
    for (int i = 0; i < anchors.size(); i++)
    {
        anchors[i].x = originAnchors[i].x * cos(rotation) - originAnchors[i].y * sin(rotation) + panning.x;
        anchors[i].y = originAnchors[i].x * sin(rotation) + originAnchors[i].y * cos(rotation) + panning.y; 
    }
}

void Element::setPanning(CoordD &panning)
{
    this->panning = panning;
    update();
}

void Element::setRotation(double &rotation)
{
    this->rotation = rotation;
    update();
}

void Element::setGeometry(CoordD &panning, double &rotation)
{
    this->panning = panning;
    this->rotation = rotation;
    update();
}

void Element::pan(CoordD &delta)
{
    this->panning += delta;
    update();
}

void Element::rotate(double &delta)
{
    this->rotation += delta;
    update();
}

void Element::panAndRotate(CoordD &delta, double &rotation)
{
    this->panning += delta;
    this->rotation += rotation;
    update();
}

ElementMap::ElementMap(const std::string &mapfile)
{
    std::ifstream scnFile(mapfile);
    if (!scnFile.is_open())
    {
        std::cerr << "Error: Cannot open scene file." << std::endl;
        return;
    }

    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(scnFile, root))
    {
        std::cerr << "Error: Cannot parse scene file." << std::endl;
        return;
    }

    scnFile.close();

    if (root["width"].isNull() || 
        root["height"].isNull() || 
        root["resolution"].isNull() ||
        root["primitives"].isNull() ||
        root["agents"].isNull())
    {
        std::cerr << "Error: Scene file is invalid." << std::endl;
        return;
    }

    this->minX = -root["width"].asDouble() / 2;
    this->minY = -root["height"].asDouble() / 2;
    this->maxX = root["width"].asDouble() / 2;
    this->maxY = root["height"].asDouble() / 2;

    Eigen::Matrix4d transform;
    transform << 1, 0, 0, -root["width"].asDouble() / 2,
                 0, -1, 0, root["height"].asDouble() / 2,
                 0, 0, -1, 0,
                 0, 0, 0, 1;
    this->visualizeCS = CoordSystem(transform);

    Json::Value agents = root["agents"];
    std::string agentName = agents[0].asString();
    Json::Value primitives = root["primitives"];
    for (int i = 0; i < primitives.size(); i++)
    {
        Json::Value primitive = primitives[i];
        if (primitive["name"].isNull() || 
            primitive["type"].isNull() || 
            primitive["center"].isNull() || 
            primitive["rotation"].isNull() || 
            primitive["vertices"].isNull() || 
            primitive["action"].isNull() ||
            (!primitive["anchor"].isNull() && primitive["anchor"].isNull()))
        {
            std::cerr << "Error: Primitive is invalid." << std::endl;
            continue;
        }

        std::string name = primitive["name"].asString();
        std::string type = primitive["type"].asString();
        CoordD center(primitive["center"][0].asDouble(), primitive["center"][1].asDouble());
        double rotation = primitive["rotation"].asDouble();
        std::vector<CoordD> vertices;
        for (int j = 0; j < primitive["vertices"].size(); j++)
        {
            CoordD vertex(primitive["vertices"][j][0].asDouble(), primitive["vertices"][j][1].asDouble());
            vertices.push_back(vertex);
        }
        std::string action = primitive["action"].asString();
        std::vector<CoordD> anchor;
        if (action != "None")
        {
            for (int j = 0; j < primitive["anchor"].size(); j++)
            {
                CoordD anchorPoint(primitive["anchor"][j][0].asDouble(), primitive["anchor"][j][1].asDouble());
                anchor.push_back(anchorPoint);
            }
        }
        Polygon P(vertices);
        Element element(
            P,
            anchor,
            center,
            rotation
        );

        if (name == agentName)
        {
            element.isAgent = true;
            element.isStatic = false;
            this->agent = element;
        }
        else
        {
            if (element.anchors.size() == 0)
            {
                element.isStatic = true;
                element.isAgent = false;
                this->elements.push_back(element);
            }
            else
            {
                element.isStatic = false;
                element.isAgent = false;
                this->elements.push_back(element);
            }
        }
    }
}
