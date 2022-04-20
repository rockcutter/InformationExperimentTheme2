#pragma once
#include <Siv3D.hpp>
#include <vector>
#include "MyVec.h"


//エイリアス
using Position = MyVec<double>;
using MovementVec = MyVec<double>;

class Graph {
private:
	int graphPosX;
	int graphPosY;
	int unitSize;
	int graphSize;
	std::vector<Line> graphLines;

	std::vector<Circle> circles;
	std::vector<Line> lines;
	
	void InitGraphLine();
	
public:
	Position ConvertPos(Position);
	Graph(Position pos, int unitSize, int graphSize);
	void InitObjects();
	void Put(Line);
	void Put(Circle);
	void Draw(Line, Color col = Palette::Black);
	void Draw(Circle, Color col = Palette::Black);
	void DrawArrow(Line, Color col = Palette::Black);
	void Show();
};

inline Position Graph::ConvertPos(Position pos) {
	return Position{ this->graphPosX + pos.first * unitSize, this->graphPosY - pos.second * unitSize };
}

inline void Graph::InitObjects() {
	this->circles.clear();
	this->lines.clear();
}
